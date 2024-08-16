#include <limits>

#include <gflags/gflags.h>
#include <sstream>
#include <string>
#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_point_cloud.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/perception/point_cloud_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/optimization/toppra.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Runs a simulation of the manipulation station plant as a stand-alone
// simulation which mocks the network inputs and outputs of the real robot
// station.  This is a useful test in the transition from a single-process
// simulation to operating on the real robot hardware.

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using drake::systems::TrajectorySource;
using drake::multibody::Toppra;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 10.0, "Simulation duration.");
DEFINE_string(setup, "manipulation_class",
              "Manipulation station type to simulate. "
              "Can be {manipulation_class, clutter_clearing}");
DEFINE_bool(publish_cameras, false,
            "Whether to publish camera images to LCM");
DEFINE_bool(publish_point_cloud, false,
            "Whether to publish point clouds to LCM.  Note that per issue "
            "https://github.com/RobotLocomotion/drake/issues/12125 the "
            "simulated point cloud data will have registration errors.");
DEFINE_string(iiwa_starting_position, "",
              "A comma-separated list of seven joint angles (in radians) "
              "to set the starting position of the iiwa arm. "
              "If empty, the default starting position is used.");
  
VectorXd parse_starting_position(const std::string& flag_value) {
  VectorXd starting_position(7);
  if (flag_value.empty()) {
    starting_position << 0, 0, 0, 0, 0, 0, 0;  // default starting position
  } else {
    std::istringstream iss(flag_value);
    std::string token;
    int i = 0;
    while (std::getline(iss, token, ',') && i < 7) {
      starting_position[i++] = std::stod(token);
    }
    if (i != 7) {
      throw std::runtime_error("Invalid iiwa_starting_position flag value. "
                               "Exactly 7 joint angles are required.");
    }
  }
  return starting_position;
}


int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  if (FLAGS_setup == "manipulation_class") {
    station->SetupManipulationClassStation();
    station->AddManipulandFromFile(
        "drake_models/manipulation_station/061_foam_brick.sdf",
        math::RigidTransform<double>(math::RotationMatrix<double>::Identity(),
                                     Eigen::Vector3d(0.6, 0, 0)));
  } else if (FLAGS_setup == "clutter_clearing") {
    station->SetupClutterClearingStation();
    station->AddManipulandFromFile(
        "drake_models/ycb/003_cracker_box.sdf",
        math::RigidTransform<double>(math::RollPitchYaw<double>(-1.57, 0, 3),
                                     Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else {
    throw std::domain_error(
        "Unrecognized station type. Options are "
        "{manipulation_class, clutter_clearing}.");
  }
  // TODO(russt): Load sdf objects specified at the command line.  Requires
  // #9747.

  // VectorXd pose_start(7); // Define the starting pose
  // VectorXd pose_end(7);   // Define the ending pose
  // pose_start << 0, -1, 0, -2, 0, 2, 0;
  // pose_end << 0, .5, 0, -2.25, 0, -1, 0;
  VectorXd pose_start(18); // Define the starting pose
  VectorXd pose_end(18); // Define the ending pose
  pose_start << 0, 0, 0, -1, 0, -2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  pose_end << 0, 0, 0, .5, 0, -2.25, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // Create a vector of times at which the poses are defined.
  std::vector<double> times = {0.0, FLAGS_duration};  // Start and end times

  // Create a vector of Eigen vectors (poses) for the trajectory.
  std::vector<MatrixXd> poses;
  poses.push_back(pose_start);
  poses.push_back(pose_end);
  

  // Create a PiecewisePolynomial trajectory from the given poses and times.
  auto trajectory_first_pass = PiecewisePolynomial<double>::FirstOrderHold(times, poses);

  

  station->Finalize();

  // Calculate grid points for the trajectory discretization.
  drake::multibody::CalcGridPointsOptions options;
  auto& plant = station->get_multibody_plant();
  const auto gridpoints = Toppra::CalcGridPoints(trajectory_first_pass, options);
  std::cout << "Path rows " << trajectory_first_pass.rows() << '\n';
  std::cout << "Plant.num_postitions " << plant.num_positions() << '\n';
  // std::cout << "Plant actuator names " << plant.GetPositionNames();
  std::cout << "Plant position names: ";
  const auto& position_names = plant.GetPositionNames();
  for (const auto& name : position_names) {
    std::cout << name << '\n';
  }
  std::cout << std::endl;

  Toppra toppra(trajectory_first_pass, plant, gridpoints);

  VectorXd lower_velocity_limit = VectorXd::Constant(plant.num_positions(), -1.0);
  VectorXd upper_velocity_limit = VectorXd::Constant(plant.num_positions(), 1.0);
  toppra.AddJointVelocityLimit(lower_velocity_limit, upper_velocity_limit);

  auto optimized_trajectory = toppra.SolvePathParameterization();

  // Create a TrajectorySource system that outputs the trajectory.
  if (optimized_trajectory) {
    // Create a TrajectorySource system that outputs the trajectory.
    auto trajectory_source = builder.AddSystem<drake::systems::TrajectorySource<double>>(*optimized_trajectory);
    builder.Connect(trajectory_source->get_output_port(), station->GetInputPort("iiwa_position"));
  } else {
    // Handle the case when the optimization fails.
    std::cerr << "TOPPRA optimization failed." << std::endl;
    return 1;
  }


  
  geometry::DrakeVisualizerd::AddToBuilder(
      &builder, station->GetOutputPort("query_object"));

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto iiwa_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  auto iiwa_command =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber->get_output_port(),
                  iiwa_command->get_message_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_command->get_position_measured_input_port());


  auto iiwa_status =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaStatusSender>();
  builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                  iiwa_status->get_position_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  iiwa_status->get_velocity_estimated_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status->get_torque_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  iiwa_status->get_torque_external_input_port());
  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

  // Receive the WSG commands.
  auto wsg_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm));
  auto wsg_command =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandReceiver>();
  builder.Connect(wsg_command_subscriber->get_output_port(),
                  wsg_command->GetInputPort("command_message"));
  builder.Connect(wsg_command->get_position_output_port(),
                  station->GetInputPort("wsg_position"));
  builder.Connect(wsg_command->get_force_limit_output_port(),
                  station->GetInputPort("wsg_force_limit"));

  // Publish the WSG status.
  auto wsg_status =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgStatusSender>();
  builder.Connect(station->GetOutputPort("wsg_state_measured"),
                  wsg_status->get_state_input_port());
  builder.Connect(station->GetOutputPort("wsg_force_measured"),
                  wsg_status->get_force_input_port());
  auto wsg_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm, 0.05 /* publish period */));
  builder.Connect(wsg_status->get_output_port(0),
                  wsg_status_publisher->get_input_port());

  // Publish the camera outputs.
  if (FLAGS_publish_cameras) {
    auto image_encoder = builder.AddSystem<
        systems::sensors::ImageToLcmImageArrayT>();
    for (const auto& camera_name : station->get_camera_names()) {
      // RGB
      const std::string rgb_name = "camera_" + camera_name + "_rgb_image";
      const auto& rgb_output = station->GetOutputPort(rgb_name);
      const auto& rgb_input =
          image_encoder->DeclareImageInputPort<
              systems::sensors::PixelType::kRgba8U>(rgb_name);
      builder.Connect(rgb_output, rgb_input);
      // Depth
      const std::string depth_name = "camera_" + camera_name + "_depth_image";
      const auto& depth_output = station->GetOutputPort(depth_name);
      const auto& depth_input =
          image_encoder->DeclareImageInputPort<
              systems::sensors::PixelType::kDepth16U>(depth_name);
      builder.Connect(depth_output, depth_input);
    }
    const double fps = 30.0;
    auto image_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<drake::lcmt_image_array>(
            "DRAKE_RGBD_CAMERA_IMAGES", lcm, 1.0 / fps));
    builder.Connect(image_encoder->image_array_t_msg_output_port(),
                    image_publisher->get_input_port());
  }

  // Publish the point clouds.
  if (FLAGS_publish_point_cloud) {
    for (const auto& camera_name : station->get_camera_names()) {
      const std::string cloud_name = "camera_" + camera_name + "_point_cloud";
      auto cloud_encoder = builder.AddSystem<perception::PointCloudToLcm>();
      const double fps = 5.0;
      auto cloud_publisher = builder.AddSystem(
          systems::lcm::LcmPublisherSystem::Make<drake::lcmt_point_cloud>(
              "DRAKE_POINT_CLOUD_" + camera_name, lcm, 1.0 / fps));
      builder.Connect(station->GetOutputPort(cloud_name),
                      cloud_encoder->get_input_port());
      builder.Connect(cloud_encoder->get_output_port(),
                      cloud_publisher->get_input_port());
    }
  }

  // builder.Connect(trajectory_source->get_output_port(),
  //               station->GetInputPort("iiwa_position"));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
//   Set the initial starting position if provided via command-line flag
  auto context = station->CreateDefaultContext();
  if(!FLAGS_iiwa_starting_position.empty()) {
    VectorXd starting_position = parse_starting_position(FLAGS_iiwa_starting_position);
    auto& station_context = diagram->GetMutableSubsystemContext(*station, &simulator.get_mutable_context());
    station->SetIiwaPosition(&station_context, starting_position);
  }
  VectorXd default_positions = plant.GetPositions(*context);
  std::cout << "Plant positions: ";
  for (const auto& position : default_positions) {
    std::cout << position << '\n';
  }
  std::cout << std::endl;

  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.AdvanceTo(FLAGS_duration);



  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
