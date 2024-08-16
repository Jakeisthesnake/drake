#include <gflags/gflags.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include <drake/systems/framework/vector_system.h>
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/optimization/toppra.h"


DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(sim_dt, 3e-3,
              "The time step to use for MultibodyPlant model "
              "discretization.");

namespace drake {
namespace examples {
namespace jakes_acrobot_sim {
namespace {

using multibody::MultibodyPlant;
using multibody::PackageMap;
using systems::Simulator;
using drake::trajectories::PiecewisePolynomial;
using drake::multibody::Toppra;


class PdController : public drake::systems::VectorSystem<double> {
 public:
  PdController() : VectorSystem<double>(4 /* input size */, 2 /* output size */, true),
  desired_trajectory_(nullptr) {
  }
  void SetDesiredTrajectory(const drake::trajectories::Trajectory<double>& trajectory) {
    desired_trajectory_ = &trajectory;
  }

 protected:
  // Override the virtual method from the VectorSystem class.
  // This method will provide the output computation for the PdController.
  void DoCalcVectorOutput(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const VectorX<double>>& input,
      const Eigen::VectorBlock<const VectorX<double>>& state,
      Eigen::VectorBlock<VectorX<double>>* output) const override {
    // Get the current state from the input port.
    // static_cast<void>(input);
    // static_cast<void>(context);
    double current_time = context.get_time();
    Eigen::VectorXd desired_state = this->desired_trajectory_->value(current_time);
    unused(state);


    // PD control law parameters.
    const double kp_1 = 10.0;  // Position gain.
    const double kd_1 = 0.0;   // Velocity gain.
    const double kp_2 = 10.0;  // Position gain.
    const double kd_2 = 0.0;   // Velocity gain.

    // Position error.
    const double q_desired_1 = desired_state[0];  // Desired position (radians).
    const double q_error_1 = q_desired_1 - input[0];
    const double q_desired_2 = desired_state[1];  // Desired position (radians).
    const double q_error_2 = q_desired_2 - input[1];

    // Velocity error.
    const double v_desired_1 = 0;  // Desired velocity (radians/sec).
    const double v_error_1 = v_desired_1 - input[2];
    const double v_desired_2 = 0;  // Desired velocity (radians/sec).
    const double v_error_2 = v_desired_2 - input[3];

    // Control signal (torque).
    const double control_1 = kp_1 * q_error_1 + kd_1 * v_error_1;
    const double control_2 = kp_2 * q_error_2 + kd_2 * v_error_2;
    // std::cout << "AAAAAAA\n";

    std::cout << "Control!!!" << control_1 << "Control_2 " << control_2 << '\n';
    // std::cout << "q_error_1 " << q_error_1 << " q_error_2 " << q_error_2 << '\n';
    std::cout << "input[0] " << input[0] << " input[1] " << input[1] << "input[2] " << input[2] << " input[3] " << input[3] << '\n';
    // std::cout << "q_error_1 " << q_error_1 << " q_error_2 " << q_error_2 << '\n';
    // std::cout << "v_error_1 " << v_error_1 << " v_error_2 " << v_error_2 << "\n\n";
    // std::cout << "q_desired_1 " << q_desired_1 << " q_desired_2 " << q_desired_2 << '\n';


    // Write the control signal to the output vector.
    if (control_1 < -5){
      (*output)[0] = -5;
    }
    else if (control_1 > 5){
      (*output)[0] = 5;
    }
    else{
      (*output)[0] = control_1;
      std::cout << "c1\n";
    }

    if (control_2 < -5){
      (*output)[1] = -5;
    }
    else if (control_2 > 5){
      (*output)[1] = 5;
    }
    else{
      (*output)[1] = control_2;
      std::cout << "c2\n";
    }
    // (*output)[0] = control_1;
    // (*output)[1] = control_2;
  }

  

 private:
 const drake::trajectories::Trajectory<double>* desired_trajectory_; 
};


std::pair<drake::trajectories::PiecewisePolynomial<double>, std::vector<double>> MakeLinearTrajectory(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& end,
    double total_time,
    int num_steps) {
  // Calculate the time interval for each step.
  double time_interval = total_time / num_steps;

  // Create a vector to store the times at which the waypoints are defined.
  std::vector<double> times;
  for (int i = 0; i <= num_steps; ++i) {
    times.push_back(i * time_interval);
  }

  // Create a vector to store the waypoints.
  std::vector<Eigen::MatrixXd> waypoints;
  for (int i = 0; i <= num_steps; ++i) {
    // Linearly interpolate between the start and end points.
    Eigen::VectorXd waypoint = start + (end - start) * (i / static_cast<double>(num_steps));
    waypoints.push_back(waypoint);
  }

  // Create the PiecewisePolynomial trajectory.
  auto trajectory = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(times, waypoints);

  // Return both the trajectory and the times vector using std::make_pair.
  return std::make_pair(trajectory, times);
}

int DoMain() {
  systems::DiagramBuilder<double> builder;
  // Adds a plant.
  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(
      &builder, FLAGS_sim_dt);
  const char* kModelUrl =
      "/home/jake/drake/examples/jake/jakes_acrobot.sdf";

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : kModelUrl);
//   auto iiwa_instance =
      (void)multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  
  plant.Finalize();


  // Define the start and end points of the trajectory.
  // Eigen::VectorXd start(3);
  // Eigen::VectorXd end(3);
  // start << 2, 0, 0;
  // end << -1.75, 0.1, 0;
  Eigen::VectorXd pose_start(3);
  Eigen::VectorXd pose_end(3);
  pose_start << 2, 0, 0;
  pose_end << -1.9, .1, 0;

  // Create a matrix where each column represents a waypoint for the trajectory.
  Eigen::MatrixXd waypoints(3, 2);  // 3 dimensions, 2 waypoints (start and end)
  waypoints.col(0) = pose_start;
  waypoints.col(1) = pose_end;

  std::vector<Eigen::MatrixXd> waypoints_vector;
  waypoints_vector.push_back(waypoints.col(0));
  waypoints_vector.push_back(waypoints.col(1));

  

  // Define the start and end times for the trajectory.
  // double start_time = 0.0;  // Start at time t = 0
  double end_time = 3.0;   // End at time t = 10, for example
  // Create a vector of times corresponding to each waypoint.

  int num_steps = 10; // For example, change this to the desired number of steps.
  auto [linear_trajectory, times] = MakeLinearTrajectory(pose_start, pose_end, end_time, num_steps);

  // Step 2: Solve the IK problem at each time step to create a trajectory for the actuators.
  std::vector<Eigen::MatrixXd> joint_angles_trajectory;
  
  
  Eigen::VectorXd q_nominal(plant.num_positions());
  // Set q_nominal to the desired nominal pose for your robot.
  // For example, if you want all joints to be at zero in the nominal pose:
  // q_nominal.setZero();
  q_nominal << 0, 0;


  // Define the weight matrix for the regularization cost.
  // A diagonal matrix with positive entries can be used to weigh the joints differently.
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(plant.num_positions(), plant.num_positions());
  // Set the diagonal values of W to the desired weights.
  // For example, if you want equal weights for all joints:
  double regularization_weight = 1.0;  // This can be tuned to your preference.
  W = regularization_weight * W;

  Eigen::VectorXd joint_lower_bounds(plant.num_positions());
  Eigen::VectorXd joint_upper_bounds(plant.num_positions());

  // Set the lower and upper bounds for each joint.
  // For example, if all joints have a range of [-pi, pi]:
  joint_lower_bounds.fill(-M_PI - 10.1);
  joint_upper_bounds.fill(M_PI + 10.1);

  // int num_segments = linear_trajectory.get_number_of_segments();


  // Loop over each time step to solve the IK problem.
  for (double t : times){
  // for (int segment_index = 0; segment_index < num_segments; ++segment_index) {
  //   double segment_start_time = linear_trajectory.start_time(segment_index);

  //   // You can solve the IK at the start and end of each segment, or at additional points in between.
  //   // For simplicity, this example only solves at the start of each segment.
  //   double t = segment_start_time;
    // Create an IK solver instance.
    drake::multibody::InverseKinematics ik(plant);
    // Evaluate the desired end-effector position at time t.
    Eigen::VectorXd desired_position = linear_trajectory.value(t);
    // std::cout << "q_nominal: [";
    // for (int i = 0; i < q_nominal.size(); ++i) {
    //   std::cout << q_nominal[i];
    //   if (i < q_nominal.size() - 1) {
    //     std::cout << ", ";
    //   }
    // }
    // std::cout << "]\n"; 
    // std::cout << "desired_position: [";
    // for (int i = 0; i < desired_position.size(); ++i) {
    //   std::cout << desired_position[i];
    //   if (i < desired_position.size() - 1) {
    //     std::cout << ", ";
    //   }
    // }
    // std::cout << "]\n"; 



    // Specify the desired position constraint for the IK problem.
    const auto& end_effector_frame = plant.GetFrameByName("end_effector");
    constexpr double ik_error{.01};
    ik.AddPositionConstraint(end_effector_frame, Eigen::Vector3d::Zero(),  // Local position in end effector frame.
                             plant.world_frame(), desired_position - Eigen::Vector3d::Constant(ik_error),  // Desired position in world frame.
                             desired_position + Eigen::Vector3d::Constant(ik_error));
    auto q = ik.q(); // The decision variables for the IK problem.
    ik.get_mutable_prog()->AddQuadraticCost((q - q_nominal).transpose() * W * (q - q_nominal));
    ik.get_mutable_prog()->AddBoundingBoxConstraint(q_nominal + joint_lower_bounds, q_nominal + joint_upper_bounds, ik.q());
    ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_nominal);

    int max_retries = 5; // Maximum number of retries
    int attempt = 0; // Current attempt number
    bool satisfactory = false; // Flag to indicate if the solution is satisfactory
    Eigen::VectorXd joint_angles{};

    while (attempt < max_retries && !satisfactory) {
      // Solve the IK problem
      drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(ik.prog());

      if (result.is_success()) {
          // Check if the solution is satisfactory based on your criteria
          joint_angles = result.GetSolution(ik.q());
          int n_0 = std::floor((M_PI - joint_angles[0] + q_nominal[0])/(2 * M_PI));
          int n_1 = std::floor((M_PI - joint_angles[1] + q_nominal[1])/(2 * M_PI));
          // std::cout << "Joint angles try: [";
          // for (int i = 0; i < joint_angles.size(); ++i) {
          //   std::cout << joint_angles[i];
          //   if (i < joint_angles.size() - 1) {
          //     std::cout << ", ";
          //   }
          // }
          // std::cout << "],   n [" << n_0 << ", " << n_1 << '\n';
          // std::cout << "q_nominal: [";
          // for (int i = 0; i < q_nominal.size(); ++i) {
          //   std::cout << q_nominal[i];
          //   if (i < q_nominal.size() - 1) {
          //     std::cout << ", ";
          //   }
          // }
          // std::cout << "]\n"; 
          if ((n_0 == 0) && (n_1 == 0)){
            satisfactory = true;
            // std::cout << "true\n";
            break;
          }
          else{
            q_nominal << 2 * M_PI * n_0 + joint_angles[0], 2 * M_PI * n_1 + joint_angles[1];
            ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_nominal);
            // std::cout << "false\n";
          }
      }
      else{
        break; //could randomize q_nominal
      }
      attempt++; // Increment the attempt counter
    }

    if (satisfactory) {
      // Extract the joint angles from the solution.
      joint_angles_trajectory.push_back(joint_angles);
      // std::cout << "Joint angles: [";
      // for (int i = 0; i < joint_angles.size(); ++i) {
      //   std::cout << joint_angles[i];
      //   if (i < joint_angles.size() - 1) {
      //     std::cout << ", ";
      //   }
      // }
      // std::cout << "]\n"; 
      q_nominal = joint_angles;
    } else {
      // Handle the case where the IK solution was not successful.
      std::cerr << "Inverse kinematics failed at time " << t << std::endl;
      // You might choose to break the loop or handle this situation differently.
    }
  }
  

  std::vector<Eigen::VectorXd> velocities_at_knots;

  // Step 3: Construct a new trajectory for the joint angles.
  // You can use a PiecewisePolynomial or another suitable trajectory type.
  auto joint_angles_trajectory_poly =
    drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
        times, joint_angles_trajectory);

  int num_waypoints = joint_angles_trajectory.size();
  // Step 1: Create the path as a Trajectory<double> object.
  Eigen::VectorXd s_values(num_waypoints);
  for (int i = 0; i < num_waypoints; ++i) {
    s_values(i) = static_cast<double>(i) / (num_waypoints - 1);  // This will create a vector from 0 to 1.
  }

  // Convert waypoints to a matrix for PiecewisePolynomial.
  Eigen::MatrixXd waypoints_matrix(joint_angles_trajectory[0].size(), num_waypoints);
  for (int i = 0; i < num_waypoints; ++i) {
    waypoints_matrix.col(i) = joint_angles_trajectory[i];
  }

  // Create a PiecewisePolynomial using a cubic spline interpolation.
  auto q_s = drake::trajectories::PiecewisePolynomial<double>::CubicShapePreserving(
      s_values, waypoints_matrix);




  // Step 2: Calculate grid points.
  drake::multibody::CalcGridPointsOptions grid_options;
  grid_options.max_err = 1e-3; // Example value, adjust as needed.
  grid_options.max_iter = 100; // Example value, adjust as needed.
  grid_options.max_seg_length = 0.05; // Example value, adjust as needed.
  grid_options.min_points = 100; // Example value, adjust as needed.
  Eigen::VectorXd gridpoints = Toppra::CalcGridPoints(joint_angles_trajectory_poly, grid_options);

  // Step 3: Instantiate the Toppra class.
  drake::multibody::Toppra toppra(joint_angles_trajectory_poly, plant, gridpoints);

  // Step 4: Add constraints.
  Eigen::VectorXd joint_velocity_lower_limit(plant.num_velocities());
  Eigen::VectorXd joint_velocity_upper_limit(plant.num_velocities());
  joint_velocity_lower_limit << -10, -10;
  joint_velocity_upper_limit << 10, 10;
  // ... Set your joint velocity limits ...
  toppra.AddJointVelocityLimit(joint_velocity_lower_limit, joint_velocity_upper_limit);

  Eigen::VectorXd joint_torque_lower_limit(plant.num_velocities());
  Eigen::VectorXd joint_torque_upper_limit(plant.num_velocities());
  joint_torque_lower_limit << -5, -5;
  joint_torque_upper_limit << 5, 5;
  // ... Set your joint acceleration limits ...
  toppra.AddJointTorqueLimit(joint_torque_lower_limit, joint_torque_upper_limit);

  // ... Add more constraints as needed ...

  // Step 5: Solve the optimization problem.
  std::optional<PiecewisePolynomial<double>> new_joint_angles_poly = toppra.SolvePathParameterization();

  // ... [Rest of your code] ...

  // Use s_dot_optimized to create a time-parameterized trajectory q(t) if optimization succeeded.
  if (!new_joint_angles_poly) {
    std::cerr << "Toppra pooped\n";
  }
      

    // Add a PD controller to the system.
  auto pd_controller = builder.AddSystem<PdController>();
  


  builder.Connect(plant.get_state_output_port(), pd_controller->get_input_port());
  
  builder.Connect(pd_controller->get_output_port(), plant.get_actuation_input_port());


  // Add a VectorLogSink to log the output of the PD controller.
  // auto pd_output_log = LogVectorOutput(plant.get_state_output_port(), &builder);

  // Creates and adds LCM publisher for visualization.
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  visualization::ApplyVisualizationConfig({}, &builder, nullptr, nullptr,
                                          nullptr, nullptr, lcm);


  

  auto sys = builder.Build();


  // Set the trajectory for the PD controller.
  if (new_joint_angles_poly) {
    // If it does, dereference it and pass it to SetDesiredTrajectory
    pd_controller->SetDesiredTrajectory(*new_joint_angles_poly);
  } else {
    // Handle the case where new_joint_angles_poly does not contain a value
    // This might include logging an error, throwing an exception, or other error handling
    std::cerr << "Toppra optimization failed to produce a trajectory." << std::endl;
    return 1; // Return a non-zero value to indicate failure
  }



  Simulator<double> simulator(*sys);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  simulator.Initialize();


  


  // // Define the force vector in the world frame.
  // // Define the force vector in the world frame.
  // const Eigen::Vector3d force_vector(0.0, 0.0, 0.0); // 1 N in the x-direction.
  // const Eigen::Vector3d torque_vector(0.0, 10.0, 0.0); // No torque.
  // const Eigen::Vector3d point_of_application(0.0, 0.0, 10.0); // Center of mass.

  // drake::multibody::SpatialForce<double> spatial_force(torque_vector, force_vector);

  // // Define the force application event.
  // auto& link1 = plant.GetBodyByName("Link2");
  // drake::multibody::ExternallyAppliedSpatialForce<double> external_force;
  // external_force.F_Bq_W = spatial_force;
  // external_force.p_BoBq_B = point_of_application;
  // external_force.body_index = link1.index();

  // auto& plant_context = plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  // // Apply the force at the beginning of the simulation.
  // plant.get_applied_spatial_force_input_port().template FixValue<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(&plant_context, {external_force});

  // Run the simulation for 1 second to apply the force.
  simulator.AdvanceTo(20.0);



  // // Remove the force after 1 second by setting an empty vector.
  // plant.get_applied_spatial_force_input_port().template FixValue<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>(&plant_context, {});

  // // Continue the simulation.
  // simulator.AdvanceTo(FLAGS_simulation_sec);
  // const auto& log = pd_output_log->FindLog(simulator.get_context());
  // // Print the log data to the terminal.
  // const auto& times = log.sample_times();
  // const auto& data = log.data();
  // for (int i = 0; i < log.num_samples(); ++i) {
  //   std::cout << times[i]; // Print the sample time.
  //   for (int j = 0; j < data.rows(); ++j) { // Use data.rows() to get the number of channels
  //     std::cout << " " << data(j, i); // Print each data point in the sample.
  //   }
  //   std::cout << std::endl; // Newline after each sample.
  // }

  return 0;
}



}  // namespace
}  // namespace jakes_acrobot_sim
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::jakes_acrobot_sim::DoMain();
}
