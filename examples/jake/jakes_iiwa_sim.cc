

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
// #include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
// #include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
// #include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(torque_control, false, "Simulate using torque control mode.");
DEFINE_double(sim_dt, 3e-3,
              "The time step to use for MultibodyPlant model "
              "discretization.");

namespace drake {
namespace examples {
namespace jakes_iiwa {
namespace {
using multibody::MultibodyPlant;
using multibody::PackageMap;
using systems::Context;
using systems::Demultiplexer;
using systems::StateInterpolatorWithDiscreteDerivative;
using systems::Simulator;
using systems::controllers::InverseDynamicsController;
using systems::controllers::StateFeedbackControllerInterface;

int DoMain() {
  systems::DiagramBuilder<double> builder;
  // Adds a plant.
  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(
      &builder, FLAGS_sim_dt);
  const char* kModelUrl =
      "/home/jake/drake/examples/jake/jakes_iiwa.sdf";

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : PackageMap{}.ResolveUrl(kModelUrl));
  // auto iiwa_instance =
      (void)multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();


  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  visualization::ApplyVisualizationConfig({}, &builder, nullptr, nullptr,
                                          nullptr, nullptr, lcm);

  // Since we welded the model to the world above, the only remaining joints
  // should be those in the arm.
  const int kIiwaArmNumJoints{7};
  const int num_joints = plant.num_positions();
  DRAKE_DEMAND(num_joints % kIiwaArmNumJoints == 0);
  // const int num_iiwa = num_joints / kIiwaArmNumJoints;

  
  return 0;
}

}  // namespace
}  // namespace jakes_iiwa
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::jakes_iiwa::DoMain();
}
