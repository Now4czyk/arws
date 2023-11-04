
// example no 3
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  const auto &LOGGER = node->get_logger();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  for (const auto& jn : joint_names) {
    RCLCPP_INFO_STREAM(LOGGER, "Joint name: " << jn );  
  }

  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                         << " self collision");

  moveit::core::RobotState &current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "Test 2: Current state is " << (collision_result.collision ? "in" : "not in")
                                                         << " self collision");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
