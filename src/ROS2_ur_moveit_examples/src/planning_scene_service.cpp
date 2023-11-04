
// example no 4
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  RCLCPP_ERROR(logger, "getDefaultPlanningPipelineId: %s", move_group_interface.getDefaultPlanningPipelineId().c_str()); 

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = 0.3;
  goal_pose.pose.position.y = 0.2;
  goal_pose.pose.position.z = 0.25;
  goal_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::Constraints pose_start_goal = kinematic_constraints::constructGoalConstraints("tool0", goal_pose, 1e-2, 1e-2);
  std::vector<moveit_msgs::msg::Constraints> pose_start_goals;
  pose_start_goals.push_back(pose_start_goal);

  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr client = node->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto request = std::make_shared<moveit_msgs::srv::GetMotionPlan_Request>();

  request->motion_plan_request.goal_constraints = pose_start_goals;
  request->motion_plan_request.planner_id;
  request->motion_plan_request.group_name = "ur_manipulator";
  request->motion_plan_request.num_planning_attempts = 2;
  request->motion_plan_request.allowed_planning_time = 1.0;
  request->motion_plan_request.max_velocity_scaling_factor = 1.0;
  request->motion_plan_request.max_acceleration_scaling_factor = 1.0;

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call service success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_=result.get()->motion_plan_response.trajectory;

  RCLCPP_ERROR(logger, "Executing");
  move_group_interface.execute(plan);

  // Shutdown ROS
  rclcpp::shutdown();
  // spinner.join();
  return 0;
}