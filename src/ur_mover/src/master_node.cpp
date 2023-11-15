#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "rclcpp/rclcpp.hpp"
#include "ur_custom_interfaces/msg/ur_command.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

using moveit::planning_interface::MoveGroupInterface;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(std::shared_ptr<rclcpp::Node> move_group_node, geometry_msgs::msg::Pose* lookout_pos)
    : Node("master_node"), is_lookout_position(false), is_horizontally_centered(false), 
    is_vertically_centered(false), is_moving(false), lookout_pos(lookout_pos), target_pose(*lookout_pos)
    {
      RCLCPP_INFO(this->get_logger(), "Node started. Awaiting commands...");
      subscription_ = this->create_subscription<ur_custom_interfaces::msg::URCommand>(
      "custom_camera", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

      move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_node, "ur_manipulator");
      auto const robot_pos = move_group_->getCurrentPose("wrist_3_link");
      RCLCPP_INFO(this->get_logger(), "Robot position: %f, %f, %f", robot_pos.pose.position.x, robot_pos.pose.position.y, robot_pos.pose.position.z);
      // this->move_to_lookout_position();
    }

  private:
    void topic_callback(const ur_custom_interfaces::msg::URCommand::SharedPtr msg)
    {
      int x = std::stoi(msg->x);
      if(is_moving || !is_lookout_position){
        RCLCPP_INFO(this->get_logger(), "Robot is already moving. Ignoring command.");
        return;
      }
      if(x == 0){
        RCLCPP_INFO(this->get_logger(), "Robot is already centered horizontally. Ignoring command.");
        is_horizontally_centered = true;
      }

      if(x == 1 && !is_moving){
        target_pose.position.x += 0.1;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;


        double eef_step = 0.01; // Rozdzielczość trajektorii
        // move_group_interface.setPoseTarget(waypoint);
        auto res = move_group_->computeCartesianPath(std::vector<geometry_msgs::msg::Pose> {target_pose}, eef_step, 0.0, my_plan.trajectory_);

        if (res != -1) {
            is_moving = true;
            move_group_->execute(my_plan);
            is_moving = false;
            is_lookout_position = true;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed for the waypoint.");
        }
      } else if(x == -1 && !is_moving){
        target_pose.position.x -= 0.1;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;


        double eef_step = 0.01; // Rozdzielczość trajektorii
        // move_group_interface.setPoseTarget(waypoint);
        auto res = move_group_->computeCartesianPath(std::vector<geometry_msgs::msg::Pose> {target_pose}, eef_step, 0.0, my_plan.trajectory_);

        if (res != -1) {
            is_moving = true;
            move_group_->execute(my_plan);
            is_moving = false;
            is_lookout_position = true;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed for the waypoint.");
        }
      }
      
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", curr_pos.pose.position.x);
    }

    void move_to_lookout_position(){
      RCLCPP_INFO(this->get_logger(), "Moving to lookout position...");

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;


      double eef_step = 0.01; // Rozdzielczość trajektorii
        // move_group_interface.setPoseTarget(waypoint);
        auto res = move_group_->computeCartesianPath(std::vector<geometry_msgs::msg::Pose> {*lookout_pos}, eef_step, 0.0, my_plan.trajectory_);

        if (res != -1) {
            is_moving = true;
            move_group_->execute(my_plan);
            is_moving = false;
            is_lookout_position = true;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed for the waypoint.");
        }

      RCLCPP_INFO(this->get_logger(), "Arrived at lookout position.");
    }
    rclcpp::Subscription<ur_custom_interfaces::msg::URCommand>::SharedPtr subscription_;
    bool is_lookout_position;
    bool is_horizontally_centered;
    bool is_vertically_centered;
    bool is_moving;
    geometry_msgs::msg::Pose* lookout_pos;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    geometry_msgs::msg::Pose target_pose;
};

int main(int argc, char * argv[])
{
  geometry_msgs::msg::Pose lookout_pos;
  lookout_pos.orientation.w = 1.0;
  lookout_pos.orientation.x = 0.027;
  lookout_pos.orientation.y = 2.203;
  lookout_pos.orientation.z = -2.214;
  lookout_pos.position.x = -0.138;
  lookout_pos.position.y = -0.264;
  lookout_pos.position.z = 0.22;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_robot_node = rclcpp::Node::make_shared("move_robot", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_robot_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  rclcpp::spin(std::make_shared<MinimalPublisher>(move_robot_node, &lookout_pos));
  // rclcpp::spin(move_robot);
  rclcpp::shutdown();
  return 0;
}