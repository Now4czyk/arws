#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ur_custom_interfaces/msg/ur_command.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

using moveit::planning_interface::MoveGroupInterface;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RobotMasterController : public rclcpp::Node
{
  public:
    RobotMasterController(std::shared_ptr<rclcpp::Node> move_group_node, geometry_msgs::msg::Pose* lookout_pos, geometry_msgs::msg::Pose* apple_drop_pos)
    : Node("master_node"), is_lookout_position(false), is_horizontally_centered(false), 
    is_vertically_centered(false), is_moving(false), lookout_pos(lookout_pos), target_pose(*lookout_pos), prev_x(0),
    is_depth_reached(false), was_centered_message_shown(false), depth(0.0), apple_drop_pose(apple_drop_pose), is_at_apple_position(false), 
    is_apple_grabbed(false), is_apple_picked(false), is_with_apple_at_lookout_position(false)
    {
      RCLCPP_INFO(this->get_logger(), "Node started. Awaiting commands...");
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      RCLCPP_INFO(this->get_logger(), "SETUP LOGS");
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      subscription_ = this->create_subscription<ur_custom_interfaces::msg::URCommand>(
      "custom_camera", 1, std::bind(&RobotMasterController::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to /custom_camera topic");
      publisher = this->create_publisher<std_msgs::msg::String>("/custom_gripper", 1);
      RCLCPP_INFO(this->get_logger(), "Publisher on custom_gripper created");


      auto message = std_msgs::msg::String();
      message.data = "open";
      rclcpp::sleep_for(1s);
      publisher->publish(message);
      rclcpp::sleep_for(5s);



      move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_node, "ur_manipulator");
      // move_group_->setMaxVelocityScalingFactor(0.1);
      auto const robot_pos = move_group_->getCurrentPose("wrist_3_link");
      RCLCPP_INFO(this->get_logger(), "Robot position: %f, %f, %f", robot_pos.pose.position.x, robot_pos.pose.position.y, robot_pos.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Robot rotation: %f, %f, %f, %f", 
      robot_pos.pose.orientation.x, robot_pos.pose.orientation.y, robot_pos.pose.orientation.z, robot_pos.pose.orientation.w
      );

      RCLCPP_INFO(this->get_logger(), "Lookout position: %f, %f, %f", lookout_pos->position.x, lookout_pos->position.y, lookout_pos->position.z);
      this->move_to_lookout_position();
    }

  private:
    void topic_callback(const ur_custom_interfaces::msg::URCommand::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      int x = std::stoi(msg->x);
      int y = std::stoi(msg->y);
      if(!is_moving) {
        depth = sanitize_depth(msg->depth);
      }
      RCLCPP_INFO(this->get_logger(), "Received commands: x:%i, y: %i, depth: %f", x, y, depth);

      if(depth > 0.0 && depth < 0.5){
        depths.push_back(depth);
      }

      if(is_depth_reached && is_horizontally_centered && is_vertically_centered) {
        RCLCPP_INFO(this->get_logger(), "At apple position");
        return;
      }
      
      if(is_moving || !is_lookout_position){
        RCLCPP_INFO(this->get_logger(), "Robot is already moving. Ignoring command.");
        return;
      }
      if(x == 0 && !is_horizontally_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is already centered horizontally. Ignoring command.");
        is_horizontally_centered = true;
        is_moving = false;
        move_group_->stop();
        return;
      }

      if(x == 1 && !is_moving && !is_horizontally_centered){
        target_pose.position.x += 0.01;
        this->move(target_pose, "Moving robot to the right");

      } else if(x == -1 && !is_moving && !is_horizontally_centered){
        target_pose.position.x -= 0.01;
        this->move(target_pose, "Moving robot to the left");
      }

      // MOVING IN Y

      if(!is_horizontally_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is not centered horizontally. Ignoring command.");
        return;
      }

      if(y == 0 && !is_vertically_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is already centered vertically. Ignoring command.");
        is_vertically_centered = true;
        move_group_->stop();
        is_moving = false;
        return;
      }

      if(y == 1 && !is_moving && !is_vertically_centered){
        target_pose.position.z -= 0.01;
        this->move(target_pose, "Moving robot to the bottom");
      } else if(y == -1 && !is_moving && !is_vertically_centered){
        target_pose.position.z += 0.01;
        this->move(target_pose, "Moving robot to the top");
      }
      
      bool const is_robot_centered = is_horizontally_centered && is_vertically_centered;
      if(is_robot_centered && !was_centered_message_shown) {
        RCLCPP_INFO(this->get_logger(), "Robot is centered. Started timer.");
        was_centered_message_shown = true;
        end_timer = this->get_clock()->now() + rclcpp::Duration(3s);
      }
      else if(!is_robot_centered) {
        return;
      }
      timer = this->get_clock()->now();

      if(timer < end_timer) {
        return;
      }

      if(depth < 0.01) {
        RCLCPP_INFO(this->get_logger(), "Depth too small. Awaiting another reading.");
        return;
      }
      if (depth > 0.8) {
        RCLCPP_INFO(this->get_logger(), "Depth too big. Awaiting another reading.");
        return;
      }

      // Reaching the apple
      if(!is_moving && !is_depth_reached) {
        // float depth_calc = 0;
        // for(auto const &d : depths){
        //   depth_calc += d;
        // }
        // depth_calc /= depths.size();

        // including camera offset
        target_pose.position.z += 0.14;
        bool const offset_res = this->move(target_pose, "Applying camera offset");
        if(offset_res){
          RCLCPP_INFO(this->get_logger(), "Applied camera offset");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not apply camera offset. Shutting down.");
          rclcpp::shutdown();
        }
        
        RCLCPP_INFO(this->get_logger(), "Moving robot forward by %f", depth);
        float camera_offset = 0.11;
        float gripper_offset = 0.14;
        target_pose.position.y += depth - camera_offset - gripper_offset;
        // shouldnt be hardcoded
        target_pose.position.x -= 0.03;

        bool const forward_res = this->move(target_pose, "Moving robot forward");

        if(forward_res){
          is_depth_reached = true;
          RCLCPP_INFO(this->get_logger(), "Arrived at apple position.");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not arrive at apple position. Shutting down.");
          rclcpp::shutdown();
        }
      }

      // Grabbing the apple
      if(is_depth_reached && !is_apple_grabbed){
        RCLCPP_INFO(this->get_logger(), "About to close gripper");
        rclcpp::sleep_for(1s);
        publisher->publish(std_msgs::msg::String().set__data("close"));
        rclcpp::sleep_for(5s);
        RCLCPP_INFO(this->get_logger(), "Gripper closed");
        is_apple_grabbed = true;
      }


      // Picking the apple
      if(is_apple_grabbed && !is_moving){
        target_pose.position.z += 0.03;
        target_pose.position.y -= 0.07;
        bool const backward_res = this->move(target_pose, "Picking the apple");
        if(backward_res){
          is_apple_picked = true;
          RCLCPP_INFO(this->get_logger(), "Position after picking an apple");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not pick an apple. Shutting down.");
          rclcpp::shutdown();
        }
      }

          RCLCPP_INFO(this->get_logger(), "Before going to lookout position with apple");
      // Going back to lookout position with apple
      if(is_apple_picked && !is_moving){
        this->move_to_lookout_position();
          RCLCPP_INFO(this->get_logger(), "Going to lookout position");
        is_with_apple_at_lookout_position = true;
      }
          RCLCPP_INFO(this->get_logger(), "After going to lookout position with apple");

      // Moving to drop apple position & dropping the apple
      if(is_with_apple_at_lookout_position && !is_moving){
          RCLCPP_INFO(this->get_logger(), "IN IF");
        bool const apple_lookout_pose_res = this->move(*lookout_pos, "Moving to apple drop position");

          RCLCPP_INFO(this->get_logger(), "AFTER MOVE");
        if(apple_lookout_pose_res){
          RCLCPP_INFO(this->get_logger(), "Arrived at apple drop position.");
          rclcpp::sleep_for(1s);
          publisher->publish(std_msgs::msg::String().set__data("open"));
          rclcpp::sleep_for(5s);
          reset_robot_loop();
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not arrive at apple drop position. Shutting down.");
          rclcpp::shutdown();
        }
      }
    }

    void move_to_lookout_position(){
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      bool const move_res = move(*lookout_pos, "Moving to lookout position");
      if(move_res){
        is_lookout_position = true;
        RCLCPP_INFO(this->get_logger(), "Arrived at lookout position.");
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Could not arrive at lookout position. Shutting down.");
        rclcpp::shutdown();
      }

    }

    void reset_robot_loop(){
      is_lookout_position = false;
      is_horizontally_centered = false;
      is_vertically_centered = false;
      is_moving = false;
      is_depth_reached = false;
      is_at_apple_position = false;
      is_apple_grabbed = false;
      is_apple_picked = false;
      is_with_apple_at_lookout_position = false;
      was_centered_message_shown = false;
      depths.clear();
      prev_x = 0;
      this->move_to_lookout_position();
    }

    float sanitize_depth(std::string raw_depth){
      float depth = std::stof(raw_depth) / 1000;
      if(depth > 0.8){
        depth = 0.8;
      }
      else if(depth < 0){
        depth = 0;
      }
      return depth;
    }

    bool move(geometry_msgs::msg::Pose target_pose, const char * log_message = "Moving robot"){
      is_moving = true;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group_->setEndEffectorLink("wrist_3_link");

      double eef_step = 0.01; // Rozdzielczość trajektorii
      auto res = move_group_->computeCartesianPath(std::vector<geometry_msgs::msg::Pose> {target_pose}, eef_step, 0.0, my_plan.trajectory_);
      RCLCPP_INFO(this->get_logger(), log_message);

      if (res != -1) {
        auto move_res = move_group_->execute(my_plan);
          if(move_res == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            is_moving = false;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
            return true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed for the waypoint.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan the trajectory");
        }
        is_moving = false;
        return false;
    }

    rclcpp::Subscription<ur_custom_interfaces::msg::URCommand>::SharedPtr subscription_;
    bool is_lookout_position;
    bool is_horizontally_centered;
    bool is_vertically_centered;
    bool is_moving;
    bool is_depth_reached;
    bool is_at_apple_position;
    bool is_apple_grabbed;
    bool is_apple_picked;
    bool is_with_apple_at_lookout_position;
    int prev_x;
    bool was_centered_message_shown;
    float depth;
    std::vector<float> depths;
    rclcpp::Time end_timer;
    rclcpp::Time timer;
    geometry_msgs::msg::Pose* lookout_pos;
    geometry_msgs::msg::Pose* apple_drop_pose;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    geometry_msgs::msg::Pose target_pose;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  // geometry_msgs::msg::Pose lookout_pos;
  // lookout_pos.orientation.w = 0.714969;
  // lookout_pos.orientation.x = -0.698944;
  // lookout_pos.orientation.y = 0.007149;
  // lookout_pos.orientation.z = -0.015674;
  // lookout_pos.position.x = -0.132976;
  // lookout_pos.position.y = 0.133019;
  // lookout_pos.position.z = 0.433306;

    geometry_msgs::msg::Pose lookout_pos;
  lookout_pos.orientation.w = 0.704585;
  lookout_pos.orientation.x = -0.708946;
  lookout_pos.orientation.y = -0.028501;
  lookout_pos.orientation.z = 0.011923;
  lookout_pos.position.x = -0.131775;
  lookout_pos.position.y = -0.127100;
  lookout_pos.position.z = 0.571933;


  geometry_msgs::msg::Pose apple_drop_pos;
  apple_drop_pos.orientation.w = 0.714969;
  apple_drop_pos.orientation.x = -0.698944;
  apple_drop_pos.orientation.y = 0.007149;
  apple_drop_pos.orientation.z = -0.015674;
  apple_drop_pos.position.x = -0.132976;
  apple_drop_pos.position.y = 0.133019;
  apple_drop_pos.position.z = 0.433306;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_robot_node = rclcpp::Node::make_shared("move_robot", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_robot_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  rclcpp::spin(std::make_shared<RobotMasterController>(move_robot_node, &lookout_pos, &apple_drop_pos));
  rclcpp::shutdown();
  return 0;
}