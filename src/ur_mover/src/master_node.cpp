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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(std::shared_ptr<rclcpp::Node> move_group_node)
    : Node("master_node")
    {
      RCLCPP_INFO(this->get_logger(), "Node started. Awaiting commands...");
      subscription_ = this->create_subscription<ur_custom_interfaces::msg::URCommand>(
      "custom_camera", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

      using moveit::planning_interface::MoveGroupInterface;
      move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_node, "your_move_group_name");
      this->move_group_->stop();
    }

  private:
    void topic_callback(const ur_custom_interfaces::msg::URCommand::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.get()->command.c_str());
    }
    rclcpp::Subscription<ur_custom_interfaces::msg::URCommand>::SharedPtr subscription_;
    moveit::planning_interface::MoveGroupInterface* move_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}