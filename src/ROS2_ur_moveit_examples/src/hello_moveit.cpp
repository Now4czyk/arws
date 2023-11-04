
// example no 1
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");
    
    if (argc < 3)
    {
        RCLCPP_ERROR(logger, "Insufficient command-line arguments. Usage: hello_moveit arg1 arg2");
        return 1;
    }

    float msgOrientationW = std::stof(argv[2]);
    float msgOrientationX = std::stof(argv[3]);
    float msgOrientationY = std::stof(argv[4]);
    float msgOrientationZ = std::stof(argv[5]);
    float msgPositionX = std::stof(argv[6]);
    float msgPositionY = std::stof(argv[7]);
    float msgPositionZ = std::stof(argv[8]);


    std::string arg1 = argv[1];

    std::cout << "Argument 1: " << arg1 << std::endl;
    std::cout << "Argument 2: " << msgOrientationW << std::endl;
    std::cout << "Argument 3: " << msgOrientationX << std::endl; 
    std::cout << "Argument 4: " << msgOrientationY << std::endl;
    std::cout << "Argument 5: " << msgOrientationZ << std::endl;
    std::cout << "Argument 6: " << msgPositionX << std::endl;
    std::cout << "Argument 7: " << msgPositionY << std::endl;
    std::cout << "Argument 8: " << msgPositionZ << std::endl;

    RCLCPP_WARN(logger, "xxxx");
    RCLCPP_WARN(logger, std::to_string(argc).c_str());
    RCLCPP_WARN(logger, *argv);

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    // Set a target Pose
    auto const target_pose = [
            msgOrientationW, 
            msgOrientationX, 
            msgOrientationY, 
            msgOrientationZ,
            msgPositionX,
            msgPositionY,
            msgPositionZ
        ]
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = msgOrientationW;
        msg.orientation.x = msgOrientationX;
        msg.orientation.y = msgOrientationY;
        msg.orientation.z = msgOrientationZ;
        msg.position.x = msgPositionX;
        msg.position.y = msgPositionY;
        msg.position.z = msgPositionZ;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
