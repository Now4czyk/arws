
// example no 1
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    // move_group_interface.setPlannerId("TRRTkConfigDefault");
    move_group_interface.setGoalTolerance(0.01);
    move_group_interface.setPoseReferenceFrame("wrist_3_link");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });
    auto position = move_group_interface.getCurrentPose("wrist_3_link").pose;
    auto position2 = move_group_interface.getCurrentPose("wrist_3_link");
    RCLCPP_INFO(logger, "Current position: x=%s", position2.header.frame_id.c_str());
    auto frame = move_group_interface.getPlanningFrame();
    RCLCPP_INFO(logger, "Planning Frame=%s", frame);
    // RCLCPP_INFO(logger, "Current position: x=%f, y=%f, z=%f", position.x, position.y, position.z);

    // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
                                     move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.0;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   planning_scene_interface.applyCollisionObject(collision_object);

    // Set a target Pose
    // auto const target_pose = [
    //         msgOrientationW, 
    //         msgOrientationX, 
    //         msgOrientationY, 
    //         msgOrientationZ,
    //         msgPositionX,
    //         msgPositionY,
    //         msgPositionZ
    //     ]
    // {
    //     geometry_msgs::msg::Pose msg;
    //     msg.orientation.w = msgOrientationW;
    //     msg.orientation.x = msgOrientationX;
    //     msg.orientation.y = msgOrientationY;
    //     msg.orientation.z = msgOrientationZ;
    //     msg.position.x = msgPositionX;
    //     msg.position.y = msgPositionY;
    //     msg.position.z = msgPositionZ;
    //     return msg;
    // }();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose point;
    point.position.x = 0.01;
    point.position.y = 0.0;
    point.position.z = 0.0;

    point.orientation.w = 1.0;
    point.orientation.x = 0.0;
    point.orientation.y = 0.0;
    point.orientation.z = 0.0;
    waypoints.push_back(point);

    // geometry_msgs::msg::Pose target_pose3 = target_pose;

    // target_pose3.position.z -= 0.2;
    // waypoints.push_back(target_pose3);  // down

    // target_pose3.position.y -= 0.2;
    // waypoints.push_back(target_pose3);  // right

    // target_pose3.position.z += 0.2;
    // target_pose3.position.y += 0.2;
    // target_pose3.position.x -= 0.2;
    // waypoints.push_back(target_pose3);  // up and left


    // Create a plan to that target pose
    for (const auto& waypoint : waypoints) {
        move_group_interface.setPoseTarget(waypoint, "wrist_3_link");
        // move_group_interface.setJointValueTarget(std::vector<double>{1.6306328773498535, -0.08663781106982427, -0.8417906761169434, 3.6392885881611328, -1.237258259450094, -5.0265167395221155});
        auto joints = move_group_interface.getCurrentJointValues();
        for (auto join : joints){
            RCLCPP_INFO(logger, "Current position: joint=%f", join);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));

        if (success) {
            move_group_interface.execute(plan);
            RCLCPP_INFO(logger, "Execution successful for the waypoint.");
        } else {
            RCLCPP_ERROR(logger, "Path planning failed for the waypoint.");
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
