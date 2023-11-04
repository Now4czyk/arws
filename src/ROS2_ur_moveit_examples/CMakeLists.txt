cmake_minimum_required(VERSION 3.8)
project(ros2_ur_moveit_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# moveit_package()

include_directories(
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/urdf
  /opt/ros/humble/include/moveit_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/include/trajectory_msgs
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/visualization_msgs
  /opt/ros/humble/include/object_recognition_msgs
  /opt/ros/humble/include/shape_msgs
  /opt/ros/humble/include/octomap_msgs
  /opt/ros/humble/include/unique_identifier_msgs
  /opt/ros/humble/include/action_msgs
  /opt/ros/humble/include/rclcpp_action
  /opt/ros/humble/include/rcl_action
  /opt/ros/humble/include/tf2_msgs
  /opt/ros/humble/include/pluginlib
  /opt/ros/humble/include/class_loader
  /opt/ros/humble/include/graph_msgs
  ${EIGEN3_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC include)
ament_target_dependencies(hello_moveit moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs)

add_executable(kinematics src/kinematics.cpp)
target_include_directories(kinematics PUBLIC include)
ament_target_dependencies(kinematics moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs)

add_executable(check_collisions src/check_collisions.cpp)
target_include_directories(check_collisions PUBLIC include)
ament_target_dependencies(check_collisions moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs)

add_executable(planning_scene src/planning_scene.cpp)
target_include_directories(planning_scene PUBLIC include)
ament_target_dependencies(planning_scene moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs)

add_executable(planning_scene_service src/planning_scene_service.cpp)
target_include_directories(planning_scene_service PUBLIC include)
ament_target_dependencies(planning_scene_service moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS hello_moveit kinematics check_collisions planning_scene planning_scene_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
