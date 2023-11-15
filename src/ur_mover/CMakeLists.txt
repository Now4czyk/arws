cmake_minimum_required(VERSION 3.8)
project(ur_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur_custom_interfaces REQUIRED)                      # CHANGE

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
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

add_executable(master_node src/master_node.cpp)
ament_target_dependencies(master_node rclcpp ur_custom_interfaces moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools)  # CHANGE

install(TARGETS
  master_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
