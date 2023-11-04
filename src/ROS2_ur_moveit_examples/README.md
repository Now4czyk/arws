# ros2_ur_moveit_examples
ROS2 humble MoveIt examples for Universal Robots on Ubuntu 22.04.

Examples from:

https://moveit.picknik.ai/humble/index.html

and

https://github.com/ros-planning/moveit2_tutorials

Implemented for Universal Robots.

Clone required repositiories:

$ cd ~/ros2_ws/src

$ git clone https://github.com/dominikbelter/ROS2_ur_moveit_examples

$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

$ git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

$ cd ~/ros2_ws

$ colcon build --symlink-install

$ . install/setup.bash

To run examples (change the robot names if needed):

$ ros2 run ur_robot_driver start_ursim.sh -m ur3e

Start the robot in the browser and then start the controller:

$ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller

Start MoveIt:

$ ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true use_fake_hardware:=true

Add additional transformations:

$ ros2 run tf2_ros static_transform_publisher 0 0 0.1 0.0 0.0 0.0 tool0 gripper

$ ros2 run tf2_ros static_transform_publisher 0.3 0.3 0.1 0.0 3.14 0.0 base_link object

And run examples:

$ ros2 launch ros2_ur_moveit_examples hello_moveit.launch.py

$ ros2 launch ros2_ur_moveit_examples kinematics.launch.py

$ ros2 launch ros2_ur_moveit_examples check_collisions.launch.py

$ ros2 launch ros2_ur_moveit_examples planning_scene.launch.py

$ ros2 launch ros2_ur_moveit_examples planning_scene_service.launch.py

Check the output in the terminal and in the RViz2.
