# Implementation of Fruit Harvesting System Prototype Using Robotic Arm with Vision Feedback

## Description

The project includes comprehensive software capable of identifying and harvesting apples using a UR3e robot, OAK-D Pro camera, and RG2 gripper. After assembling all the components and starting the required nodes, the robot will position itself ready to scan the tree. If an apple is detected in the camera's field of view, the robot will move to the harvesting position and collect the fruit. After harvesting, the robot will return to the scanning position, waiting for the next apple.

## Nodes and Topics Structure

### Node camera_control_node

This node is responsible for processing the camera image. It processes the captured data using the YOLO v8 neural network. After detecting an apple, it sends information to the robot about the horizontal and vertical directions it should move. This is done through the custom_camera topic. The node is also responsible for measuring and transmitting the distance to the apple, essential for moving the robot the correct distance.

### Node gripper_control_node

This node is responsible for opening and closing the gripper. It listens for commands on the `custom_gripper` topic. Upon receiving a command to open or close, it sends the corresponding function along with its invocation to the robot controller. After 4 seconds, it activates the `/dashboard_client/play` service, responsible for regaining control over the robot in the `External Control` program block. This allows continued control of UR3e through ROS2.

### Node master_node

This node is responsible for moving the robotic arm. It sets the robot in the position to search for apples and moves it in the correct direction along the horizontal and vertical axes. It subscribes to the `custom_camera` topic and performs the appropriate robot movements based on the data read from it. When it needs to grip the apple, it publishes the relevant command to the `custom_gripper` topic and waits for 5 seconds before moving the apple to the target point.

### Topic custom_camera

This topic is responsible for transmitting data about the apple's position relative to the camera. The accepted data type is: `{data: {x: string, y: string, depth: string}}`.

### Topic custom_gripper

This topic controls the gripper. The accepted data type is: `{data: string}`.

## Required Hardware

- Universal Robot 3e robotic arm,
- OAK-D Pro camera,
- Camera mount,
- RG2 gripper,
- Illuminator (recommended but not required)

## Hardware Configuration

### UR3e Robot

Connect the robot to the router. Connecting our computers to the same router allows communication with the robot. In the robot configuration (on Teach Pendant), enable "Enable Remote Control," set the "Host IP" field to your computer's address, and add the "External Control" block to the robot program.

### OAK-D Pro Camera

Mount the camera using our designed mount. Then, connect it using two cables - one for power and the other for data transfer. Connect the data transmission cable to the computer where the camera handling node will run.

### RG2 Gripper

Mount the gripper at the end of the robot arm and connect it to the port near the end effector. It is essential to attach the camera to the mount first.

![Robot end effector after installing all equipment](./vision_setup.jpg)

## Installation

To run the project, follow these steps:

1. Install the required software: ROS2 humble, python3, depthai, ultralytics,
2. Build the project using the commands `colcon build` and `source install/setup.bash`,
3. In the first terminal window enter the command `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ROBOT_IP_ADDRESS> initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=false headless_mode:=false`,
4. In another terminal window, launch moveit2 with the command: `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e`,
5. In the next terminal window, run the camera control node: `ros2 run camera_control camera_control_node`,
6. Next, run the gripper control node: `ros2 run gripper_control gripper_control_node`,
7. The final step is to launch the node responsible for moving the robot: `ros2 launch ur_mover master_node.launch.py`

## Authors
Patryk Marczak,
Kacper Nowaczyk
