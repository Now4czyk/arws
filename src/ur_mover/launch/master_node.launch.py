import launch
import os
import yaml

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.event_handlers import OnProcessExit

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
    
def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic

def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "ur_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_config_package = "ur_moveit_config"
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # argumenty nie mogą być niczym innym z tego co widzę niż stringami
    # Argumenty to po kolei:
    # 1. Randomowy string pokazowo jak przekazać stringa na przyszłość
    # 2. msg.orientation.w;
    # 3. msg.orientation.x;
    # 4. msg.orientation.y;
    # 5. msg.orientation.z;
    # 6. msg.position.x;
    # 7. msg.position.y;
    # 8. msg.position.z;
    arguments1 = [
        "argument1 to arg przykladowy do stringa",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.2",
        "-0.2",
        "0.1",
    ]
    arguments2 = [
        "argument1 to arg przykladowy do stringa",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "-0.2",
        "0.2",
        "0.1",
    ]

    demo_node1 = Node(
        package="ur_mover",
        executable="master_node",
        name="master_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        arguments=arguments1,
    )

    # demo_node2 = Node(
    #     package="ros2_ur_moveit_examples",
    #     executable="hello_moveit",
    #     name="hello_moveit",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #     ],
    #     arguments=arguments2,
    # )
    # # Define an event handler to launch demo_node2 after demo_node1 exits
    # demo_node1_exit_handler = launch.actions.RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=demo_node1,
    #         on_exit=LaunchDescription([
    #                 demo_node2
    #             ])
    #     )
    # )

    # return launch.LaunchDescription([demo_node1, demo_node1_exit_handler])

    return launch.LaunchDescription([demo_node1])