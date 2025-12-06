"""
Launch file for fr_arm with REAL hardware (MyActuator RMD motors via CAN).

Usage:
    ros2 launch fr_arm_moveit_config hardware.launch.py
    ros2 launch fr_arm_moveit_config hardware.launch.py can_interface:=can1

Prerequisites:
    1. CAN interface must be configured:
       sudo ip link set can0 type can bitrate 1000000
       sudo ip link set can0 up
    
    2. Motors must be powered on and connected via CAN bus
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name (e.g., can0, can1)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level (debug, info, warn, error)",
        )
    )

    # Get package paths
    fr_arm_moveit_config_path = get_package_share_directory("fr_arm_moveit_config")
    
    # Launch configurations
    can_interface = LaunchConfiguration("can_interface")
    rviz_config = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    # Generate robot description from xacro (hardware version)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("fr_arm_moveit_config"),
                "config",
                "fr_arm_hardware.urdf.xacro",
            ]),
            " can_interface:=", can_interface,
        ],
        on_stderr="ignore",
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Load SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("fr_arm_moveit_config"),
                "config",
                "fr_arm.srdf",
            ]),
        ],
        on_stderr="ignore",
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # Load kinematics configuration as dictionary
    kinematics_yaml = load_yaml("fr_arm_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # Load controllers configuration
    controllers_yaml = os.path.join(fr_arm_moveit_config_path, "config", "ros2_controllers.yaml")

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Arm controller spawner (after joint_state_broadcaster)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Gripper controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Delay arm controller after joint_state_broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Delay gripper controller after arm controller
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Delay move_group after controllers are loaded
    delay_move_group = TimerAction(
        period=3.0,
        actions=[move_group_node],
    )

    # RViz
    rviz_config_file = os.path.join(fr_arm_moveit_config_path, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        condition=IfCondition(rviz_config),
    )

    # Delay RViz
    delay_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
            delay_gripper_controller,
            delay_move_group,
            delay_rviz,
        ]
    )
