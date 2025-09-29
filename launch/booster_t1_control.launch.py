#!/usr/bin/env python3

"""
Booster Robot Driver - Standard Position Control Launch File

This launch file starts the Booster T1 robot driver with standard position control
using native ROS2 topic communication. No external SDK dependencies required.

Features:
- Native ROS2 topic communication via /joint_ctrl and /low_state
- Mock hardware mode for safe testing (default)
- Real robot mode with safety features
- Direct communication with SDK (URDF and SDK joint orders are identical)
- RViz visualization

Parameters:
- use_mock_hardware: Enable mock mode for testing (default: true)
- robot_description_file: URDF file to use (default: T1_ros2_control.urdf.xacro)
- launch_rviz: Launch RViz visualization (default: true)
- controllers_config_file: Controller configuration YAML file (default: ros2_controllers.yaml)

Usage:
  # Mock hardware (safe for testing)
  ros2 launch booster_robot_driver booster_t1_control.launch.py

  # Real robot (waits for /low_state before commanding)
  ros2 launch booster_robot_driver booster_t1_control.launch.py use_mock_hardware:=false

  # With custom controller config
  ros2 launch booster_robot_driver booster_t1_control.launch.py controllers_config_file:=/path/to/custom_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware interface for testing (true) or real robot topics (false)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="T1_ros2_control.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_file",
            default_value="ros2_controllers.yaml",
            description="Controller configuration YAML file",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_description_file = LaunchConfiguration("robot_description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_config_file = LaunchConfiguration("controllers_config_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("booster_robot_driver"), "urdf", robot_description_file]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("booster_robot_driver"),
            "config",
            controllers_config_file,
        ]
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"publish_robot_description": True}],
    )

    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Joint Group Position Controller
    joint_group_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Delay start of joint_group_position_controller after joint_state_broadcaster
    delay_joint_group_position_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_group_position_controller_spawner],
            )
        )
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("booster_robot_driver"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_joint_group_position_controller_spawner_after_joint_state_broadcaster_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)