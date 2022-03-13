# Copyright (c) 2022 ChenJun
# Licensed under the MIT License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Get URDF via xacro
    robot_description_content = Command(
        [
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare("rm_pioneer_description"), "urdf", "guard.urdf.xacro"]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rm_pioneer_control_bringup"),
            "config",
            "guard_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[("/rm_gimbal_controller/commands", "/joy"),
                    ("/rm_gimbal_controller/target", "/processor/target"),],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gimbal_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_gimbal_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        gimbal_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)