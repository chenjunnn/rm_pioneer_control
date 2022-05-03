# Copyright (c) 2022 ChenJun
# Licensed under the MIT License.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # robot_description
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_pioneer_description'), 'urdf', 'gimbal.urdf.xacro')])

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
        parameters=[{"robot_description": robot_description}, robot_controllers],
        remappings=[("/rm_gimbal_controller/target", "/processor/target"), ],
        output="screen",
        emulate_tty=True,
    )

    gimbal_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_gimbal_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        gimbal_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
