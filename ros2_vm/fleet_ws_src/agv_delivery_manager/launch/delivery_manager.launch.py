#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="agv_delivery_manager",
                executable="delivery_manager_node",
                name="delivery_manager_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
