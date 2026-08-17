#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="agv_fleet_gate",
                executable="fleet_gate_node",
                name="agv1_fleet_gate",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "robot_ns": "agv_1",
                    }
                ],
            ),
        ]
    )
