#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="agv_operator_ui",
                executable="agv_supervisor_gui",
                name="agv_supervisor_gui",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_stores_gui",
                name="agv_stores_gui",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_workbench_gui",
                name="agv_workbench_1_gui",
                output="screen",
                emulate_tty=True,
                parameters=[{"workbench_id": "bench_1"}],
            ),
        ]
    )
