#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workbench_id = LaunchConfiguration("workbench_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workbench_id",
                default_value="bench_1",
                description="Workbench node/name for this GUI instance",
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_workbench_gui",
                name="agv_workbench_gui",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "workbench_id": workbench_id,
                    }
                ],
            ),
        ]
    )
