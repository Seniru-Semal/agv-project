#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_legacy_hmi = LaunchConfiguration("start_legacy_hmi")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_legacy_hmi",
                default_value="false",
                description="Start the older fleet HMI window.",
            ),
            Node(
                package="agv_fleet_manager",
                executable="planned_route_fleet_manager_node",
                name="fleet_manager_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "auto_resume_enabled": True,
                        "auto_resume_check_period_sec": 0.25,
                        "auto_resume_retry_sec": 1.0,
                        "exact_path_retry_sec": 1.0,
                        "controlled_retreat_delay_sec": 2.0,
                        "allow_idle_egress_borrowing": True,
                    }
                ],
            ),
            Node(
                package="agv_fleet_manager",
                executable="fleet_hmi_node",
                name="fleet_hmi_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_legacy_hmi),
            ),
        ]
    )
