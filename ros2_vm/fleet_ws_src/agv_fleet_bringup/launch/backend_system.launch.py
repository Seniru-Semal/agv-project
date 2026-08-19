#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_fleet = LaunchConfiguration("start_fleet")
    start_delivery = LaunchConfiguration("start_delivery")
    start_legacy_hmi = LaunchConfiguration("start_legacy_hmi")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_fleet",
                default_value="true",
                description="Start the VM fleet manager core.",
            ),
            DeclareLaunchArgument(
                "start_delivery",
                default_value="true",
                description="Start the delivery workflow manager.",
            ),
            DeclareLaunchArgument(
                "start_legacy_hmi",
                default_value="false",
                description="Start the older fleet HMI window on the VM.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agv_fleet_bringup"),
                            "launch",
                            "fleet_core.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "start_legacy_hmi": start_legacy_hmi,
                }.items(),
                condition=IfCondition(start_fleet),
            ),
            Node(
                package="agv_delivery_manager",
                executable="delivery_manager_node",
                name="delivery_manager_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_delivery),
            ),
        ]
    )
