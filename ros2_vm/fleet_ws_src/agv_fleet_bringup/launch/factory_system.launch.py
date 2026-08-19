#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            DeclareLaunchArgument(
                "start_operator_guis",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "start_supervisor",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "start_stores",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "start_bench_1",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "start_bench_2",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "start_bench_3",
                default_value="false",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "bench_1_id",
                default_value="bench_1",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "bench_2_id",
                default_value="bench_2",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            DeclareLaunchArgument(
                "bench_3_id",
                default_value="bench_3",
                description="Compatibility only. Ignored by this backend-only launch.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agv_fleet_bringup"),
                            "launch",
                            "backend_system.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "start_fleet": start_fleet,
                    "start_delivery": start_delivery,
                    "start_legacy_hmi": start_legacy_hmi,
                }.items(),
            ),
        ]
    )
