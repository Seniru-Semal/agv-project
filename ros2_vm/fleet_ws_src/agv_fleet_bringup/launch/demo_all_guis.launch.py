#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_legacy_hmi = LaunchConfiguration("start_legacy_hmi")
    start_supervisor = LaunchConfiguration("start_supervisor")
    start_stores = LaunchConfiguration("start_stores")
    start_bench_1 = LaunchConfiguration("start_bench_1")
    start_bench_2 = LaunchConfiguration("start_bench_2")
    start_bench_3 = LaunchConfiguration("start_bench_3")
    bench_1_id = LaunchConfiguration("bench_1_id")
    bench_2_id = LaunchConfiguration("bench_2_id")
    bench_3_id = LaunchConfiguration("bench_3_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_legacy_hmi",
                default_value="false",
                description="Start the older fleet HMI window on the VM.",
            ),
            DeclareLaunchArgument(
                "start_supervisor",
                default_value="true",
                description="Start the supervisor operator GUI.",
            ),
            DeclareLaunchArgument(
                "start_stores",
                default_value="true",
                description="Start the stores operator GUI.",
            ),
            DeclareLaunchArgument(
                "start_bench_1",
                default_value="true",
                description="Start workbench GUI instance 1.",
            ),
            DeclareLaunchArgument(
                "start_bench_2",
                default_value="true",
                description="Start workbench GUI instance 2.",
            ),
            DeclareLaunchArgument(
                "start_bench_3",
                default_value="true",
                description="Start workbench GUI instance 3.",
            ),
            DeclareLaunchArgument(
                "bench_1_id",
                default_value="bench_1",
                description="Workbench ID used by workbench GUI instance 1.",
            ),
            DeclareLaunchArgument(
                "bench_2_id",
                default_value="bench_2",
                description="Workbench ID used by workbench GUI instance 2.",
            ),
            DeclareLaunchArgument(
                "bench_3_id",
                default_value="bench_3",
                description="Workbench ID used by workbench GUI instance 3.",
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
                    "start_legacy_hmi": start_legacy_hmi,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agv_fleet_bringup"),
                            "launch",
                            "operator_guis.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "start_supervisor": start_supervisor,
                    "start_stores": start_stores,
                    "start_bench_1": start_bench_1,
                    "start_bench_2": start_bench_2,
                    "start_bench_3": start_bench_3,
                    "bench_1_id": bench_1_id,
                    "bench_2_id": bench_2_id,
                    "bench_3_id": bench_3_id,
                }.items(),
            ),
        ]
    )
