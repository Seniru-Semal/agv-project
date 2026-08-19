#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
                default_value="false",
                description="Start workbench GUI instance 2.",
            ),
            DeclareLaunchArgument(
                "start_bench_3",
                default_value="false",
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
            Node(
                package="agv_operator_ui",
                executable="agv_supervisor_gui",
                name="agv_supervisor_gui",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_supervisor),
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_stores_gui",
                name="agv_stores_gui",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_stores),
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_workbench_gui",
                name="agv_workbench_1_gui",
                output="screen",
                emulate_tty=True,
                parameters=[{"workbench_id": bench_1_id}],
                condition=IfCondition(start_bench_1),
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_workbench_gui",
                name="agv_workbench_2_gui",
                output="screen",
                emulate_tty=True,
                parameters=[{"workbench_id": bench_2_id}],
                condition=IfCondition(start_bench_2),
            ),
            Node(
                package="agv_operator_ui",
                executable="agv_workbench_gui",
                name="agv_workbench_3_gui",
                output="screen",
                emulate_tty=True,
                parameters=[{"workbench_id": bench_3_id}],
                condition=IfCondition(start_bench_3),
            ),
        ]
    )
