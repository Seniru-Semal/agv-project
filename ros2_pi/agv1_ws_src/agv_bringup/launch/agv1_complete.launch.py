#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def bringup_launch(filename):
    bringup_share = get_package_share_directory("agv_bringup")
    return PythonLaunchDescriptionSource(
        os.path.join(bringup_share, "launch", filename)
    )


def generate_launch_description():
    arduino_port = LaunchConfiguration("arduino_port")
    lidar_port = LaunchConfiguration("lidar_port")

    lidar_launch = IncludeLaunchDescription(
        bringup_launch("agv1_lidar.launch.py"),
        launch_arguments={
            "serial_port": lidar_port,
            "serial_baudrate": "115200",
        }.items(),
    )

    safety_launch = IncludeLaunchDescription(
        bringup_launch("agv1_safety.launch.py")
    )

    robot_launch = IncludeLaunchDescription(
        bringup_launch("agv1_robot.launch.py"),
        launch_arguments={
            "arduino_port": arduino_port,
        }.items(),
    )

    fleet_gate_launch = IncludeLaunchDescription(
        bringup_launch("agv1_fleet_gate.launch.py")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arduino_port",
                default_value="/dev/agv_arduino",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/agv1_lidar",
            ),

            lidar_launch,

            TimerAction(
                period=1.0,
                actions=[safety_launch],
            ),

            TimerAction(
                period=2.0,
                actions=[robot_launch],
            ),

            TimerAction(
                period=3.0,
                actions=[fleet_gate_launch],
            ),
        ]
    )
