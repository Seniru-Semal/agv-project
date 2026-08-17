#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/agv1_lidar",
                description="AGV1 RPLIDAR serial port",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value="115200",
                description="RPLIDAR A1 serial baud rate",
            ),
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="agv1_rplidar",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": "agv1_laser",
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
                remappings=[
                    ("/scan", "/agv_1/scan"),
                ],
            ),
        ]
    )
