#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_guard = Node(
        package="agv_obstacle_guard",
        executable="obstacle_guard_node",
        name="agv1_obstacle_guard",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "robot_ns": "agv_1",
                "scan_topic": "/agv_1/scan",

                "front_center_deg": 0.0,

                "stop_half_angle_deg": 90.0,
                "warn_half_angle_deg": 90.0,

                "stop_distance_mm": 200.0,
                "warn_distance_mm": 300.0,
                "clear_distance_mm": 250.0,

                "scan_timeout_sec": 0.5,
                "stop_repeat_sec": 0.2,
                "publish_stop_repeatedly": True,
            }
        ],
    )

    recovery_watchdog = Node(
        package="agv_obstacle_guard",
        executable="lidar_recovery_watchdog_node",
        name="agv1_lidar_recovery_watchdog",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "scan_topic": "/agv_1/scan",
                "lidar_device": "/dev/agv1_lidar",
                "lidar_node_name": "agv1_rplidar",
                "driver_executable": "rplidar_node",

                "scan_timeout_sec": 1.5,
                "startup_grace_sec": 5.0,
                "device_settle_sec": 1.0,
                "restart_cooldown_sec": 5.0,
            }
        ],
    )

    safety_hold = Node(
        package="agv_obstacle_guard",
        executable="safety_hold_node",
        name="agv1_safety_hold",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "robot_ns": "agv_1",
                "clear_stable_sec": 2.0,
                "stop_repeat_sec": 0.25,
                "auto_clear_when_mission_inactive": True,
            }
        ],
    )

    return LaunchDescription(
        [
            obstacle_guard,
            recovery_watchdog,
            safety_hold,
        ]
    )
