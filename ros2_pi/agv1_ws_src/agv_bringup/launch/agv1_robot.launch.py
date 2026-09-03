#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")

    imu_i2c_bus = LaunchConfiguration("imu_i2c_bus")
    imu_address = LaunchConfiguration("imu_address")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arduino_port",
                default_value="/dev/agv_arduino",
                description="Arduino serial port",
            ),

            DeclareLaunchArgument(
                "arduino_baud",
                default_value="115200",
                description="Arduino serial baud rate",
            ),

            DeclareLaunchArgument(
                "imu_i2c_bus",
                default_value="1",
                description="BMI160 I2C bus number",
            ),

            DeclareLaunchArgument(
                "imu_address",
                default_value="105",
                description="BMI160 I2C address as decimal. 0x69 = 105.",
            ),

            Node(
                package="agv_arduino_bridge",
                executable="arduino_bridge_node",
                name="arduino_bridge_node",
                output="screen",
                parameters=[
                    {
                        "port": arduino_port,
                        "baud": arduino_baud,
                    }
                ],
            ),

            Node(
                package="agv_safety",
                executable="safety_manager_node",
                name="safety_manager_node",
                output="screen",
                parameters=[
                    {
                        "status_timeout_sec": 1.0,
                        "hard_timeout_sec": 3.0,
                        "auto_stop_on_stale": True,
                        "auto_estop_on_hard_timeout": False,
                        "auto_stop_on_fault": True,
                    }
                ],
            ),

            Node(
                package="agv_imu",
                executable="bmi160_imu_node",
                name="bmi160_imu_node",
                output="screen",
                parameters=[
                    {
                        "i2c_bus": imu_i2c_bus,
                        "i2c_address": imu_address,
                        "frame_id": "imu_link",
                        "publish_rate_hz": 100.0,
                        "calibrate_gyro_on_start": True,
                        "gyro_calibration_samples": 300,
                    }
                ],
            ),

            Node(
                package="agv_feature_classifier",
                executable="feature_classifier_node",
                name="feature_classifier_node",
                output="screen",
                parameters=[
                    {
                        "station_wide_confirm_frames": 12,
                        "station_solid_confirm_frames": 5,
                        "junction_narrow_confirm_frames": 2,
                        "marker_clear_confirm_frames": 2,
                        "classification_timeout_sec": 2.0,
                        "event_cooldown_sec": 0.5,
                        "ticks_per_mm": 1.064,
                        "clear_junction_ignore_distance_mm": 1200.0,
                    }
                ],
            ),

            Node(
                package="agv_turn_manager",
                executable="turn_manager_node",
                name="turn_manager_node",
                output="screen",
                parameters=[
                    {
                        "imu_topic": "/agv_1/imu/data_raw",

                        "turn_fast_pwm": 35,
                        "turn_slow_pwm": 25,
                        "turn_slowdown_angle_deg": 10.0,
                        "turn_tolerance_deg": 2.0,
                        "turn_stop_early_deg": 0.0,
                        "turn_timeout_sec": 25.0,
                        "turn_settle_delay_sec": 0.15,
                        "turn_yaw_sign": 1.0,

                        "auto_reacquire_after_turn": False,
                        "auto_start_after_turn": False,

                        "line_assist_enabled": True,
                        "old_line_ignore_yaw_deg": 60.0,
                        "line_accept_before_target_deg": 12.0,
                        "line_accept_after_target_deg": 35.0,
                        "line_center_tolerance": 220,
                        "line_active_min": 1,
                        "line_active_max": 5,
                        "line_confirm_frames": 2,
                        "line_latch_min_yaw_deg": 60.0,
                        "require_line_cross_count": True,
                    }
                ],
            ),

            Node(
                package="agv_rfid",
                executable="rfid_reader_node",
                name="rfid_reader_node",
                output="screen",
                parameters=[
                    {
                        "robot_ns": "agv_1",
                        "rfid_map_path": "/home/seniru/agv_ws/src/agv_mission_manager/config/rfid_map_fleet.json",
                        "poll_interval_sec": 0.02,
                        "same_uid_debounce_sec": 1.5,
                        "publish_unknown_uid": True,
                    }
                ],
            ),

            Node(
                package="agv_feature_action",
                executable="feature_action_node",
                name="feature_action_node",
                output="screen",
                parameters=[
                    {
                        "graph_file": "/home/seniru/agv_ws/src/agv_mission_manager/config/track_graph_fleet.json",

                        "ticks_per_mm": 1.064,

                        # Not used in normal RFID mode.
                        # Kept only for optical backup mode.
                        "junction_center_offset_mm": 190.0,
                        "station_stop_offset_mm": 200.0,

                        "approach_raw_left_pwm": 30,
                        "approach_raw_right_pwm": 30,

                        "exit_raw_left_pwm": 30,
                        "exit_raw_right_pwm": 30,

                        "clear_junction_ignore_distance_mm": 1200.0,

                        "event_cooldown_sec": 1.0,
                        "move_timeout_sec": 20.0,
                        "require_mission_active": True,

                        "use_rfid_node_events": True,
                        "use_optical_feature_events": False,
                        "rfid_arrival_debounce_sec": 1.0,

                        "left_turn_angle_deg": 100.0,
                        "right_turn_angle_deg": -100.0,
                        "uturn_angle_deg": 180.0,
                    }
                ],
            ),

            Node(
                package="agv_mission_manager",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "graph_file": "/home/seniru/agv_ws/src/agv_mission_manager/config/track_graph_fleet.json",
                        "ticks_per_mm": 1.064,
                        "reset_feature_action_on_start": True,
                        "start_line_follow_on_start": True,
                        "auto_stop_on_cancel": True,
                        "geometry_straight_dot_threshold": 0.70,
                        "geometry_uturn_dot_threshold": -0.70,
                    }
                ],
            ),
        ]
    )
