#!/usr/bin/env python3

import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool, Float32


class TurnManagerNode(Node):
    def __init__(self):
        super().__init__("turn_manager_node")

        # ==================================================
        # Parameters
        # ==================================================

        self.declare_parameter("imu_topic", "/agv_1/imu/data_raw")

        self.declare_parameter("turn_fast_pwm", 80)
        self.declare_parameter("turn_slow_pwm", 60)

        self.declare_parameter("turn_slowdown_angle_deg", 10.0)
        self.declare_parameter("turn_tolerance_deg", 2.0)
        self.declare_parameter("turn_stop_early_deg", 0.0)
        self.declare_parameter("turn_timeout_sec", 25.0)
        self.declare_parameter("turn_settle_delay_sec", 0.15)

        # If progress goes negative during a positive left turn, set this to -1.0.
        self.declare_parameter("turn_yaw_sign", 1.0)

        # Keep false because feature_action_node handles post-turn exit.
        self.declare_parameter("auto_reacquire_after_turn", False)
        self.declare_parameter("auto_start_after_turn", False)

        # ==================================================
        # Line-assisted turn latch
        # ==================================================

        self.declare_parameter("line_assist_enabled", True)

        # Ignore the old/current line during early rotation.
        self.declare_parameter("old_line_ignore_yaw_deg", 35.0)

        # For 90 degree turns, accept centered outgoing line near target.
        self.declare_parameter("line_accept_before_target_deg", 35.0)
        self.declare_parameter("line_accept_after_target_deg", 25.0)

        self.declare_parameter("line_center_tolerance", 190)
        self.declare_parameter("line_active_min", 1)
        self.declare_parameter("line_active_max", 5)
        self.declare_parameter("line_confirm_frames", 3)

        self.declare_parameter("line_latch_min_yaw_deg", 45.0)
        self.declare_parameter("require_line_cross_count", True)

        # ==================================================
        # Loaded parameters
        # ==================================================

        self.imu_topic = str(self.get_parameter("imu_topic").value)

        self.turn_fast_pwm = int(self.get_parameter("turn_fast_pwm").value)
        self.turn_slow_pwm = int(self.get_parameter("turn_slow_pwm").value)

        self.turn_slowdown_angle_deg = float(
            self.get_parameter("turn_slowdown_angle_deg").value
        )
        self.turn_tolerance_deg = float(
            self.get_parameter("turn_tolerance_deg").value
        )
        self.turn_stop_early_deg = float(
            self.get_parameter("turn_stop_early_deg").value
        )
        self.turn_timeout_sec = float(
            self.get_parameter("turn_timeout_sec").value
        )
        self.turn_settle_delay_sec = float(
            self.get_parameter("turn_settle_delay_sec").value
        )
        self.turn_yaw_sign = float(
            self.get_parameter("turn_yaw_sign").value
        )

        self.auto_reacquire_after_turn = bool(
            self.get_parameter("auto_reacquire_after_turn").value
        )
        self.auto_start_after_turn = bool(
            self.get_parameter("auto_start_after_turn").value
        )

        self.line_assist_enabled = bool(
            self.get_parameter("line_assist_enabled").value
        )
        self.old_line_ignore_yaw_deg = float(
            self.get_parameter("old_line_ignore_yaw_deg").value
        )
        self.line_accept_before_target_deg = float(
            self.get_parameter("line_accept_before_target_deg").value
        )
        self.line_accept_after_target_deg = float(
            self.get_parameter("line_accept_after_target_deg").value
        )
        self.line_center_tolerance = int(
            self.get_parameter("line_center_tolerance").value
        )
        self.line_active_min = int(
            self.get_parameter("line_active_min").value
        )
        self.line_active_max = int(
            self.get_parameter("line_active_max").value
        )
        self.line_confirm_frames = int(
            self.get_parameter("line_confirm_frames").value
        )
        self.line_latch_min_yaw_deg = float(
            self.get_parameter("line_latch_min_yaw_deg").value
        )
        self.require_line_cross_count = bool(
            self.get_parameter("require_line_cross_count").value
        )

        # ==================================================
        # Internal turn state
        # ==================================================

        self.state = "IDLE"

        self.imu_received = False
        self.latest_gyro_z_rad_s = 0.0
        self.last_gyro_time: Optional[float] = None

        self.accumulated_turn_deg = 0.0

        self.target_angle_deg = 0.0
        self.turn_direction = 0

        self.turn_start_time = 0.0
        self.settle_start_time = 0.0

        self.last_pivot_command = "NONE"
        self.last_pwm = 0

        # ==================================================
        # Latest Arduino line state
        # ==================================================

        self.latest_valid = False
        self.latest_active = 0
        self.latest_pos = 0
        self.latest_wide = False
        self.latest_solid = False
        self.latest_marker = False

        self.latest_left_ticks = 0
        self.latest_right_ticks = 0

        # Line latch state
        self.line_confirm_count = 0
        self.line_cross_count = 0
        self.previous_line_seen_after_ignore = False

        self.line_latched = False
        self.line_latch_reason = "NONE"

        # ==================================================
        # ROS I/O
        # ==================================================

        self.create_subscription(
            Float32,
            "/agv_1/cmd/turn_angle_deg",
            self.turn_command_callback,
            10,
        )

        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            "/agv_1/status_raw",
            self.status_callback,
            30,
        )

        self.raw_pub = self.create_publisher(
            String,
            "/agv_1/cmd/raw",
            10,
        )

        self.stop_pub = self.create_publisher(
            Bool,
            "/agv_1/cmd/stop",
            10,
        )

        self.start_pub = self.create_publisher(
            Bool,
            "/agv_1/cmd/start",
            10,
        )

        self.turn_done_pub = self.create_publisher(
            Bool,
            "/agv_1/turn/done",
            10,
        )

        self.turn_failed_pub = self.create_publisher(
            Bool,
            "/agv_1/turn/failed",
            10,
        )

        self.turn_state_pub = self.create_publisher(
            String,
            "/agv_1/turn/state",
            10,
        )

        self.turn_event_pub = self.create_publisher(
            String,
            "/agv_1/turn/event",
            10,
        )

        self.turn_progress_pub = self.create_publisher(
            Float32,
            "/agv_1/turn/progress_deg",
            10,
        )

        self.turn_error_pub = self.create_publisher(
            Float32,
            "/agv_1/turn/error_deg",
            10,
        )

        self.line_cross_count_pub = self.create_publisher(
            String,
            "/agv_1/turn/line_cross_count",
            10,
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("AGV turn manager started")
        self.get_logger().info(f"imu_topic={self.imu_topic}")
        self.get_logger().info("Using gyro Z integration from Imu.angular_velocity.z")
        self.get_logger().info(f"turn_fast_pwm={self.turn_fast_pwm}")
        self.get_logger().info(f"turn_slow_pwm={self.turn_slow_pwm}")
        self.get_logger().info(f"turn_yaw_sign={self.turn_yaw_sign}")
        self.get_logger().info(f"line_assist_enabled={self.line_assist_enabled}")
        self.get_logger().info(
            f"line window: target-{self.line_accept_before_target_deg} "
            f"to target+{self.line_accept_after_target_deg}"
        )

    # ==================================================
    # Input callbacks
    # ==================================================

    def turn_command_callback(self, msg: Float32):
        angle_deg = float(msg.data)

        if abs(angle_deg) < 1.0:
            self.get_logger().warn("Ignoring tiny turn command")
            return

        if not self.imu_received:
            self.get_logger().warn("Cannot start turn: no IMU gyro data yet")
            self.publish_turn_failed(True)
            self.publish_event("TURN_FAILED_NO_IMU_GYRO")
            return

        if self.state not in ["IDLE", "DONE", "FAILED", "TIMEOUT"]:
            self.get_logger().warn(
                f"Ignoring turn command {angle_deg:.1f}; "
                f"turn manager state is {self.state}"
            )
            return

        self.start_turn(angle_deg)

    def imu_callback(self, msg: Imu):
        now = time.time()

        self.imu_received = True
        self.latest_gyro_z_rad_s = float(msg.angular_velocity.z)

        if self.state == "TURNING":
            if self.last_gyro_time is None:
                self.last_gyro_time = now
                return

            dt = now - self.last_gyro_time
            self.last_gyro_time = now

            if dt <= 0.0 or dt > 0.2:
                return

            delta_deg = math.degrees(self.latest_gyro_z_rad_s * dt)
            self.accumulated_turn_deg += delta_deg * self.turn_yaw_sign

        elif self.state == "SETTLING":
            self.last_gyro_time = now

        else:
            self.last_gyro_time = now

    def status_callback(self, msg: String):
        data = self.parse_status_line(msg.data)

        if data is None:
            return

        self.latest_valid = self.safe_int(data.get("VALID", "0")) == 1
        self.latest_active = self.safe_int(data.get("ACTIVE", "0"))
        self.latest_pos = self.safe_int(data.get("POS", "0"))

        self.latest_wide = self.safe_int(data.get("WIDE", "0")) == 1
        self.latest_solid = self.safe_int(data.get("SOLID", "0")) == 1
        self.latest_marker = self.safe_int(data.get("MARKER", "0")) == 1

        self.latest_left_ticks = self.safe_int(data.get("L", "0"))
        self.latest_right_ticks = self.safe_int(data.get("R", "0"))

    def parse_status_line(self, line: str) -> Optional[Dict[str, str]]:
        line = line.strip()

        if not line.startswith("A:"):
            return None

        payload = line[2:]
        fields = payload.split(";")

        data: Dict[str, str] = {}

        for field in fields:
            if "=" not in field:
                continue

            key, value = field.split("=", 1)
            key = key.strip()
            value = value.strip()

            if key:
                data[key] = value

        return data

    # ==================================================
    # Turn state machine
    # ==================================================

    def start_turn(self, angle_deg: float):
        self.target_angle_deg = angle_deg
        self.turn_direction = 1 if angle_deg > 0.0 else -1

        self.accumulated_turn_deg = 0.0
        self.last_gyro_time = None

        self.turn_start_time = time.time()
        self.settle_start_time = 0.0

        self.line_confirm_count = 0
        self.line_cross_count = 0
        self.previous_line_seen_after_ignore = False
        self.line_latched = False
        self.line_latch_reason = "NONE"

        self.state = "TURNING"

        self.publish_turn_done(False)
        self.publish_turn_failed(False)

        self.publish_event(f"TURN_STARTED_{self.direction_text()}")

        self.get_logger().info(
            f"Turn started: target={self.target_angle_deg:.1f} deg"
        )

        self.command_pivot(self.turn_fast_pwm)

    def timer_callback(self):
        self.publish_state()

        if self.state == "TURNING":
            self.update_turning()
            return

        if self.state == "SETTLING":
            self.update_settling()
            return

    def update_turning(self):
        elapsed = time.time() - self.turn_start_time

        if elapsed > self.turn_timeout_sec:
            self.stop_turn_failed("TURN_TIMEOUT")
            return

        progress_deg = self.get_turn_progress_deg()
        error_deg = self.target_angle_deg - progress_deg

        self.publish_progress(progress_deg)
        self.publish_error(error_deg)

        self.update_line_cross_counter(progress_deg)

        if self.should_stop_for_line_assist(progress_deg):
            self.line_latched = True
            self.line_latch_reason = "LINE_ASSIST_LATCH"
            self.stop_turn_success("TURN_DONE_LINE_ASSIST")
            return

        if self.should_stop_for_imu(error_deg):
            self.line_latch_reason = "IMU_TARGET_REACHED"
            self.stop_turn_success("TURN_DONE_IMU")
            return

        remaining_abs = abs(error_deg)

        if remaining_abs <= self.turn_slowdown_angle_deg:
            self.command_pivot(self.turn_slow_pwm)
        else:
            self.command_pivot(self.turn_fast_pwm)

    def update_settling(self):
        elapsed = time.time() - self.settle_start_time

        if elapsed < self.turn_settle_delay_sec:
            return

        self.state = "DONE"

        if self.auto_reacquire_after_turn:
            self.publish_raw("C:REACQUIRE_LINE")

        if self.auto_start_after_turn:
            self.start_robot()

        self.publish_turn_done(True)
        self.publish_event("TURN_DONE")

        self.get_logger().info(
            f"Turn complete. reason={self.line_latch_reason}, "
            f"line_cross_count={self.line_cross_count}"
        )

    def should_stop_for_imu(self, error_deg: float) -> bool:
        adjusted_error = abs(error_deg)

        if self.turn_stop_early_deg > 0.0:
            adjusted_error -= self.turn_stop_early_deg

        return adjusted_error <= self.turn_tolerance_deg

    def should_stop_for_line_assist(self, progress_deg: float) -> bool:
        if not self.line_assist_enabled:
            return False

        progress_abs = abs(progress_deg)
        target_abs = abs(self.target_angle_deg)

        if progress_abs < self.line_latch_min_yaw_deg:
            self.line_confirm_count = 0
            return False

        lower_limit = max(
            self.old_line_ignore_yaw_deg,
            target_abs - self.line_accept_before_target_deg,
        )
        upper_limit = target_abs + self.line_accept_after_target_deg

        if not (lower_limit <= progress_abs <= upper_limit):
            self.line_confirm_count = 0
            return False

        if self.require_line_cross_count and self.line_cross_count < 1:
            self.line_confirm_count = 0
            return False

        if self.is_centered_narrow_line():
            self.line_confirm_count += 1
        else:
            self.line_confirm_count = 0

        if self.line_confirm_count >= self.line_confirm_frames:
            return True

        return False

    def update_line_cross_counter(self, progress_deg: float):
        progress_abs = abs(progress_deg)

        if progress_abs < self.old_line_ignore_yaw_deg:
            self.previous_line_seen_after_ignore = False
            return

        line_seen = self.is_any_narrow_line()

        if line_seen and not self.previous_line_seen_after_ignore:
            self.line_cross_count += 1
            self.publish_line_cross_count()
            self.get_logger().info(
                f"Line crossing counted during turn. "
                f"count={self.line_cross_count}, "
                f"yaw_progress={progress_deg:.1f}, "
                f"pos={self.latest_pos}, "
                f"active={self.latest_active}"
            )

        self.previous_line_seen_after_ignore = line_seen

    def stop_turn_success(self, reason: str):
        self.stop_robot()

        self.state = "SETTLING"
        self.settle_start_time = time.time()

        self.publish_event(reason)

        self.get_logger().info(
            f"Stopping turn. reason={reason}, "
            f"progress={self.get_turn_progress_deg():.1f}, "
            f"line_count={self.line_cross_count}, "
            f"pos={self.latest_pos}, "
            f"active={self.latest_active}"
        )

    def stop_turn_failed(self, reason: str):
        self.stop_robot()

        self.state = "FAILED"

        self.publish_turn_failed(True)
        self.publish_event(reason)

        self.get_logger().warn(
            f"Turn failed. reason={reason}, "
            f"progress={self.get_turn_progress_deg():.1f}, "
            f"target={self.target_angle_deg:.1f}"
        )

    # ==================================================
    # Line detection helpers
    # ==================================================

    def is_any_narrow_line(self) -> bool:
        if not self.latest_valid:
            return False

        if self.latest_wide or self.latest_solid or self.latest_marker:
            return False

        if self.latest_active < self.line_active_min:
            return False

        if self.latest_active > self.line_active_max:
            return False

        return True

    def is_centered_narrow_line(self) -> bool:
        if not self.is_any_narrow_line():
            return False

        if abs(self.latest_pos) > self.line_center_tolerance:
            return False

        return True

    # ==================================================
    # Turn measurement
    # ==================================================

    def get_turn_progress_deg(self) -> float:
        return self.accumulated_turn_deg

    # ==================================================
    # Output commands
    # ==================================================

    def command_pivot(self, pwm: int):
        pwm = int(max(0, min(255, pwm)))

        if self.turn_direction > 0:
            command = f"C:PIVOT_LEFT,{pwm}"
        else:
            command = f"C:PIVOT_RIGHT,{pwm}"

        if command == self.last_pivot_command and pwm == self.last_pwm:
            return

        self.last_pivot_command = command
        self.last_pwm = pwm

        self.publish_raw(command)

    def stop_robot(self):
        self.last_pivot_command = "NONE"
        self.last_pwm = 0

        self.publish_raw("C:STOP")

        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

    def start_robot(self):
        self.publish_raw("C:START")

        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)

    def publish_raw(self, command: str):
        msg = String()
        msg.data = command
        self.raw_pub.publish(msg)

    # ==================================================
    # Output topics
    # ==================================================

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.turn_state_pub.publish(msg)

    def publish_event(self, event: str):
        msg = String()
        msg.data = event
        self.turn_event_pub.publish(msg)

    def publish_turn_done(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.turn_done_pub.publish(msg)

    def publish_turn_failed(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.turn_failed_pub.publish(msg)

    def publish_progress(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self.turn_progress_pub.publish(msg)

    def publish_error(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self.turn_error_pub.publish(msg)

    def publish_line_cross_count(self):
        msg = String()
        msg.data = str(self.line_cross_count)
        self.line_cross_count_pub.publish(msg)

    # ==================================================
    # Text helpers
    # ==================================================

    def direction_text(self) -> str:
        if self.turn_direction > 0:
            return "LEFT"

        if self.turn_direction < 0:
            return "RIGHT"

        return "NONE"

    # ==================================================
    # Safe parsing
    # ==================================================

    @staticmethod
    def safe_int(value: str) -> int:
        try:
            return int(value)
        except Exception:
            return 0


def main(args=None):
    rclpy.init(args=args)

    node = TurnManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
