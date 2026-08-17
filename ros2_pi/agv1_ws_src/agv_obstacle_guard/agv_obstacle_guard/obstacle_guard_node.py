#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class ObstacleGuardNode(Node):
    def __init__(self):
        super().__init__("obstacle_guard_node")

        self.declare_parameter("robot_ns", "agv_1")
        self.declare_parameter("scan_topic", "/agv_1/scan")

        self.declare_parameter("front_center_deg", 0.0)
        self.declare_parameter("stop_half_angle_deg", 90.0)
        self.declare_parameter("warn_half_angle_deg", 90.0)

        self.declare_parameter("stop_distance_mm", 200.0)
        self.declare_parameter("warn_distance_mm", 300.0)
        self.declare_parameter("clear_distance_mm", 250.0)

        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("stop_repeat_sec", 0.2)
        self.declare_parameter("publish_stop_repeatedly", True)

        self.robot_ns = str(
            self.get_parameter("robot_ns").value
        ).strip("/")

        self.scan_topic = str(
            self.get_parameter("scan_topic").value
        )

        self.front_center_rad = math.radians(
            float(
                self.get_parameter(
                    "front_center_deg"
                ).value
            )
        )

        self.stop_half_angle_rad = math.radians(
            float(
                self.get_parameter(
                    "stop_half_angle_deg"
                ).value
            )
        )

        self.warn_half_angle_rad = math.radians(
            float(
                self.get_parameter(
                    "warn_half_angle_deg"
                ).value
            )
        )

        self.stop_distance_m = (
            float(
                self.get_parameter(
                    "stop_distance_mm"
                ).value
            )
            / 1000.0
        )

        self.warn_distance_m = (
            float(
                self.get_parameter(
                    "warn_distance_mm"
                ).value
            )
            / 1000.0
        )

        self.clear_distance_m = (
            float(
                self.get_parameter(
                    "clear_distance_mm"
                ).value
            )
            / 1000.0
        )

        self.scan_timeout_sec = float(
            self.get_parameter(
                "scan_timeout_sec"
            ).value
        )

        self.stop_repeat_sec = float(
            self.get_parameter(
                "stop_repeat_sec"
            ).value
        )

        self.publish_stop_repeatedly = bool(
            self.get_parameter(
                "publish_stop_repeatedly"
            ).value
        )

        self.state = "UNKNOWN"
        self.last_scan_time = None
        self.last_stop_publish_time = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.stop_pub = self.create_publisher(
            Bool,
            f"/{self.robot_ns}/cmd/stop",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            f"/{self.robot_ns}/obstacle/state",
            10,
        )

        self.min_distance_pub = self.create_publisher(
            Float32,
            f"/{self.robot_ns}/obstacle/min_distance_mm",
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().info("Obstacle guard started")
        self.get_logger().info(
            f"Scan topic: {self.scan_topic}"
        )
        self.get_logger().info(
            f"Stop topic: /{self.robot_ns}/cmd/stop"
        )

    @staticmethod
    def normalize_angle(angle):
        return (
            angle + math.pi
        ) % (2.0 * math.pi) - math.pi

    def angle_error(self, angle):
        return self.normalize_angle(
            angle - self.front_center_rad
        )

    def publish_state(self, new_state):
        changed = new_state != self.state
        self.state = new_state

        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)

        if changed:
            self.get_logger().info(
                f"Obstacle state: {new_state}"
            )

        return changed

    def publish_min_distance(self, distance_mm):
        msg = Float32()
        msg.data = float(distance_mm)
        self.min_distance_pub.publish(msg)

    def publish_stop(self, force=False):
        now = time.monotonic()

        if not force:
            elapsed = (
                now - self.last_stop_publish_time
            )

            if elapsed < self.stop_repeat_sec:
                return

        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

        self.last_stop_publish_time = now

    def scan_callback(self, msg):
        self.last_scan_time = time.monotonic()

        stop_ranges = []
        warning_ranges = []

        angle = msg.angle_min

        for distance in msg.ranges:
            valid = (
                math.isfinite(distance)
                and distance >= msg.range_min
                and distance <= msg.range_max
            )

            if valid:
                angle_difference = abs(
                    self.angle_error(angle)
                )

                if (
                    angle_difference
                    <= self.warn_half_angle_rad
                ):
                    warning_ranges.append(distance)

                if (
                    angle_difference
                    <= self.stop_half_angle_rad
                ):
                    stop_ranges.append(distance)

            angle += msg.angle_increment

        min_stop = (
            min(stop_ranges)
            if stop_ranges
            else None
        )

        min_warning = (
            min(warning_ranges)
            if warning_ranges
            else None
        )

        if min_warning is None:
            self.publish_min_distance(-1.0)
        else:
            self.publish_min_distance(
                min_warning * 1000.0
            )

        new_state = "CLEAR"

        if (
            min_stop is not None
            and min_stop < self.stop_distance_m
        ):
            new_state = "STOPPED"

        elif (
            self.state == "STOPPED"
            and min_stop is not None
            and min_stop < self.clear_distance_m
        ):
            new_state = "STOPPED"

        elif (
            min_warning is not None
            and min_warning < self.warn_distance_m
        ):
            new_state = "WARNING"

        changed = self.publish_state(
            new_state
        )

        if new_state == "STOPPED":
            self.publish_stop(force=changed)

    def timer_callback(self):
        now = time.monotonic()

        scan_missing = self.last_scan_time is None

        scan_stale = (
            self.last_scan_time is not None
            and (
                now - self.last_scan_time
                > self.scan_timeout_sec
            )
        )

        if scan_missing or scan_stale:
            self.publish_min_distance(-1.0)

            changed = self.publish_state(
                "SENSOR_TIMEOUT"
            )

            self.publish_stop(force=changed)
            return

        if (
            self.publish_stop_repeatedly
            and self.state == "STOPPED"
        ):
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleGuardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
