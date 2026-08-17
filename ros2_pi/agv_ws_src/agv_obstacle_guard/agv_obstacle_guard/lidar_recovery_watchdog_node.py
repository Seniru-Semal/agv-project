#!/usr/bin/env python3

import glob
import os
import signal
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


class LidarRecoveryWatchdogNode(Node):
    def __init__(self):
        super().__init__(
            "lidar_recovery_watchdog_node"
        )

        self.declare_parameter(
            "scan_topic",
            "/agv_1/scan",
        )

        self.declare_parameter(
            "lidar_device",
            "/dev/agv1_lidar",
        )

        self.declare_parameter(
            "lidar_node_name",
            "agv1_rplidar",
        )

        self.declare_parameter(
            "driver_executable",
            "rplidar_node",
        )

        self.declare_parameter(
            "scan_timeout_sec",
            1.5,
        )

        self.declare_parameter(
            "startup_grace_sec",
            5.0,
        )

        self.declare_parameter(
            "device_settle_sec",
            1.0,
        )

        self.declare_parameter(
            "restart_cooldown_sec",
            5.0,
        )

        self.scan_topic = str(
            self.get_parameter(
                "scan_topic"
            ).value
        )

        self.lidar_device = str(
            self.get_parameter(
                "lidar_device"
            ).value
        )

        self.lidar_node_name = str(
            self.get_parameter(
                "lidar_node_name"
            ).value
        )

        self.driver_executable = str(
            self.get_parameter(
                "driver_executable"
            ).value
        )

        self.scan_timeout_sec = float(
            self.get_parameter(
                "scan_timeout_sec"
            ).value
        )

        self.startup_grace_sec = float(
            self.get_parameter(
                "startup_grace_sec"
            ).value
        )

        self.device_settle_sec = float(
            self.get_parameter(
                "device_settle_sec"
            ).value
        )

        self.restart_cooldown_sec = float(
            self.get_parameter(
                "restart_cooldown_sec"
            ).value
        )

        now = time.monotonic()

        self.start_time = now
        self.last_scan_time = None
        self.device_detected_time = None
        self.last_restart_time = 0.0
        self.last_missing_device_log = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.timer = self.create_timer(
            0.25,
            self.timer_callback,
        )

        self.get_logger().info(
            "Lidar recovery watchdog started"
        )

    def scan_callback(self, _msg):
        self.last_scan_time = time.monotonic()
        self.device_detected_time = None

    def scan_is_fresh(self, now):
        if self.last_scan_time is None:
            return False

        return (
            now - self.last_scan_time
            <= self.scan_timeout_sec
        )

    def find_driver_processes(self):
        matching_pids = []

        for cmdline_path in glob.glob(
            "/proc/[0-9]*/cmdline"
        ):
            try:
                pid = int(
                    cmdline_path.split("/")[2]
                )

                if pid == os.getpid():
                    continue

                with open(
                    cmdline_path,
                    "rb",
                ) as file:
                    raw_cmdline = file.read()

                cmdline = raw_cmdline.replace(
                    b"\x00",
                    b" ",
                ).decode(
                    errors="ignore"
                )

                correct_executable = (
                    self.driver_executable
                    in cmdline
                )

                correct_node = (
                    f"__node:={self.lidar_node_name}"
                    in cmdline
                )

                if (
                    correct_executable
                    and correct_node
                ):
                    matching_pids.append(pid)

            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                ValueError,
            ):
                continue

        return matching_pids

    def restart_stalled_driver(self):
        pids = self.find_driver_processes()

        if not pids:
            self.get_logger().warn(
                "No matching stalled lidar "
                "process was found"
            )
            return

        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)

                self.get_logger().warn(
                    "Force-stopped stalled lidar "
                    f"driver PID {pid}"
                )

            except ProcessLookupError:
                pass

            except PermissionError:
                self.get_logger().error(
                    "Permission denied stopping "
                    f"lidar PID {pid}"
                )

    def timer_callback(self):
        now = time.monotonic()

        if (
            now - self.start_time
            < self.startup_grace_sec
        ):
            return

        if self.scan_is_fresh(now):
            self.device_detected_time = None
            return

        if not os.path.exists(
            self.lidar_device
        ):
            self.device_detected_time = None

            if (
                now - self.last_missing_device_log
                >= 5.0
            ):
                self.get_logger().warn(
                    "No scan and lidar device "
                    f"is absent: {self.lidar_device}"
                )

                self.last_missing_device_log = now

            return

        if self.device_detected_time is None:
            self.device_detected_time = now

            self.get_logger().info(
                "Lidar device detected. "
                "Waiting for USB to settle."
            )
            return

        if (
            now - self.device_detected_time
            < self.device_settle_sec
        ):
            return

        if (
            now - self.last_restart_time
            < self.restart_cooldown_sec
        ):
            return

        self.last_restart_time = now
        self.device_detected_time = None

        self.get_logger().warn(
            "Lidar device is present but "
            "scan has not resumed. "
            "Restarting driver."
        )

        self.restart_stalled_driver()


def main(args=None):
    rclpy.init(args=args)

    node = LidarRecoveryWatchdogNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
