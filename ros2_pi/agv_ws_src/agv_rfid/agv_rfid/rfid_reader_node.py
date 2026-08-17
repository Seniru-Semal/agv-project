#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import RPi.GPIO as GPIO
from mfrc522 import MFRC522


def normalize_uid(uid_text: str) -> str:
    uid_text = uid_text.strip().upper().replace("-", ":").replace(" ", ":")
    parts = [p for p in uid_text.split(":") if p]
    return ":".join(f"{int(p, 16):02X}" for p in parts)


class RfidReaderNode(Node):
    def __init__(self):
        super().__init__("rfid_reader_node")

        self.declare_parameter("robot_ns", "agv_1")
        self.declare_parameter(
            "rfid_map_path",
            os.path.expanduser("~/agv_ws/src/agv_mission_manager/config/rfid_map_fleet.json"),
        )
        self.declare_parameter("poll_interval_sec", 0.02)
        self.declare_parameter("same_uid_debounce_sec", 1.5)
        self.declare_parameter("publish_unknown_uid", True)

        self.robot_ns = self.get_parameter("robot_ns").value.strip().strip("/")
        self.rfid_map_path = self.get_parameter("rfid_map_path").value
        self.poll_interval_sec = float(self.get_parameter("poll_interval_sec").value)
        self.same_uid_debounce_sec = float(
            self.get_parameter("same_uid_debounce_sec").value
        )
        self.publish_unknown_uid = bool(
            self.get_parameter("publish_unknown_uid").value
        )

        self.uid_to_node = self.load_uid_map(self.rfid_map_path)

        self.uid_pub = self.create_publisher(
            String,
            f"/{self.robot_ns}/rfid/uid",
            10,
        )
        self.node_pub = self.create_publisher(
            String,
            f"/{self.robot_ns}/rfid/node",
            10,
        )
        self.unknown_pub = self.create_publisher(
            String,
            f"/{self.robot_ns}/rfid/unknown_uid",
            10,
        )

        self.reader = MFRC522()

        self.last_uid = None
        self.last_uid_publish_time = 0.0

        self.timer = self.create_timer(self.poll_interval_sec, self.poll_once)

        self.get_logger().info(f"RFID reader started for /{self.robot_ns}")
        self.get_logger().info(f"RFID map path: {self.rfid_map_path}")
        self.get_logger().info(f"Loaded {len(self.uid_to_node)} RFID UID mappings")
        self.get_logger().info(
            f"Publishing /{self.robot_ns}/rfid/uid and /{self.robot_ns}/rfid/node"
        )

    def load_uid_map(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"RFID map file not found: {path}")
            return {}

        with open(path, "r", encoding="utf-8") as f:
            raw_map = json.load(f)

        uid_map = {}

        for uid, node_name in raw_map.items():
            try:
                uid_map[normalize_uid(uid)] = str(node_name).strip().lower()
            except Exception as exc:
                self.get_logger().warn(f"Skipping invalid UID entry {uid}: {exc}")

        return uid_map

    def uid_list_to_text(self, uid):
        # MFRC522_Anticoll normally gives 5 bytes.
        # First 4 bytes are UID, fifth byte is checksum/BCC.
        return ":".join(f"{b:02X}" for b in uid[:4])

    def publish_string(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def poll_once(self):
        try:
            status, _ = self.reader.MFRC522_Request(self.reader.PICC_REQIDL)

            if status != self.reader.MI_OK:
                return

            status, uid = self.reader.MFRC522_Anticoll()

            if status != self.reader.MI_OK:
                return

            uid_text = self.uid_list_to_text(uid)
            now = time.time()

            if (
                uid_text == self.last_uid
                and (now - self.last_uid_publish_time) < self.same_uid_debounce_sec
            ):
                return

            self.last_uid = uid_text
            self.last_uid_publish_time = now

            self.publish_string(self.uid_pub, uid_text)

            node_name = self.uid_to_node.get(uid_text)

            if node_name:
                self.publish_string(self.node_pub, node_name)
                self.get_logger().info(f"RFID detected: {uid_text} -> {node_name}")
            else:
                self.get_logger().warn(f"Unknown RFID UID detected: {uid_text}")

                if self.publish_unknown_uid:
                    self.publish_string(self.unknown_pub, uid_text)

        except Exception as exc:
            self.get_logger().error(f"RFID polling error: {exc}")

    def destroy_node(self):
        try:
            GPIO.cleanup()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RfidReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
