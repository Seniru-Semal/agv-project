#!/usr/bin/env python3

import time
from typing import Dict, Optional

import serial
import serial.serialutil

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Int64, Bool


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge_node")

        self.declare_parameter("port", "/dev/agv_arduino")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_delay_sec", 2.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_delay_sec = float(self.get_parameter("reconnect_delay_sec").value)

        self.serial_port: Optional[serial.Serial] = None
        self.last_reconnect_attempt = 0.0

        # Used for command arbitration.
        self.turn_state = "IDLE"

        # Publishers
        self.status_raw_pub = self.create_publisher(String, "/agv_1/status_raw", 10)
        self.state_pub = self.create_publisher(String, "/agv_1/arduino/state", 10)
        self.line_position_pub = self.create_publisher(Int32, "/agv_1/line_position", 10)
        self.left_ticks_pub = self.create_publisher(Int64, "/agv_1/left_ticks", 10)
        self.right_ticks_pub = self.create_publisher(Int64, "/agv_1/right_ticks", 10)
        self.active_sensors_pub = self.create_publisher(Int32, "/agv_1/active_sensors", 10)
        self.line_valid_pub = self.create_publisher(Bool, "/agv_1/line_valid", 10)
        self.line_lost_frames_pub = self.create_publisher(Int32, "/agv_1/line_lost_frames", 10)
        self.speed_pub = self.create_publisher(Int32, "/agv_1/base_speed", 10)
        self.estop_state_pub = self.create_publisher(Bool, "/agv_1/estop_state", 10)
        self.fault_code_pub = self.create_publisher(Int32, "/agv_1/fault_code", 10)
        self.bridge_connected_pub = self.create_publisher(Bool, "/agv_1/bridge_connected", 10)
        self.last_command_pub = self.create_publisher(String, "/agv_1/last_command_sent", 10)

        # Command subscribers
        self.create_subscription(Bool, "/agv_1/cmd/start", self.start_callback, 10)
        self.create_subscription(Bool, "/agv_1/cmd/stop", self.stop_callback, 10)
        self.create_subscription(Bool, "/agv_1/cmd/estop", self.estop_callback, 10)
        self.create_subscription(Bool, "/agv_1/cmd/reset", self.reset_callback, 10)
        self.create_subscription(Bool, "/agv_1/cmd/reset_ticks", self.reset_ticks_callback, 10)
        self.create_subscription(Int32, "/agv_1/cmd/speed", self.speed_callback, 10)
        self.create_subscription(String, "/agv_1/cmd/pid", self.pid_callback, 10)
        self.create_subscription(String, "/agv_1/cmd/raw", self.raw_command_callback, 10)

        # Turn manager state subscriber.
        self.create_subscription(String, "/agv_1/turn/state", self.turn_state_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("AGV Arduino bridge started")
        self.get_logger().info(f"Port: {self.port}")
        self.get_logger().info(f"Baud: {self.baud}")

    # ==================================================
    # Serial connection
    # ==================================================

    def connect_serial(self) -> bool:
        now = time.time()

        if now - self.last_reconnect_attempt < self.reconnect_delay_sec:
            return False

        self.last_reconnect_attempt = now

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.02,
                write_timeout=0.1,
            )

            time.sleep(0.2)

            self.get_logger().info(f"Connected to Arduino on {self.port}")
            self.publish_bridge_connected(True)
            return True

        except serial.serialutil.SerialException as exc:
            self.serial_port = None
            self.publish_bridge_connected(False)
            self.get_logger().warn(f"Could not open {self.port}: {exc}")
            return False

    def disconnect_serial(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

        self.serial_port = None
        self.publish_bridge_connected(False)

    # ==================================================
    # Main read loop
    # ==================================================

    def timer_callback(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect_serial()
            return

        try:
            raw_bytes = self.serial_port.readline()

            if not raw_bytes:
                return

            line = raw_bytes.decode("utf-8", errors="ignore").strip()

            if not line:
                return

            self.publish_raw(line)

            data = self.parse_status_line(line)

            if data is None:
                return

            self.publish_parsed_status(data)

        except serial.serialutil.SerialException as exc:
            self.get_logger().warn(f"Serial error: {exc}")
            self.disconnect_serial()

        except Exception as exc:
            self.get_logger().warn(f"Unexpected parse/read error: {exc}")

    # ==================================================
    # Parser
    # ==================================================

    def parse_status_line(self, line: str) -> Optional[Dict[str, str]]:
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

        required_keys = ["STATE", "POS", "L", "R", "ACTIVE", "VALID", "LOST", "FAULT"]

        for key in required_keys:
            if key not in data:
                self.get_logger().warn(f"Missing key {key} in line: {line}")
                return None

        return data

    # ==================================================
    # Publishers
    # ==================================================

    def publish_raw(self, line: str):
        msg = String()
        msg.data = line
        self.status_raw_pub.publish(msg)

    def publish_parsed_status(self, data: Dict[str, str]):
        state_msg = String()
        state_msg.data = data["STATE"]
        self.state_pub.publish(state_msg)

        pos_msg = Int32()
        pos_msg.data = self.safe_int(data["POS"])
        self.line_position_pub.publish(pos_msg)

        left_msg = Int64()
        left_msg.data = self.safe_int(data["L"])
        self.left_ticks_pub.publish(left_msg)

        right_msg = Int64()
        right_msg.data = self.safe_int(data["R"])
        self.right_ticks_pub.publish(right_msg)

        active_msg = Int32()
        active_msg.data = self.safe_int(data["ACTIVE"])
        self.active_sensors_pub.publish(active_msg)

        valid_msg = Bool()
        valid_msg.data = self.safe_int(data["VALID"]) == 1
        self.line_valid_pub.publish(valid_msg)

        lost_msg = Int32()
        lost_msg.data = self.safe_int(data["LOST"])
        self.line_lost_frames_pub.publish(lost_msg)

        if "SPEED" in data:
            speed_msg = Int32()
            speed_msg.data = self.safe_int(data["SPEED"])
            self.speed_pub.publish(speed_msg)

        if "ESTOP" in data:
            estop_msg = Bool()
            estop_msg.data = self.safe_int(data["ESTOP"]) == 1
            self.estop_state_pub.publish(estop_msg)

        fault_msg = Int32()
        fault_msg.data = self.safe_int(data["FAULT"])
        self.fault_code_pub.publish(fault_msg)

    def publish_bridge_connected(self, connected: bool):
        msg = Bool()
        msg.data = connected
        self.bridge_connected_pub.publish(msg)

    def publish_last_command(self, command: str):
        msg = String()
        msg.data = command
        self.last_command_pub.publish(msg)

    # ==================================================
    # Turn-state arbitration
    # ==================================================

    def turn_state_callback(self, msg: String):
        self.turn_state = msg.data.strip()

    def turn_active(self) -> bool:
        return self.turn_state in [
            "TURNING",
            "SETTLING",
            "SEARCHING_LINE",
            "REACQUIRE_SETTLING",
        ]

    def raw_command_allowed_during_turn(self, command: str) -> bool:
        if command == "C:STOP":
            return True

        if command == "C:ESTOP":
            return True

        if command == "C:START":
            return True

        if command == "C:REACQUIRE_LINE":
            return True

        if command == "C:CLEAR_BRANCH":
            return True

        if command.startswith("C:SET_BRANCH,"):
            return True

        if command.startswith("C:PIVOT_LEFT,"):
            return True

        if command.startswith("C:PIVOT_RIGHT,"):
            return True

        # Allow feature_action_node to clear the junction immediately after
        # turn_manager publishes /agv_1/turn/done. The bridge may still have the
        # previous turn_state as SETTLING for a few milliseconds due to ROS
        # callback ordering, so RAW_DRIVE must be allowed in that state.
        #
        # Keep it blocked during active TURNING to avoid interrupting the pivot.
        if command.startswith("C:RAW_DRIVE,"):
            return self.turn_state in [
                "SETTLING",
                "REACQUIRE_SETTLING",
                "DONE",
                "FAILED",
                "TIMEOUT",
                "IDLE",
            ]

        return False

    # ==================================================
    # ROS command callbacks
    # ==================================================

    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.turn_active():
            self.get_logger().warn("Rejected START because IMU turn/reacquire is active")
            return

        self.send_command("C:START")

    def stop_callback(self, msg: Bool):
        if not msg.data:
            return

        self.send_command("C:STOP")

    def estop_callback(self, msg: Bool):
        if not msg.data:
            return

        self.send_command("C:ESTOP")

    def reset_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.turn_active():
            self.get_logger().warn("Rejected RESET because IMU turn/reacquire is active")
            return

        self.send_command("C:RESET")

    def reset_ticks_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.turn_active():
            self.get_logger().warn("Rejected RESET_TICKS because IMU turn/reacquire is active")
            return

        self.send_command("C:RESET_TICKS")

    def speed_callback(self, msg: Int32):
        if self.turn_active():
            self.get_logger().warn("Rejected SET_SPEED because IMU turn/reacquire is active")
            return

        speed = max(0, min(255, int(msg.data)))
        self.send_command(f"C:SET_SPEED,{speed}")

    def pid_callback(self, msg: String):
        if self.turn_active():
            self.get_logger().warn("Rejected SET_PID because IMU turn/reacquire is active")
            return

        text = msg.data.strip()

        if not text:
            return

        self.send_command(f"C:SET_PID,{text}")

    def raw_command_callback(self, msg: String):
        command = msg.data.strip()

        if not command:
            return

        if not command.startswith("C:"):
            self.get_logger().warn(f"Rejected raw command without C: prefix: {command}")
            return

        if self.turn_active():
            if not self.raw_command_allowed_during_turn(command):
                self.get_logger().warn(
                    f"Rejected raw command during IMU turn/reacquire: {command}"
                )
                return

        self.send_command(command)

    # ==================================================
    # Serial command write
    # ==================================================

    def send_command(self, command: str) -> bool:
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn(f"Cannot send command, serial not connected: {command}")
            return False

        try:
            line = command.strip() + "\n"
            self.serial_port.write(line.encode("utf-8"))
            self.serial_port.flush()

            self.publish_last_command(command)
            self.get_logger().info(f"Sent command: {command}")
            return True

        except serial.serialutil.SerialException as exc:
            self.get_logger().warn(f"Serial write error: {exc}")
            self.disconnect_serial()
            return False

        except Exception as exc:
            self.get_logger().warn(f"Unexpected command write error: {exc}")
            return False

    # ==================================================
    # Utility
    # ==================================================

    @staticmethod
    def safe_int(value: str) -> int:
        try:
            return int(value)
        except ValueError:
            return 0


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
