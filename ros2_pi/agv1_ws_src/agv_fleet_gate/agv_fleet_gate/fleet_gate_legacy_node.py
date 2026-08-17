#!/usr/bin/env python3

import json
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FleetGateNode(Node):
    def __init__(self) -> None:
        super().__init__("fleet_gate_node")

        self.declare_parameter("robot_ns", "agv_1")

        self.robot_ns = str(
            self.get_parameter("robot_ns").value
        ).strip().strip("/")

        if not self.robot_ns:
            raise RuntimeError("robot_ns must not be empty")

        prefix = f"/{self.robot_ns}"

        self.mission_id = ""
        self.path: List[str] = []

        self.current_node = ""
        self.current_index = -1
        self.released_until_index = -1

        self.mission_active = False
        self.safety_ok = True
        self.safety_hold = False

        self.pending_command: Optional[str] = None
        self.pending_command_time = 0.0

        self.last_forwarded_command = ""
        self.last_forwarded_time = 0.0

        self.junction_cmd_pub = self.create_publisher(
            String,
            f"{prefix}/junction_cmd",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            f"{prefix}/fleet/gate_state",
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/fleet/internal/junction_cmd",
            self.internal_junction_cmd_callback,
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/fleet/route",
            self.route_callback,
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/fleet/release",
            self.release_callback,
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/mission/current_node",
            self.current_node_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/mission/active",
            self.mission_active_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/safety/ok",
            self.safety_ok_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/safety_hold/active",
            self.safety_hold_callback,
            10,
        )

        self.create_timer(
            0.2,
            self.timer_callback,
        )

        self.get_logger().info(
            f"Fleet gate started for {self.robot_ns}. "
            f"Internal command topic: "
            f"{prefix}/fleet/internal/junction_cmd"
        )

    @staticmethod
    def parse_json(
        text: str,
    ) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            return None

        if not isinstance(value, dict):
            return None

        return value

    def route_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(msg.data)

        if payload is None:
            self.get_logger().error(
                "Rejected fleet route: invalid JSON"
            )
            return

        raw_path = payload.get("path")

        if (
            not isinstance(raw_path, list)
            or not raw_path
        ):
            self.get_logger().error(
                "Rejected fleet route: "
                "path must be a non-empty list"
            )
            return

        path = [
            str(node).strip().lower()
            for node in raw_path
        ]

        if any(not node for node in path):
            self.get_logger().error(
                "Rejected fleet route: "
                "path contains an empty node"
            )
            return

        self.mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        self.path = path

        try:
            released_index = int(
                payload.get(
                    "released_until_index",
                    0,
                )
            )
        except (TypeError, ValueError):
            released_index = 0

        self.released_until_index = max(
            0,
            min(
                released_index,
                len(self.path) - 1,
            ),
        )

        self.current_index = (
            self.find_current_index(
                self.current_node
            )
        )

        self.pending_command = None

        self.get_logger().info(
            f"Fleet route loaded: "
            f"mission={self.mission_id}, "
            f"path={self.path}, "
            f"released_until="
            f"{self.released_until_index}"
        )

    def release_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(msg.data)

        if payload is None:
            self.get_logger().error(
                "Rejected fleet release: invalid JSON"
            )
            return

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        if (
            self.mission_id
            and mission_id
            and mission_id != self.mission_id
        ):
            self.get_logger().warn(
                f"Ignored release for mission "
                f"{mission_id}; active mission is "
                f"{self.mission_id}"
            )
            return

        if not self.path:
            self.get_logger().warn(
                "Ignored release because no fleet "
                "route is loaded"
            )
            return

        try:
            new_index = int(
                payload[
                    "released_until_index"
                ]
            )
        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            self.get_logger().error(
                "Rejected fleet release: "
                "released_until_index is required"
            )
            return

        new_index = max(
            0,
            min(
                new_index,
                len(self.path) - 1,
            ),
        )

        if (
            new_index
            < self.released_until_index
        ):
            self.get_logger().warn(
                "Ignored release reduction"
            )
            return

        if (
            new_index
            != self.released_until_index
        ):
            self.released_until_index = (
                new_index
            )

            self.get_logger().info(
                f"Route release extended to "
                f"index {new_index} "
                f"({self.path[new_index]})"
            )

        self.try_forward_pending_command()

    def current_node_callback(
        self,
        msg: String,
    ) -> None:
        node = msg.data.strip().lower()

        if not node:
            return

        self.current_node = node

        self.current_index = (
            self.find_current_index(node)
        )

        self.try_forward_pending_command()

    def mission_active_callback(
        self,
        msg: Bool,
    ) -> None:
        self.mission_active = bool(
            msg.data
        )

        if not self.mission_active:
            self.pending_command = None

    def safety_ok_callback(
        self,
        msg: Bool,
    ) -> None:
        self.safety_ok = bool(
            msg.data
        )

        self.try_forward_pending_command()

    def safety_hold_callback(
        self,
        msg: Bool,
    ) -> None:
        self.safety_hold = bool(
            msg.data
        )

        self.try_forward_pending_command()

    def internal_junction_cmd_callback(
        self,
        msg: String,
    ) -> None:
        command = msg.data.strip()

        if not command:
            return

        self.pending_command = command

        self.pending_command_time = (
            time.time()
        )

        self.try_forward_pending_command()

    def find_current_index(
        self,
        node: str,
    ) -> int:
        if not node or not self.path:
            return -1

        start_index = max(
            0,
            self.current_index,
        )

        for index in range(
            start_index,
            len(self.path),
        ):
            if self.path[index] == node:
                return index

        for (
            index,
            path_node,
        ) in enumerate(self.path):
            if path_node == node:
                return index

        return -1

    def can_forward(self) -> bool:
        if self.pending_command is None:
            return False

        if not self.mission_active:
            return False

        if not self.safety_ok:
            return False

        if self.safety_hold:
            return False

        if not self.path:
            return False

        if self.current_index < 0:
            return False

        next_index = (
            self.current_index + 1
        )

        if next_index >= len(self.path):
            return False

        return (
            next_index
            <= self.released_until_index
        )

    def try_forward_pending_command(
        self,
    ) -> None:
        if not self.can_forward():
            return

        if self.pending_command is None:
            return

        command = self.pending_command

        output = String()
        output.data = command

        self.junction_cmd_pub.publish(
            output
        )

        self.last_forwarded_command = (
            command
        )

        self.last_forwarded_time = (
            time.time()
        )

        self.pending_command = None

        next_node = self.path[
            self.current_index + 1
        ]

        self.get_logger().info(
            f"Forwarded junction command "
            f"'{command}' from "
            f"{self.current_node} toward "
            f"{next_node}"
        )

    def timer_callback(
        self,
    ) -> None:
        self.try_forward_pending_command()

        next_node = ""

        if (
            self.path
            and 0 <= self.current_index
            < len(self.path) - 1
        ):
            next_node = self.path[
                self.current_index + 1
            ]

        waiting = (
            self.pending_command is not None
            and not self.can_forward()
        )

        payload = {
            "robot":
                self.robot_ns,

            "mission_id":
                self.mission_id,

            "current_node":
                self.current_node,

            "current_index":
                self.current_index,

            "next_node":
                next_node,

            "released_until_index":
                self.released_until_index,

            "path":
                self.path,

            "mission_active":
                self.mission_active,

            "safety_ok":
                self.safety_ok,

            "safety_hold":
                self.safety_hold,

            "waiting_for_release":
                waiting,

            "pending_command":
                self.pending_command or "",

            "pending_age_sec": (
                round(
                    time.time()
                    - self.pending_command_time,
                    3,
                )
                if self.pending_command
                is not None
                else 0.0
            ),

            "last_forwarded_command":
                self.last_forwarded_command,

            "last_forwarded_age_sec": (
                round(
                    time.time()
                    - self.last_forwarded_time,
                    3,
                )
                if self.last_forwarded_time
                > 0.0
                else 0.0
            ),
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FleetGateNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
