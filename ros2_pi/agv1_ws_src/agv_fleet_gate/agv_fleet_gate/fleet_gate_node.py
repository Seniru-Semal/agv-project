#!/usr/bin/env python3

from __future__ import annotations

import json
import time

from typing import (
    Any,
    Dict,
    List,
    Optional,
)

import rclpy

from std_msgs.msg import String

from .fleet_gate_legacy_node import (
    FleetGateNode
    as LegacyFleetGateNode,
)


class FleetGateNode(
    LegacyFleetGateNode
):
    """
    AGV1 revision-aware route-release gate.

    The original safety and command-gating
    behavior remains inherited from the
    legacy gate.
    """

    def __init__(self) -> None:
        self.route_revision = 0

        super().__init__()

        prefix = f"/{self.robot_ns}"

        self.route_ack_pub = (
            self.create_publisher(
                String,
                f"{prefix}/fleet/route_ack",
                10,
            )
        )

        self.get_logger().info(
            "Revision-aware fleet gate "
            f"enabled for {self.robot_ns}"
        )

    @staticmethod
    def parse_json_dict(
        text: str,
    ) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(
                text
            )

        except json.JSONDecodeError:
            return None

        if not isinstance(
            value,
            dict,
        ):
            return None

        return value

    def publish_route_ack(
        self,
    ) -> None:
        payload = {
            "robot":
                self.robot_ns,

            "mission_id":
                self.mission_id,

            "route_revision":
                self.route_revision,

            "current_node":
                self.current_node,

            "current_index":
                self.current_index,

            "path":
                self.path,

            "released_until_index":
                self.released_until_index,
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        for _ in range(3):
            self.route_ack_pub.publish(
                msg
            )
            time.sleep(0.02)

    # ==================================================
    # Revision-aware route loading
    # ==================================================

    def route_callback(
        self,
        msg: String,
    ) -> None:
        payload = (
            self.parse_json_dict(
                msg.data
            )
        )

        if payload is None:
            self.get_logger().error(
                "Rejected fleet route: "
                "invalid JSON"
            )
            return

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        try:
            route_revision = int(
                payload.get(
                    "route_revision",
                    0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):
            route_revision = 0

        raw_path = payload.get(
            "path",
            [],
        )

        if not mission_id:
            self.get_logger().error(
                "Rejected fleet route: "
                "missing mission_id"
            )
            return

        if route_revision <= 0:
            self.get_logger().error(
                "Rejected fleet route: "
                "invalid route_revision"
            )
            return

        if (
            not isinstance(
                raw_path,
                list,
            )
            or not raw_path
        ):
            self.get_logger().error(
                "Rejected fleet route: "
                "path must be non-empty"
            )
            return

        path: List[str] = [
            str(node)
            .strip()
            .lower()
            for node in raw_path
        ]

        if any(
            not node
            for node in path
        ):
            self.get_logger().error(
                "Rejected fleet route: "
                "path contains empty node"
            )
            return

        if (
            self.mission_id
            == mission_id
            and route_revision
            < self.route_revision
        ):
            self.get_logger().warn(
                "Ignored older route revision"
            )
            return

        try:
            released_index = int(
                payload.get(
                    "released_until_index",
                    0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):
            released_index = 0

        self.mission_id = (
            mission_id
        )

        self.route_revision = (
            route_revision
        )

        self.path = path

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

        # Any command generated from the
        # previous route must not survive
        # a route replacement.
        self.pending_command = None
        self.pending_command_time = 0.0

        self.publish_route_ack()

        self.get_logger().info(
            "Fleet route loaded: "
            f"mission={self.mission_id}, "
            f"revision={self.route_revision}, "
            "released_until="
            f"{self.released_until_index}, "
            f"path={self.path}"
        )

    # ==================================================
    # Revision-aware release
    # ==================================================

    def release_callback(
        self,
        msg: String,
    ) -> None:
        payload = (
            self.parse_json_dict(
                msg.data
            )
        )

        if payload is None:
            self.get_logger().error(
                "Rejected fleet release: "
                "invalid JSON"
            )
            return

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        try:
            route_revision = int(
                payload.get(
                    "route_revision",
                    0,
                )
            )

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
                "invalid fields"
            )
            return

        if mission_id != self.mission_id:
            self.get_logger().warn(
                "Ignored release for "
                "different mission"
            )
            return

        if (
            route_revision
            != self.route_revision
        ):
            self.get_logger().warn(
                "Ignored release for "
                "different route revision"
            )
            return

        if not self.path:
            self.get_logger().warn(
                "Ignored release because "
                "no route is loaded"
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
                "Route release extended "
                f"to index {new_index} "
                f"({self.path[new_index]})"
            )

        self.try_forward_pending_command()

    # ==================================================
    # Gate-state publication
    # ==================================================

    def timer_callback(
        self,
    ) -> None:
        self.try_forward_pending_command()

        next_node = ""

        if (
            self.path
            and 0
            <= self.current_index
            < len(self.path) - 1
        ):
            next_node = self.path[
                self.current_index + 1
            ]

        waiting = (
            self.pending_command
            is not None
            and not self.can_forward()
        )

        payload = {
            "robot":
                self.robot_ns,

            "mission_id":
                self.mission_id,

            "route_revision":
                self.route_revision,

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
                self.pending_command
                or "",

            "pending_age_sec":
                (
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

            "last_forwarded_age_sec":
                (
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

        self.state_pub.publish(
            msg
        )


def main(args=None) -> None:
    rclpy.init(
        args=args
    )

    node = FleetGateNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
