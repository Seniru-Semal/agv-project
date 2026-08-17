#!/usr/bin/env python3

import json
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
)

import rclpy
from std_msgs.msg import String

from .fleet_manager_node import (
    FleetManagerNode,
    RobotState,
    edge_key,
)


class DockSafeFleetManagerNode(
    FleetManagerNode
):
    """
    Adds persistent berth and egress protection to the existing
    rolling-reservation fleet manager.

    A robot docked at a station or home owns:

      1. the dock node;
      2. every configured egress node;
      3. every dock-to-egress edge.

    Locks remain after mission completion and are released only
    after the robot reports another physical node.
    """

    def __init__(self) -> None:
        self.dock_locks: Dict[
            str,
            str,
        ] = {}

        self.egress_node_locks: Dict[
            str,
            str,
        ] = {}

        self.egress_edge_locks: Dict[
            Tuple[str, str],
            str,
        ] = {}

        super().__init__()

        self.dock_state_pub = (
            self.create_publisher(
                String,
                "/fleet/dock_state",
                10,
            )
        )

        for robot in self.robots.values():
            setattr(
                robot,
                "docked_at",
                "",
            )

            self.claim_dock_protection(
                robot,
                robot.current_node,
                publish_event=False,
            )

        self.publish_event(
            "DOCK_EGRESS_PROTECTION_ENABLED"
        )

    def is_persistent_dock(
        self,
        node: str,
    ) -> bool:
        return bool(
            self.nodes.get(
                node,
                {},
            ).get(
                "persistent_occupancy",
                False,
            )
        )

    def dock_egress_nodes(
        self,
        dock_node: str,
    ) -> List[str]:
        raw = self.nodes.get(
            dock_node,
            {},
        ).get(
            "egress_nodes",
            [],
        )

        if not isinstance(raw, list):
            return []

        result: List[str] = []

        for value in raw:
            egress_node = (
                str(value)
                .strip()
                .lower()
            )

            if (
                egress_node
                and egress_node in self.nodes
                and edge_key(
                    dock_node,
                    egress_node,
                ) in self.connections
            ):
                result.append(
                    egress_node
                )

        return result

    def claim_dock_protection(
        self,
        robot: RobotState,
        dock_node: str,
        publish_event: bool = True,
    ) -> None:
        if not self.is_persistent_dock(
            dock_node
        ):
            setattr(
                robot,
                "docked_at",
                "",
            )
            return

        dock_owner = self.dock_locks.get(
            dock_node
        )

        if dock_owner not in (
            None,
            robot.name,
        ):
            self.publish_event(
                "DOCK_LOCK_CONFLICT "
                f"dock={dock_node} "
                f"owner={dock_owner} "
                f"claimant={robot.name}"
            )
            return

        self.dock_locks[
            dock_node
        ] = robot.name

        setattr(
            robot,
            "docked_at",
            dock_node,
        )

        for egress_node in (
            self.dock_egress_nodes(
                dock_node
            )
        ):
            node_owner = (
                self.egress_node_locks.get(
                    egress_node
                )
            )

            if node_owner not in (
                None,
                robot.name,
            ):
                self.publish_event(
                    "DOCK_EGRESS_LOCK_CONFLICT "
                    f"dock={dock_node} "
                    f"egress={egress_node} "
                    f"owner={node_owner} "
                    f"claimant={robot.name}"
                )
                continue

            edge = edge_key(
                dock_node,
                egress_node,
            )

            edge_owner = (
                self.egress_edge_locks.get(
                    edge
                )
            )

            if edge_owner not in (
                None,
                robot.name,
            ):
                self.publish_event(
                    "DOCK_EGRESS_EDGE_CONFLICT "
                    f"dock={dock_node} "
                    f"edge={edge[0]}--"
                    f"{edge[1]} "
                    f"owner={edge_owner} "
                    f"claimant={robot.name}"
                )
                continue

            self.egress_node_locks[
                egress_node
            ] = robot.name

            self.egress_edge_locks[
                edge
            ] = robot.name

        if publish_event:
            self.publish_event(
                "DOCK_PROTECTED "
                f"robot={robot.name} "
                f"dock={dock_node} "
                "egress="
                f"{self.dock_egress_nodes(dock_node)}"
            )

    def release_dock_protection(
        self,
        robot: RobotState,
        dock_node: str,
        publish_event: bool = True,
    ) -> None:
        if (
            self.dock_locks.get(
                dock_node
            )
            == robot.name
        ):
            self.dock_locks.pop(
                dock_node,
                None,
            )

        for egress_node in (
            self.dock_egress_nodes(
                dock_node
            )
        ):
            if (
                self.egress_node_locks.get(
                    egress_node
                )
                == robot.name
            ):
                self.egress_node_locks.pop(
                    egress_node,
                    None,
                )

            edge = edge_key(
                dock_node,
                egress_node,
            )

            if (
                self.egress_edge_locks.get(
                    edge
                )
                == robot.name
            ):
                self.egress_edge_locks.pop(
                    edge,
                    None,
                )

        if (
            getattr(
                robot,
                "docked_at",
                "",
            )
            == dock_node
        ):
            setattr(
                robot,
                "docked_at",
                "",
            )

        if publish_event:
            self.publish_event(
                "DOCK_RELEASED "
                f"robot={robot.name} "
                f"dock={dock_node}"
            )

    def current_node_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        node = (
            msg.data
            .strip()
            .lower()
        )

        if (
            not node
            or node not in self.nodes
        ):
            return

        robot = self.robots[
            robot_name
        ]

        old_node = robot.current_node

        if node == old_node:
            if self.is_persistent_dock(
                node
            ):
                self.claim_dock_protection(
                    robot,
                    node,
                    publish_event=False,
                )

            return

        old_dock = getattr(
            robot,
            "docked_at",
            "",
        )

        if (
            old_dock
            and old_dock == old_node
        ):
            self.release_dock_protection(
                robot,
                old_dock,
            )

        if self.is_persistent_dock(
            node
        ):
            self.claim_dock_protection(
                robot,
                node,
            )

        super().current_node_callback(
            robot_name,
            msg,
        )

    def resource_conflict(
        self,
        robot: RobotState,
        from_node: str,
        to_node: str,
    ) -> Optional[
        Tuple[str, str, str]
    ]:
        dock_owner = self.dock_locks.get(
            to_node
        )

        if dock_owner not in (
            None,
            robot.name,
        ):
            return (
                "DOCK_OCCUPIED",
                to_node,
                dock_owner,
            )

        egress_owner = (
            self.egress_node_locks.get(
                to_node
            )
        )

        if egress_owner not in (
            None,
            robot.name,
        ):
            return (
                "DOCK_EGRESS_PROTECTED",
                to_node,
                egress_owner,
            )

        edge = edge_key(
            from_node,
            to_node,
        )

        edge_owner = (
            self.egress_edge_locks.get(
                edge
            )
        )

        if edge_owner not in (
            None,
            robot.name,
        ):
            return (
                "DOCK_EGRESS_EDGE_PROTECTED",
                f"{edge[0]}--{edge[1]}",
                edge_owner,
            )

        return super().resource_conflict(
            robot,
            from_node,
            to_node,
        )

    def clear_reservations_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            super().clear_reservations_callback(
                msg
            )
            return

        robot_name = str(
            payload.get(
                "robot",
                "",
            )
        ).strip()

        confirmed_node = str(
            payload.get(
                "confirmed_node",
                "",
            )
        ).strip().lower()

        if (
            robot_name not in self.robots
            or confirmed_node
            not in self.nodes
        ):
            super().clear_reservations_callback(
                msg
            )
            return

        robot = self.robots[
            robot_name
        ]

        if robot.mission_active:
            super().clear_reservations_callback(
                msg
            )
            return

        old_dock = getattr(
            robot,
            "docked_at",
            "",
        )

        if old_dock:
            self.release_dock_protection(
                robot,
                old_dock,
            )

        super().clear_reservations_callback(
            msg
        )

        if self.is_persistent_dock(
            confirmed_node
        ):
            self.claim_dock_protection(
                robot,
                confirmed_node,
            )

    def publish_state(self) -> None:
        super().publish_state()

        payload: Dict[str, Any] = {
            "dock_locks":
                self.dock_locks,

            "egress_node_locks":
                self.egress_node_locks,

            "egress_edge_locks": [
                {
                    "edge":
                        list(edge),

                    "robot":
                        owner,
                }
                for (
                    edge,
                    owner,
                ) in sorted(
                    self.egress_edge_locks.items()
                )
            ],

            "robots": {
                name: {
                    "current_node":
                        robot.current_node,

                    "docked_at":
                        getattr(
                            robot,
                            "docked_at",
                            "",
                        ),
                }
                for (
                    name,
                    robot,
                ) in self.robots.items()
            },
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        self.dock_state_pub.publish(
            msg
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DockSafeFleetManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
