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

from .mission_manager_legacy_node import (
    MissionManagerNode
    as LegacyMissionManagerNode,
)


class MissionManagerNode(
    LegacyMissionManagerNode
):
    """
    AGV1 exact-path protocol adapter.

    The original AGV1 mission logic remains in
    mission_manager_legacy_node.py.

    This adapter adds:

      /agv_1/mission/load_path
      /agv_1/mission/start_path
      /agv_1/mission/path_ack

    Existing station-exit, RFID, encoder,
    feature-action and junction behavior is
    inherited unchanged.
    """

    def __init__(self) -> None:
        super().__init__()

        self.mission_id = ""
        self.route_revision = 0

        self.path_candidate: Optional[
            Dict[str, Any]
        ] = None

        self.replan_previous_node = ""

        self.feature_action_state = (
            "UNKNOWN"
        )

        self.path_ack_pub = (
            self.create_publisher(
                String,
                "/agv_1/mission/path_ack",
                10,
            )
        )

        self.create_subscription(
            String,
            "/agv_1/mission/load_path",
            self.load_path_callback,
            10,
        )

        self.create_subscription(
            String,
            "/agv_1/mission/start_path",
            self.start_path_callback,
            10,
        )

        self.create_subscription(
            String,
            "/agv_1/feature_action/state",
            self.feature_action_state_callback,
            10,
        )

        self.get_logger().info(
            "AGV1 exact-path mission "
            "adapter started"
        )

    # ==================================================
    # JSON and state helpers
    # ==================================================

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

    def feature_action_state_callback(
        self,
        msg: String,
    ) -> None:
        state = (
            msg.data
            .strip()
            .upper()
        )

        if state:
            self.feature_action_state = (
                state
            )

    def station_exit_busy_compat(
        self,
    ) -> bool:
        return bool(
            self.waiting_for_station_exit_turn_done
            or self.station_exit_raw_active
            or self.state in [
                "EXITING_DEAD_END_STATION_TURN",
                "WAITING_AFTER_STATION_EXIT_TURN",
                "EXITING_STATION_RAW_DRIVE",
            ]
        )

    def connection_exists(
        self,
        node_a: str,
        node_b: str,
    ) -> bool:
        return (
            (
                node_a,
                node_b,
            ) in getattr(
                self,
                "directed_connections",
                [],
            )
            or
            (
                node_a,
                node_b,
            ) in self.connections
            or (
                node_b,
                node_a,
            ) in self.connections
        )

    # ==================================================
    # Exact-path validation
    # ==================================================

    def validate_supplied_path(
        self,
        path: List[str],
        destination: str,
    ) -> str:
        if not path:
            return "empty_path"

        if path[0] != self.current_node:
            return (
                "wrong_start_"
                f"expected_{self.current_node}_"
                f"got_{path[0]}"
            )

        if path[-1] != destination:
            return (
                "wrong_destination_"
                f"expected_{destination}_"
                f"got_{path[-1]}"
            )

        for node in path:
            if node not in self.nodes:
                return (
                    "unknown_node_"
                    f"{node}"
                )

        for (
            node_a,
            node_b,
        ) in zip(
            path,
            path[1:],
        ):
            if not self.connection_exists(
                node_a,
                node_b,
            ):
                return (
                    "invalid_edge_"
                    f"{node_a}_{node_b}"
                )

        for node in path[1:-1]:
            if not self.is_junction(
                node
            ):
                return (
                    "station_used_as_transit_"
                    f"{node}"
                )

        for index in range(
            1,
            len(path) - 1,
        ):
            previous = path[
                index - 1
            ]

            at_node = path[
                index
            ]

            next_node = path[
                index + 1
            ]

            if not self.is_junction(
                at_node
            ):
                continue

            if self.is_junction_exit_transition(
                previous,
                at_node,
            ):
                continue

            command = (
                self.compute_junction_command(
                    previous,
                    at_node,
                    next_node,
                )
            )

            if command is None:
                return (
                    "no_junction_command_"
                    f"{previous}_{at_node}_"
                    f"{next_node}"
                )

        return ""

    def active_replacement_safe(
        self,
    ) -> bool:
        if not self.active:
            return False

        if not self.is_junction(
            self.current_node
        ):
            return False

        if self.station_exit_busy_compat():
            return False

        if (
            self.feature_action_state
            != "WAITING_FOR_JUNCTION_COMMAND"
        ):
            return False

        if not str(
            self.state
        ).startswith(
            "COMMAND_SENT_"
        ):
            return False

        if self.path_index <= 0:
            return False

        return True

    # ==================================================
    # ACK publisher
    # ==================================================

    def publish_path_ack(
        self,
        accepted: bool,
        mission_id: str,
        route_revision: int,
        destination: str,
        path: List[str],
        reason: str = "",
    ) -> None:
        payload = {
            "robot":
                "agv_1",

            "accepted":
                bool(accepted),

            "mission_id":
                mission_id,

            "route_revision":
                int(route_revision),

            "current_node":
                self.current_node,

            "destination":
                destination,

            "path":
                path,

            "reason":
                reason,
        }

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        for _ in range(8):
            self.path_ack_pub.publish(
                msg
            )
            time.sleep(0.05)

    # ==================================================
    # Load and validate VM path
    # ==================================================

    def load_path_callback(
        self,
        msg: String,
    ) -> None:
        payload = (
            self.parse_json_dict(
                msg.data
            )
        )

        if payload is None:
            self.get_logger().warn(
                "Rejected exact path: "
                "invalid JSON"
            )
            return

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        destination = str(
            payload.get(
                "destination",
                "",
            )
        ).strip().lower()

        replace_active = bool(
            payload.get(
                "replace_active_route",
                False,
            )
        )

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

        if not isinstance(
            raw_path,
            list,
        ):
            raw_path = []

        path = [
            str(node)
            .strip()
            .lower()
            for node in raw_path
            if str(node).strip()
        ]

        reason = ""

        if not mission_id:
            reason = (
                "missing_mission_id"
            )

        elif route_revision <= 0:
            reason = (
                "invalid_route_revision"
            )

        elif destination not in self.nodes:
            reason = (
                "unknown_destination"
            )

        elif replace_active:
            if (
                mission_id
                != self.mission_id
            ):
                reason = (
                    "active_mission_id_mismatch"
                )

            elif (
                route_revision
                <= self.route_revision
            ):
                reason = (
                    "route_revision_not_newer"
                )

            elif (
                destination
                != self.destination_node
            ):
                reason = (
                    "active_destination_changed"
                )

            elif not self.active_replacement_safe():
                reason = (
                    "active_replacement_not_safe"
                )

        elif self.active:
            reason = (
                "mission_already_active"
            )

        if not reason:
            reason = (
                self.validate_supplied_path(
                    path,
                    destination,
                )
            )

        if reason:
            self.publish_path_ack(
                False,
                mission_id,
                route_revision,
                destination,
                path,
                reason,
            )

            self.get_logger().warn(
                "Exact path rejected: "
                f"mission={mission_id}, "
                f"revision={route_revision}, "
                f"reason={reason}"
            )

            return

        if replace_active:
            self.replan_previous_node = (
                self.path_nodes[
                    self.path_index - 1
                ]
            )

        else:
            self.replan_previous_node = ""

        self.path_candidate = {
            "mission_id":
                mission_id,

            "route_revision":
                route_revision,

            "destination":
                destination,

            "path":
                path,

            "replace_active_route":
                replace_active,
        }

        self.publish_path_ack(
            True,
            mission_id,
            route_revision,
            destination,
            path,
        )

        self.get_logger().info(
            "Exact path accepted: "
            f"mission={mission_id}, "
            f"revision={route_revision}, "
            f"replace={replace_active}, "
            f"path={path}"
        )

    # ==================================================
    # Start exact path after VM/gate approval
    # ==================================================

    def start_path_callback(
        self,
        msg: String,
    ) -> None:
        payload = (
            self.parse_json_dict(
                msg.data
            )
        )

        if payload is None:
            return

        candidate = (
            self.path_candidate
        )

        if candidate is None:
            self.get_logger().warn(
                "Ignored start_path: "
                "no validated path is loaded"
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
            return

        if (
            mission_id
            != candidate[
                "mission_id"
            ]
        ):
            self.get_logger().warn(
                "Ignored start_path: "
                "mission ID mismatch"
            )
            return

        if (
            route_revision
            != candidate[
                "route_revision"
            ]
        ):
            self.get_logger().warn(
                "Ignored start_path: "
                "route revision mismatch"
            )
            return

        if candidate[
            "replace_active_route"
        ]:
            success = (
                self.commit_active_replacement(
                    candidate
                )
            )

        else:
            success = (
                self.start_supplied_path(
                    candidate
                )
            )

        if success:
            self.path_candidate = None

    # ==================================================
    # Initial exact mission
    # ==================================================

    def start_supplied_path(
        self,
        candidate: Dict[
            str,
            Any,
        ],
    ) -> bool:
        if self.active:
            return False

        path = list(
            candidate[
                "path"
            ]
        )

        destination = str(
            candidate[
                "destination"
            ]
        )

        if (
            not path
            or path[0]
            != self.current_node
        ):
            return False

        self.mission_id = str(
            candidate[
                "mission_id"
            ]
        )

        self.route_revision = int(
            candidate[
                "route_revision"
            ]
        )

        self.destination_node = (
            destination
        )

        self.path_nodes = path
        self.path_index = 0

        if len(path) == 1:
            self.active = False
            self.state = "COMPLETED"

            self.publish_event(
                "MISSION_ALREADY_AT_"
                f"{destination}"
            )

            self.publish_route_plan(
                self.path_nodes,
                [],
            )

            self.publish_state()

            return True

        self.active = True
        self.state = "STARTING"

        self.mission_start_time = (
            time.time()
        )

        self.waiting_for_station_exit_turn_done = (
            False
        )

        self.station_exit_turn_done_time = (
            0.0
        )

        self.station_exit_raw_active = (
            False
        )

        self.station_exit_start_left_ticks = (
            None
        )

        self.station_exit_start_right_ticks = (
            None
        )

        actions = (
            self.build_route_actions(
                self.path_nodes
            )
        )

        self.publish_event(
            "MISSION_STARTED_EXACT_PATH_TO_"
            f"{destination}"
        )

        self.publish_route_plan(
            self.path_nodes,
            actions,
        )

        self.publish_state()

        self.get_logger().info(
            "Exact mission started: "
            f"mission={self.mission_id}, "
            f"revision={self.route_revision}, "
            f"path={self.path_nodes}"
        )

        if self.reset_feature_action_on_start:
            self.publish_feature_action_reset()

            time.sleep(
                0.05
            )

        if self.is_dead_end_station(
            self.current_node
        ):
            self.publish_ignore_station_until_junction(
                True
            )

            time.sleep(
                0.05
            )

            self.start_dead_end_station_exit()

        else:
            self.publish_ignore_station_until_junction(
                False
            )

            self.start_line_follow_to_next_node()

        return True

    # ==================================================
    # Active replacement at stopped junction
    # ==================================================

    def commit_active_replacement(
        self,
        candidate: Dict[
            str,
            Any,
        ],
    ) -> bool:
        if not self.active_replacement_safe():
            self.get_logger().warn(
                "Replacement became unsafe "
                "before start_path"
            )
            return False

        path = list(
            candidate[
                "path"
            ]
        )

        if (
            not path
            or path[0]
            != self.current_node
        ):
            return False

        if not self.replan_previous_node:
            return False

        self.mission_id = str(
            candidate[
                "mission_id"
            ]
        )

        self.route_revision = int(
            candidate[
                "route_revision"
            ]
        )

        self.destination_node = str(
            candidate[
                "destination"
            ]
        )

        self.path_nodes = path
        self.path_index = 0

        actions = (
            self.build_route_actions(
                self.path_nodes
            )
        )

        self.publish_route_plan(
            self.path_nodes,
            actions,
        )

        if len(path) < 2:
            self.complete_mission()
            return True

        next_node = path[1]

        command = (
            self.compute_junction_command(
                self.replan_previous_node,
                self.current_node,
                next_node,
            )
        )

        if command is None:
            self.cancel_mission(
                "MISSION_CANCELLED_"
                "REPLAN_NO_JUNCTION_COMMAND"
            )

            return False

        self.publish_junction_command(
            command
        )

        self.state = (
            "COMMAND_SENT_"
            f"{command.upper()}"
        )

        self.publish_event(
            "MISSION_REPLAN_COMMITTED_"
            f"REVISION_{self.route_revision}_"
            f"AT_{self.current_node}_"
            f"TO_{next_node}"
        )

        self.publish_state()

        self.get_logger().info(
            "Active route replacement "
            f"committed: revision="
            f"{self.route_revision}, "
            f"path={self.path_nodes}"
        )

        return True

    # ==================================================
    # Route-plan publication with IDs
    # ==================================================

    def publish_route_plan(
        self,
        path_nodes: List[str],
        actions: List[
            Dict[str, Any]
        ],
    ) -> None:
        msg = String()

        msg.data = json.dumps(
            {
                "mission_id":
                    self.mission_id,

                "route_revision":
                    self.route_revision,

                "from":
                    (
                        path_nodes[0]
                        if path_nodes
                        else self.current_node
                    ),

                "to":
                    (
                        path_nodes[-1]
                        if path_nodes
                        else self.destination_node
                    ),

                "path":
                    path_nodes,

                "actions":
                    actions,
            },
            separators=(",", ":"),
        )

        self.route_plan_pub.publish(
            msg
        )

    # ==================================================
    # Cleanup
    # ==================================================

    def complete_mission(
        self,
    ) -> None:
        super().complete_mission()

        self.path_candidate = None
        self.replan_previous_node = ""

    def cancel_mission(
        self,
        reason: str,
    ) -> None:
        super().cancel_mission(
            reason
        )

        self.path_candidate = None
        self.replan_previous_node = ""

    def set_current_node_callback(
        self,
        msg: String,
    ) -> None:
        super().set_current_node_callback(
            msg
        )

        if not self.active:
            self.path_candidate = None
            self.replan_previous_node = ""


def main(args=None) -> None:
    rclpy.init(
        args=args
    )

    node = MissionManagerNode()

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
