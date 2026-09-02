#!/usr/bin/env python3

from __future__ import annotations

import json
import time
import uuid

from typing import (
    Dict,
    List,
    Optional,
    Set,
    Tuple,
)

import rclpy

from std_msgs.msg import String

from .auto_resume_dock_safe_fleet_manager_node import (
    AutoResumeDockSafeFleetManagerNode,
)

from .fleet_manager_node import (
    FleetManagerNode,
    RobotState,
    edge_key,
)

from .traffic_planner import (
    Edge,
    MissionRequest,
    TrackPlanner,
)


Conflict = Tuple[
    str,
    str,
    str,
]


class PlannedRouteFleetManagerNode(
    AutoResumeDockSafeFleetManagerNode
):
    """
    VM-side planner with:

      - destination admission;
      - pending mission queue;
      - weighted shortest-path routing;
      - exact-path AGV loading;
      - mission and route revision validation;
      - rolling node/edge reservations;
      - atomic transit through non-holding nodes;
      - conditional idle-dock egress borrowing;
      - active rerouting at safe stopped junctions;
      - controlled retreat when no forward alternate exists.
    """

    def __init__(self) -> None:
        self.pending_requests: List[
            MissionRequest
        ] = []

        self.destination_claims: Dict[
            str,
            str,
        ] = {}

        super().__init__()

        self.declare_parameter(
            "exact_path_retry_sec",
            1.0,
        )

        self.declare_parameter(
            "controlled_retreat_delay_sec",
            2.0,
        )

        self.declare_parameter(
            "allow_idle_egress_borrowing",
            True,
        )

        self.exact_path_retry_sec = max(
            0.5,
            float(
                self.get_parameter(
                    "exact_path_retry_sec"
                ).value
            ),
        )

        self.controlled_retreat_delay_sec = max(
            0.0,
            float(
                self.config.get(
                    "controlled_retreat_delay_seconds",
                    self.get_parameter(
                        "controlled_retreat_delay_sec"
                    ).value,
                )
            ),
        )

        self.allow_idle_egress_borrowing = bool(
            self.config.get(
                "allow_idle_egress_borrowing",
                self.get_parameter(
                    "allow_idle_egress_borrowing"
                ).value,
            )
        )

        self.directed_connections = self.load_directed_connections()
        self.undirected_connections = set()

        if not self.directed_connections:
            self.undirected_connections = set(
                self.connections
            )

        for (
            node_a,
            node_b,
        ) in self.directed_connections:
            self.connections.add(
                edge_key(
                    node_a,
                    node_b,
                )
            )

        self.track_planner = TrackPlanner(
            self.nodes,
            list(
                self.undirected_connections
            ),
            directed_connections=self.directed_connections,
        )

        for (
            robot_name,
            robot,
        ) in self.robots.items():
            default_priority = int(
                self.config.get(
                    "robots",
                    {},
                ).get(
                    robot_name,
                    {},
                ).get(
                    "priority",
                    robot.priority,
                )
            )

            robot.priority = (
                default_priority
            )

            setattr(
                robot,
                "default_priority",
                default_priority,
            )

            setattr(
                robot,
                "route_revision",
                0,
            )

            setattr(
                robot,
                "candidate_path",
                [],
            )

            setattr(
                robot,
                "candidate_revision",
                0,
            )

            setattr(
                robot,
                "candidate_replace",
                False,
            )

            setattr(
                robot,
                "awaiting_path_ack",
                False,
            )

            setattr(
                robot,
                "awaiting_gate_ack",
                False,
            )

            setattr(
                robot,
                "path_request_time",
                0.0,
            )

            setattr(
                robot,
                "gate_route_time",
                0.0,
            )

            setattr(
                robot,
                "exact_protocol_ready",
                False,
            )

            setattr(
                robot,
                "blocked_since",
                0.0,
            )

            setattr(
                robot,
                "last_replan_attempt",
                0.0,
            )

        self.publish_event(
            "PLANNED_ROUTE_MANAGER_STARTED "
            "retreat_delay="
            f"{self.controlled_retreat_delay_sec} "
            "egress_borrowing="
            f"{self.allow_idle_egress_borrowing}"
        )

    # ==================================================
    # Config helpers
    # ==================================================

    def load_directed_connections(
        self,
    ) -> List[Tuple[str, str]]:
        raw_items = self.config.get(
            "directed_connections",
            [],
        )

        if not isinstance(
            raw_items,
            list,
        ):
            return []

        result: List[Tuple[str, str]] = []

        for item in raw_items:
            if (
                not isinstance(item, list)
                or len(item) != 2
            ):
                continue

            node_a = str(item[0]).strip().lower()
            node_b = str(item[1]).strip().lower()

            if node_a not in self.nodes or node_b not in self.nodes:
                continue

            result.append((node_a, node_b))

        return result

    # ==================================================
    # ROS interfaces
    # ==================================================

    def create_robot_interfaces(
        self,
        robot_name: str,
    ) -> None:
        super().create_robot_interfaces(
            robot_name
        )

        prefix = f"/{robot_name}"

        self.robot_publishers[
            robot_name
        ]["load_path"] = (
            self.create_publisher(
                String,
                f"{prefix}/mission/load_path",
                10,
            )
        )

        self.robot_publishers[
            robot_name
        ]["start_path"] = (
            self.create_publisher(
                String,
                f"{prefix}/mission/start_path",
                10,
            )
        )

        self.create_subscription(
            String,
            f"{prefix}/mission/path_ack",
            lambda msg, name=robot_name:
                self.path_ack_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/fleet/route_ack",
            lambda msg, name=robot_name:
                self.gate_route_ack_callback(
                    name,
                    msg,
                ),
            10,
        )

    # ==================================================
    # Destination queue
    # ==================================================

    def robot_has_pending_request(
        self,
        robot_name: str,
    ) -> bool:
        return any(
            request.robot == robot_name
            for request
            in self.pending_requests
        )

    def destination_available(
        self,
        robot_name: str,
        destination: str,
    ) -> bool:
        claim_owner = (
            self.destination_claims.get(
                destination
            )
        )

        dock_owner = (
            self.dock_locks.get(
                destination
            )
        )

        occupied_owner = (
            self.occupied_nodes.get(
                destination
            )
        )

        return (
            claim_owner in (
                None,
                robot_name,
            )
            and dock_owner in (
                None,
                robot_name,
            )
            and occupied_owner in (
                None,
                robot_name,
            )
        )

    def queue_request(
        self,
        request: MissionRequest,
        reason: str,
    ) -> None:
        if self.robot_has_pending_request(
            request.robot
        ):
            return

        robot = self.robots[
            request.robot
        ]

        robot.destination = (
            request.destination
        )

        robot.priority = (
            request.priority
        )

        robot.queued_at = (
            request.queued_at
        )

        robot.status = (
            "QUEUED_DESTINATION"
        )

        robot.waiting_reason = reason

        robot.blocked_resource = (
            request.destination
        )

        robot.blocked_by = (
            self.destination_claims.get(
                request.destination
            )
            or self.dock_locks.get(
                request.destination
            )
            or self.occupied_nodes.get(
                request.destination
            )
            or ""
        )

        self.pending_requests.append(
            request
        )

        self.publish_event(
            "MISSION_QUEUED "
            f"robot={request.robot} "
            "destination="
            f"{request.destination} "
            f"reason={reason}"
        )

    def dispatch_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            self.publish_event(
                "DISPATCH_REJECTED "
                "invalid_json"
            )
            return

        robot_name = str(
            payload.get(
                "robot",
                "",
            )
        ).strip()

        destination = str(
            payload.get(
                "destination",
                "",
            )
        ).strip().lower()

        try:
            priority = int(
                payload.get(
                    "priority",
                    10,
                )
            )
        except (
            TypeError,
            ValueError,
        ):
            priority = 10

        if robot_name not in self.robots:
            self.publish_event(
                "DISPATCH_REJECTED "
                f"unknown_robot={robot_name}"
            )
            return

        if destination not in self.nodes:
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "unknown_destination="
                f"{destination}"
            )
            return

        robot = self.robots[
            robot_name
        ]

        if (
            robot.mission_id
            or robot.pending_plan
            is not None
            or self.robot_has_pending_request(
                robot_name
            )
            or getattr(
                robot,
                "awaiting_path_ack",
                False,
            )
            or getattr(
                robot,
                "awaiting_gate_ack",
                False,
            )
        ):
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "busy"
            )
            return

        if robot.locked_after_stop:
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "reservations_locked_after_stop"
            )
            return

        if not robot.bridge_connected:
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "bridge_offline"
            )
            return

        if (
            not robot.safety_ok
            or robot.safety_hold
        ):
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "safety_not_ready"
            )
            return

        if not robot.current_node:
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} "
                "node_unknown"
            )
            return

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        if not mission_id:
            mission_id = (
                "mission_"
                + uuid.uuid4().hex[:10]
            )

        request = MissionRequest(
            robot=robot_name,
            destination=destination,
            priority=priority,
            mission_id=mission_id,
        )

        if (
            self.is_persistent_dock(
                destination
            )
            and not self.destination_available(
                robot_name,
                destination,
            )
        ):
            self.queue_request(
                request,
                "DESTINATION_OCCUPIED_OR_CLAIMED",
            )
            return

        self.start_request(
            request
        )

    def process_pending_requests(
        self,
    ) -> None:
        if not self.pending_requests:
            return

        ordered = sorted(
            self.pending_requests,
            key=lambda request: (
                -request.priority,
                request.queued_at,
                request.robot,
            ),
        )

        self.pending_requests = []

        for request in ordered:
            robot = self.robots[
                request.robot
            ]

            if (
                robot.mission_id
                or robot.mission_active
            ):
                continue

            if (
                self.is_persistent_dock(
                    request.destination
                )
                and not self.destination_available(
                    request.robot,
                    request.destination,
                )
            ):
                self.pending_requests.append(
                    request
                )
                continue

            path = self.plan_for_robot(
                request.robot,
                request.destination,
            )

            if path is None:
                robot.status = (
                    "WAITING_FOR_ROUTE"
                )

                robot.waiting_reason = (
                    "NO_SAFE_ROUTE"
                )

                self.pending_requests.append(
                    request
                )

                continue

            self.start_request(
                request,
                path,
            )

    # ==================================================
    # Planning and egress borrowing
    # ==================================================

    def owner_has_departure_intent(
        self,
        owner_name: str,
    ) -> bool:
        owner = self.robots.get(
            owner_name
        )

        if owner is None:
            return True

        return bool(
            owner.mission_id
            or owner.mission_active
            or self.robot_has_pending_request(
                owner_name
            )
            or getattr(
                owner,
                "awaiting_path_ack",
                False,
            )
            or getattr(
                owner,
                "awaiting_gate_ack",
                False,
            )
        )

    def egress_borrowable(
        self,
        robot: RobotState,
        node: str,
        owner_name: str,
    ) -> bool:
        if not self.allow_idle_egress_borrowing:
            return False

        if owner_name == robot.name:
            return True

        owner = self.robots.get(
            owner_name
        )

        if owner is None:
            return False

        docked_at = getattr(
            owner,
            "docked_at",
            "",
        )

        if not docked_at:
            return False

        if self.owner_has_departure_intent(
            owner_name
        ):
            return False

        if robot.destination in (
            node,
            docked_at,
        ):
            return False

        return True

    def planning_blocks(
        self,
        robot: RobotState,
        forbid_first_step: str = "",
    ) -> Tuple[
        Set[str],
        Set[Edge],
    ]:
        blocked_nodes: Set[
            str
        ] = set()

        blocked_edges: Set[
            Edge
        ] = set()

        for other in (
            self.robots.values()
        ):
            if other.name == robot.name:
                continue

            if other.current_node:
                blocked_nodes.add(
                    other.current_node
                )

            blocked_nodes.update(
                other.reserved_nodes
            )

            blocked_edges.update(
                other.reserved_edges
            )

        for (
            node,
            owner,
        ) in self.destination_claims.items():
            if owner != robot.name:
                blocked_nodes.add(
                    node
                )

        for (
            node,
            owner,
        ) in self.dock_locks.items():
            if owner != robot.name:
                blocked_nodes.add(
                    node
                )

        for (
            node,
            owner,
        ) in self.egress_node_locks.items():
            if (
                owner != robot.name
                and not self.egress_borrowable(
                    robot,
                    node,
                    owner,
                )
            ):
                blocked_nodes.add(
                    node
                )

        for (
            edge,
            owner,
        ) in self.egress_edge_locks.items():
            if owner != robot.name:
                blocked_edges.add(
                    edge
                )

        blocked_nodes.discard(
            robot.current_node
        )

        if (
            self.destination_claims.get(
                robot.destination
            )
            == robot.name
        ):
            blocked_nodes.discard(
                robot.destination
            )

        if forbid_first_step:
            blocked_edges.add(
                edge_key(
                    robot.current_node,
                    forbid_first_step,
                )
            )

        return (
            blocked_nodes,
            blocked_edges,
        )

    def plan_for_robot(
        self,
        robot_name: str,
        destination: str,
        forbid_first_step: str = "",
    ) -> Optional[List[str]]:
        robot = self.robots[
            robot_name
        ]

        original_destination = (
            robot.destination
        )

        robot.destination = destination

        (
            blocked_nodes,
            blocked_edges,
        ) = self.planning_blocks(
            robot,
            forbid_first_step=(
                forbid_first_step
            ),
        )

        path = (
            self.track_planner.shortest_path(
                robot.current_node,
                destination,
                blocked_nodes=(
                    blocked_nodes
                ),
                blocked_edges=(
                    blocked_edges
                ),
            )
        )

        robot.destination = (
            original_destination
        )

        return path

    # ==================================================
    # Mission setup and exact-path transaction
    # ==================================================

    def start_request(
        self,
        request: MissionRequest,
        path: Optional[List[str]] = None,
    ) -> None:
        robot = self.robots[
            request.robot
        ]

        route = (
            path
            or self.plan_for_robot(
                request.robot,
                request.destination,
            )
        )

        if route is None:
            self.queue_request(
                request,
                "NO_SAFE_ROUTE",
            )
            return

        if len(route) == 1:
            robot.destination = ""
            robot.status = "IDLE"
            robot.waiting_reason = ""

            self.publish_event(
                "MISSION_ALREADY_AT_DESTINATION "
                f"robot={robot.name} "
                f"node={request.destination}"
            )
            return

        if self.is_persistent_dock(
            request.destination
        ):
            self.destination_claims[
                request.destination
            ] = robot.name

        robot.mission_id = (
            request.mission_id
        )

        robot.destination = (
            request.destination
        )

        robot.full_path = list(
            route
        )

        robot.current_index = 0
        robot.released_until_index = 0

        robot.started = False
        robot.ever_active = False

        robot.priority = (
            request.priority
        )

        robot.queued_at = (
            request.queued_at
        )

        robot.status = (
            "WAITING_INITIAL_RELEASE"
        )

        robot.waiting_reason = (
            "WAITING_INITIAL_RELEASE"
        )

        robot.blocked_resource = ""
        robot.blocked_by = ""

        revision = (
            int(
                getattr(
                    robot,
                    "route_revision",
                    0,
                )
            )
            + 1
        )

        setattr(
            robot,
            "route_revision",
            revision,
        )

        setattr(
            robot,
            "candidate_path",
            list(route),
        )

        setattr(
            robot,
            "candidate_revision",
            revision,
        )

        setattr(
            robot,
            "candidate_replace",
            False,
        )

        setattr(
            robot,
            "exact_protocol_ready",
            False,
        )

        self.try_extend_robot(
            robot
        )

        self.send_path_load(
            robot
        )

        self.publish_event(
            "MISSION_PLANNED "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            f"revision={revision} "
            f"path={route}"
        )

    def send_path_load(
        self,
        robot: RobotState,
    ) -> None:
        path = list(
            getattr(
                robot,
                "candidate_path",
                [],
            )
        )

        revision = int(
            getattr(
                robot,
                "candidate_revision",
                0,
            )
        )

        if (
            not robot.mission_id
            or not path
            or revision <= 0
        ):
            return

        payload = {
            "mission_id":
                robot.mission_id,

            "route_revision":
                revision,

            "destination":
                robot.destination,

            "path":
                path,

            "replace_active_route":
                bool(
                    getattr(
                        robot,
                        "candidate_replace",
                        False,
                    )
                ),
        }

        self.publish_string(
            self.robot_publishers[
                robot.name
            ]["load_path"],
            json.dumps(
                payload,
                separators=(",", ":"),
            ),
        )

        setattr(
            robot,
            "awaiting_path_ack",
            True,
        )

        setattr(
            robot,
            "path_request_time",
            time.monotonic(),
        )

        if payload[
            "replace_active_route"
        ]:
            robot.status = (
                "WAITING_REPLAN_ACK"
            )
        else:
            robot.status = (
                "WAITING_PATH_ACK"
            )

        self.publish_event(
            "PATH_LOAD_SENT "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            f"revision={revision} "
            "replace="
            f"{payload['replace_active_route']}"
        )

    def path_ack_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            return

        robot = self.robots[
            robot_name
        ]

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        try:
            revision = int(
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

        candidate_revision = int(
            getattr(
                robot,
                "candidate_revision",
                0,
            )
        )

        candidate_path = list(
            getattr(
                robot,
                "candidate_path",
                [],
            )
        )

        if (
            mission_id
            != robot.mission_id
            or revision
            != candidate_revision
        ):
            return

        setattr(
            robot,
            "awaiting_path_ack",
            False,
        )

        if not bool(
            payload.get(
                "accepted",
                False,
            )
        ):
            reason = str(
                payload.get(
                    "reason",
                    "rejected",
                )
            )

            replacing = bool(
                getattr(
                    robot,
                    "candidate_replace",
                    False,
                )
            )

            self.publish_event(
                "PATH_LOAD_REJECTED "
                f"robot={robot.name} "
                f"revision={revision} "
                f"reason={reason}"
            )

            if replacing:
                self.clear_candidate(
                    robot
                )

                robot.status = (
                    "WAITING_FOR_RELEASE"
                )
                return

            self.abort_initial_transaction(
                robot
            )
            return

        ack_path = [
            str(node)
            .strip()
            .lower()
            for node
            in payload.get(
                "path",
                [],
            )
        ]

        if ack_path != candidate_path:
            self.publish_event(
                "PATH_ACK_REJECTED "
                f"robot={robot.name} "
                "path_mismatch"
            )
            return

        if getattr(
            robot,
            "candidate_replace",
            False,
        ):
            if not self.commit_candidate_route(
                robot
            ):
                self.publish_event(
                    "REPLAN_COMMIT_REJECTED "
                    f"robot={robot.name} "
                    "resources_changed"
                )

                self.clear_candidate(
                    robot
                )

                robot.status = (
                    "WAITING_FOR_RELEASE"
                )
                return

        setattr(
            robot,
            "awaiting_gate_ack",
            True,
        )

        setattr(
            robot,
            "gate_route_time",
            time.monotonic(),
        )

        setattr(
            robot,
            "exact_protocol_ready",
            False,
        )

        self.publish_fleet_route(
            robot
        )

    def publish_fleet_route(
        self,
        robot: RobotState,
    ) -> None:
        payload = {
            "mission_id":
                robot.mission_id,

            "route_revision":
                int(
                    getattr(
                        robot,
                        "route_revision",
                        0,
                    )
                ),

            "destination":
                robot.destination,

            "path":
                robot.full_path,

            "released_until_index":
                robot.released_until_index,
        }

        self.publish_string(
            self.robot_publishers[
                robot.name
            ]["route"],
            json.dumps(
                payload,
                separators=(",", ":"),
            ),
        )

        self.publish_event(
            "FLEET_ROUTE_SENT "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            "revision="
            f"{payload['route_revision']}"
        )

    def gate_route_ack_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            return

        robot = self.robots[
            robot_name
        ]

        mission_id = str(
            payload.get(
                "mission_id",
                "",
            )
        ).strip()

        try:
            revision = int(
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
            != robot.mission_id
            or revision
            != int(
                getattr(
                    robot,
                    "route_revision",
                    0,
                )
            )
        ):
            return

        ack_path = [
            str(node)
            .strip()
            .lower()
            for node
            in payload.get(
                "path",
                [],
            )
        ]

        if ack_path != robot.full_path:
            return

        setattr(
            robot,
            "awaiting_gate_ack",
            False,
        )

        setattr(
            robot,
            "exact_protocol_ready",
            True,
        )

        self.publish_release(
            robot
        )

        start_payload = {
            "mission_id":
                robot.mission_id,

            "route_revision":
                revision,
        }

        self.publish_string(
            self.robot_publishers[
                robot.name
            ]["start_path"],
            json.dumps(
                start_payload,
                separators=(",", ":"),
            ),
        )

        if not robot.started:
            robot.started = True

        robot.status = (
            "START_COMMAND_SENT"
        )

        robot.waiting_reason = ""

        self.clear_candidate(
            robot
        )

        self.publish_event(
            "EXACT_PATH_START_SENT "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            f"revision={revision}"
        )

    # ==================================================
    # Reservation behavior
    # ==================================================

    def next_atomic_segment(
        self,
        robot: RobotState,
    ) -> Optional[List[int]]:
        start_index = (
            robot.released_until_index
        )

        if (
            start_index
            >= len(robot.full_path) - 1
        ):
            return None

        segment: List[int] = []

        index = (
            start_index + 1
        )

        while index < len(
            robot.full_path
        ):
            segment.append(
                index
            )

            node = robot.full_path[
                index
            ]

            if node == robot.destination:
                break

            if self.node_holding_allowed(
                node
            ):
                owner = (
                    self.egress_node_locks.get(
                        node
                    )
                )

                if (
                    owner
                    and owner != robot.name
                    and self.egress_borrowable(
                        robot,
                        node,
                        owner,
                    )
                ):
                    index += 1
                    continue

                break

            index += 1

        return segment

    def resource_conflict(
        self,
        robot: RobotState,
        from_node: str,
        to_node: str,
    ) -> Optional[Conflict]:
        dock_owner = (
            self.dock_locks.get(
                to_node
            )
        )

        if dock_owner not in (
            None,
            robot.name,
        ):
            return (
                "DOCK_OCCUPIED",
                to_node,
                str(dock_owner),
            )

        egress_owner = (
            self.egress_node_locks.get(
                to_node
            )
        )

        if (
            egress_owner not in (
                None,
                robot.name,
            )
            and not self.egress_borrowable(
                robot,
                to_node,
                str(egress_owner),
            )
        ):
            return (
                "DOCK_EGRESS_PROTECTED",
                to_node,
                str(egress_owner),
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
                str(edge_owner),
            )

        return (
            FleetManagerNode.resource_conflict(
                self,
                robot,
                from_node,
                to_node,
            )
        )

    def publish_release(
        self,
        robot: RobotState,
    ) -> None:
        if not getattr(
            robot,
            "exact_protocol_ready",
            False,
        ):
            return

        payload = {
            "mission_id":
                robot.mission_id,

            "route_revision":
                int(
                    getattr(
                        robot,
                        "route_revision",
                        0,
                    )
                ),

            "released_until_index":
                robot.released_until_index,
        }

        self.publish_string(
            self.robot_publishers[
                robot.name
            ]["release"],
            json.dumps(
                payload,
                separators=(",", ":"),
            ),
        )

        self.publish_event(
            "ROUTE_RELEASE_EXTENDED "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            "revision="
            f"{payload['route_revision']} "
            "index="
            f"{robot.released_until_index} "
            "node="
            f"{robot.full_path[robot.released_until_index]}"
        )

    def try_start_robot(
        self,
        robot: RobotState,
    ) -> None:
        # Exact-path start is sent only
        # after the mission-manager ACK
        # and fleet-gate ACK.
        del robot

    # ==================================================
    # Live rerouting
    # ==================================================

    def next_release_conflict(
        self,
        robot: RobotState,
    ) -> Optional[Conflict]:
        segment = self.next_atomic_segment(
            robot
        )

        if not segment:
            return None

        for index in segment:
            conflict = self.resource_conflict(
                robot,
                robot.full_path[
                    index - 1
                ],
                robot.full_path[
                    index
                ],
            )

            if conflict is not None:
                return conflict

        return None

    def safe_for_active_replan(
        self,
        robot: RobotState,
    ) -> bool:
        if (
            not robot.mission_id
            or not robot.mission_active
        ):
            return False

        if not self.node_holding_allowed(
            robot.current_node
        ):
            return False

        if self.is_persistent_dock(
            robot.current_node
        ):
            return False

        if (
            getattr(
                robot,
                "awaiting_path_ack",
                False,
            )
            or getattr(
                robot,
                "awaiting_gate_ack",
                False,
            )
        ):
            return False

        gate_waiting = bool(
            robot.gate_state.get(
                "waiting_for_release",
                False,
            )
        )

        state_safe = str(
            robot.mission_state
        ).startswith(
            "COMMAND_SENT_"
        )

        return (
            gate_waiting
            and state_safe
        )

    def stage_replacement(
        self,
        robot: RobotState,
        new_path: List[str],
        mode: str,
    ) -> bool:
        remaining = robot.full_path[
            robot.current_index:
        ]

        if (
            not new_path
            or new_path == remaining
        ):
            return False

        revision = (
            int(
                getattr(
                    robot,
                    "route_revision",
                    0,
                )
            )
            + 1
        )

        setattr(
            robot,
            "candidate_path",
            list(new_path),
        )

        setattr(
            robot,
            "candidate_revision",
            revision,
        )

        setattr(
            robot,
            "candidate_replace",
            True,
        )

        robot.status = mode

        self.send_path_load(
            robot
        )

        self.publish_event(
            f"{mode} "
            f"robot={robot.name} "
            f"old_path={remaining} "
            f"new_path={new_path}"
        )

        return True

    def candidate_initial_segment(
        self,
        robot: RobotState,
        path: List[str],
    ) -> List[int]:
        if len(path) < 2:
            return []

        result: List[int] = []

        index = 1

        while index < len(path):
            result.append(
                index
            )

            node = path[
                index
            ]

            if node == robot.destination:
                break

            if self.node_holding_allowed(
                node
            ):
                owner = (
                    self.egress_node_locks.get(
                        node
                    )
                )

                if (
                    owner
                    and owner != robot.name
                    and self.egress_borrowable(
                        robot,
                        node,
                        owner,
                    )
                ):
                    index += 1
                    continue

                break

            index += 1

        return result

    def commit_candidate_route(
        self,
        robot: RobotState,
    ) -> bool:
        path = list(
            getattr(
                robot,
                "candidate_path",
                [],
            )
        )

        if (
            not path
            or path[0]
            != robot.current_node
        ):
            return False

        for index in (
            self.candidate_initial_segment(
                robot,
                path,
            )
        ):
            conflict = (
                self.resource_conflict(
                    robot,
                    path[index - 1],
                    path[index],
                )
            )

            if conflict is not None:
                return False

        self.release_all_reservations(
            robot
        )

        robot.full_path = path
        robot.current_index = 0
        robot.released_until_index = 0

        robot.waiting_reason = ""
        robot.blocked_resource = ""
        robot.blocked_by = ""

        setattr(
            robot,
            "route_revision",
            int(
                getattr(
                    robot,
                    "candidate_revision",
                    0,
                )
            ),
        )

        setattr(
            robot,
            "exact_protocol_ready",
            False,
        )

        setattr(
            robot,
            "blocked_since",
            0.0,
        )

        self.try_extend_robot(
            robot
        )

        return (
            robot.released_until_index
            > 0
        )

    def try_reroute_blocked_robot(
        self,
        robot: RobotState,
    ) -> bool:
        if not self.safe_for_active_replan(
            robot
        ):
            return False

        conflict = (
            self.next_release_conflict(
                robot
            )
        )

        if conflict is None:
            return False

        (
            reason,
            resource,
            owner,
        ) = conflict

        now = time.monotonic()

        if (
            robot.blocked_by != owner
            or robot.blocked_resource
            != resource
            or float(
                getattr(
                    robot,
                    "blocked_since",
                    0.0,
                )
            ) <= 0.0
        ):
            setattr(
                robot,
                "blocked_since",
                now,
            )

        robot.waiting_reason = reason
        robot.blocked_resource = resource
        robot.blocked_by = owner

        last_attempt = float(
            getattr(
                robot,
                "last_replan_attempt",
                0.0,
            )
        )

        if now - last_attempt < 0.5:
            return False

        setattr(
            robot,
            "last_replan_attempt",
            now,
        )

        alternate = (
            self.plan_for_robot(
                robot.name,
                robot.destination,
                forbid_first_step=(
                    robot.previous_node
                ),
            )
        )

        if (
            alternate
            and alternate
            != robot.full_path[
                robot.current_index:
            ]
        ):
            return (
                self.stage_replacement(
                    robot,
                    alternate,
                    "ROUTE_REPLAN_REQUESTED",
                )
            )

        blocked_since = float(
            getattr(
                robot,
                "blocked_since",
                0.0,
            )
        )

        if (
            now - blocked_since
            < self.controlled_retreat_delay_sec
        ):
            return False

        retreat = self.plan_for_robot(
            robot.name,
            robot.destination,
        )

        if (
            not retreat
            or retreat
            == robot.full_path[
                robot.current_index:
            ]
        ):
            return False

        mode = (
            "ROUTE_REPLAN_REQUESTED"
        )

        if (
            robot.previous_node
            and len(retreat) > 1
            and retreat[1]
            == robot.previous_node
        ):
            mode = (
                "CONTROLLED_RETREAT_REQUESTED"
            )

        return self.stage_replacement(
            robot,
            retreat,
            mode,
        )

    # ==================================================
    # Cleanup and timers
    # ==================================================

    def clear_candidate(
        self,
        robot: RobotState,
    ) -> None:
        setattr(
            robot,
            "candidate_path",
            [],
        )

        setattr(
            robot,
            "candidate_revision",
            0,
        )

        setattr(
            robot,
            "candidate_replace",
            False,
        )

        setattr(
            robot,
            "awaiting_path_ack",
            False,
        )

        setattr(
            robot,
            "awaiting_gate_ack",
            False,
        )

        setattr(
            robot,
            "path_request_time",
            0.0,
        )

        setattr(
            robot,
            "gate_route_time",
            0.0,
        )

    def reset_exact_fields(
        self,
        robot: RobotState,
    ) -> None:
        self.clear_candidate(
            robot
        )

        setattr(
            robot,
            "exact_protocol_ready",
            False,
        )

        setattr(
            robot,
            "blocked_since",
            0.0,
        )

        setattr(
            robot,
            "last_replan_attempt",
            0.0,
        )

    def abort_initial_transaction(
        self,
        robot: RobotState,
    ) -> None:
        destination = (
            robot.destination
        )

        if (
            self.destination_claims.get(
                destination
            )
            == robot.name
        ):
            self.destination_claims.pop(
                destination,
                None,
            )

        self.release_all_reservations(
            robot
        )

        self.clear_mission_fields(
            robot
        )

        robot.status = "IDLE"

    def cancel_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is not None:
            robot_name = str(
                payload.get(
                    "robot",
                    "",
                )
            ).strip()
        else:
            robot_name = (
                msg.data.strip()
            )

        if robot_name in self.robots:
            pending = [
                request
                for request
                in self.pending_requests
                if request.robot
                != robot_name
            ]

            if (
                len(pending)
                != len(
                    self.pending_requests
                )
            ):
                self.pending_requests = (
                    pending
                )

                robot = self.robots[
                    robot_name
                ]

                robot.destination = ""
                robot.waiting_reason = ""
                robot.blocked_resource = ""
                robot.blocked_by = ""
                robot.status = "IDLE"

                robot.priority = int(
                    getattr(
                        robot,
                        "default_priority",
                        10,
                    )
                )

                self.publish_event(
                    "QUEUED_MISSION_CANCELLED "
                    f"robot={robot_name}"
                )
                return

        super().cancel_callback(
            msg
        )

    def clear_completed_mission(
        self,
        robot: RobotState,
    ) -> None:
        destination = (
            robot.destination
        )

        if (
            self.destination_claims.get(
                destination
            )
            == robot.name
        ):
            self.destination_claims.pop(
                destination,
                None,
            )

        super().clear_completed_mission(
            robot
        )

        self.reset_exact_fields(
            robot
        )

        self.process_pending_requests()

    def clear_mission_fields(
        self,
        robot: RobotState,
    ) -> None:
        super().clear_mission_fields(
            robot
        )

        robot.priority = int(
            getattr(
                robot,
                "default_priority",
                10,
            )
        )

        self.reset_exact_fields(
            robot
        )

    def clear_reservations_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is not None:
            robot_name = str(
                payload.get(
                    "robot",
                    "",
                )
            ).strip()

            robot = self.robots.get(
                robot_name
            )

            if robot is not None:
                destination = (
                    robot.destination
                )

                if (
                    self.destination_claims.get(
                        destination
                    )
                    == robot_name
                ):
                    self.destination_claims.pop(
                        destination,
                        None,
                    )

        super().clear_reservations_callback(
            msg
        )

        self.process_pending_requests()

    def try_progress_all(
        self,
    ) -> None:
        self.process_pending_requests()

        ordered = (
            self.sorted_progress_order()
        )

        for robot in ordered:
            self.try_extend_robot(
                robot
            )

        for robot in ordered:
            self.try_reroute_blocked_robot(
                robot
            )

        self.detect_deadlock()

    def retry_exact_transactions(
        self,
    ) -> None:
        now = time.monotonic()

        for robot in (
            self.robots.values()
        ):
            if getattr(
                robot,
                "awaiting_path_ack",
                False,
            ):
                last_request = float(
                    getattr(
                        robot,
                        "path_request_time",
                        0.0,
                    )
                )

                if (
                    now - last_request
                    >= self.exact_path_retry_sec
                ):
                    self.send_path_load(
                        robot
                    )

            elif getattr(
                robot,
                "awaiting_gate_ack",
                False,
            ):
                last_request = float(
                    getattr(
                        robot,
                        "gate_route_time",
                        0.0,
                    )
                )

                if (
                    now - last_request
                    >= self.exact_path_retry_sec
                ):
                    setattr(
                        robot,
                        "gate_route_time",
                        now,
                    )

                    self.publish_fleet_route(
                        robot
                    )

    def timer_callback(
        self,
    ) -> None:
        self.retry_exact_transactions()

        super().timer_callback()


def main(args=None) -> None:
    rclpy.init(
        args=args
    )

    node = (
        PlannedRouteFleetManagerNode()
    )

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
