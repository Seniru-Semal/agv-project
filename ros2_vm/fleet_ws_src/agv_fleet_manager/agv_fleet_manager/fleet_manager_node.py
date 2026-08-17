#!/usr/bin/env python3

import json
import os
import time
import uuid

from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Set,
    Tuple,
)

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import Bool, String


Edge = Tuple[str, str]


def edge_key(
    node_a: str,
    node_b: str,
) -> Edge:
    values = sorted(
        (
            node_a,
            node_b,
        )
    )

    return (
        values[0],
        values[1],
    )


@dataclass
class RobotState:
    name: str
    initial_node: str

    current_node: str = ""
    previous_node: str = ""

    bridge_connected: bool = False
    last_status_time: float = 0.0

    safety_ok: bool = False
    safety_hold: bool = False
    resume_allowed: bool = False
    obstacle_state: str = "UNKNOWN"

    mission_active: bool = False
    mission_state: str = "IDLE"

    gate_state: Dict[str, Any] = field(
        default_factory=dict
    )

    pending_plan: Optional[
        Dict[str, Any]
    ] = None

    mission_id: str = ""
    destination: str = ""

    full_path: List[str] = field(
        default_factory=list
    )

    current_index: int = 0
    released_until_index: int = 0

    started: bool = False
    ever_active: bool = False

    status: str = "IDLE"
    priority: int = 10
    queued_at: float = 0.0

    waiting_reason: str = ""
    blocked_resource: str = ""
    blocked_by: str = ""

    locked_after_stop: bool = False

    reserved_nodes: Set[str] = field(
        default_factory=set
    )

    reserved_edges: Set[Edge] = field(
        default_factory=set
    )


class FleetManagerNode(Node):

    def __init__(self) -> None:
        super().__init__(
            "fleet_manager_node"
        )

        default_config = os.path.join(
            get_package_share_directory(
                "agv_fleet_manager"
            ),
            "config",
            "fleet_config.json",
        )

        self.declare_parameter(
            "config_file",
            default_config,
        )

        self.declare_parameter(
            "status_timeout_sec",
            3.0,
        )

        self.config_file = os.path.expanduser(
            str(
                self.get_parameter(
                    "config_file"
                ).value
            )
        )

        self.status_timeout_sec = float(
            self.get_parameter(
                "status_timeout_sec"
            ).value
        )

        self.config = self.load_config(
            self.config_file
        )

        self.nodes: Dict[
            str,
            Dict[str, Any],
        ] = self.config["nodes"]

        self.connections: Set[Edge] = {
            edge_key(
                str(item[0]),
                str(item[1]),
            )
            for item in self.config[
                "connections"
            ]
        }

        self.lookahead_edges = max(
            1,
            int(
                self.config.get(
                    "lookahead_edges",
                    2,
                )
            ),
        )

        self.robots: Dict[
            str,
            RobotState,
        ] = {}

        for (
            robot_name,
            robot_info,
        ) in self.config[
            "robots"
        ].items():
            initial_node = str(
                robot_info[
                    "initial_node"
                ]
            ).strip().lower()

            self.robots[
                robot_name
            ] = RobotState(
                name=robot_name,
                initial_node=initial_node,
                current_node=initial_node,
            )

        self.occupied_nodes: Dict[
            str,
            str,
        ] = {
            robot.current_node:
                robot.name
            for robot in self.robots.values()
            if robot.current_node
        }

        self.node_reservations: Dict[
            str,
            str,
        ] = {}

        self.edge_reservations: Dict[
            Edge,
            str,
        ] = {}

        self.last_deadlock_signature = ""

        self.event_pub = self.create_publisher(
            String,
            "/fleet/event",
            20,
        )

        self.state_pub = self.create_publisher(
            String,
            "/fleet/state",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/fleet/status",
            10,
        )

        self.reservations_pub = (
            self.create_publisher(
                String,
                "/fleet/reservations",
                10,
            )
        )

        self.queue_pub = self.create_publisher(
            String,
            "/fleet/queue",
            10,
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "/fleet/ready",
            10,
        )

        self.create_subscription(
            String,
            "/fleet/dispatch",
            self.dispatch_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/cancel",
            self.cancel_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/resume",
            self.resume_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/estop",
            self.estop_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/reset",
            self.reset_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/clear_reservations",
            self.clear_reservations_callback,
            20,
        )

        self.create_subscription(
            String,
            "/fleet/set_current_node",
            self.set_current_node_callback,
            20,
        )

        self.robot_publishers: Dict[
            str,
            Dict[str, Any],
        ] = {}

        for robot_name in self.robots:
            self.create_robot_interfaces(
                robot_name
            )

        self.create_timer(
            0.2,
            self.timer_callback,
        )

        self.publish_event(
            "FLEET_MANAGER_STARTED "
            f"lookahead_edges="
            f"{self.lookahead_edges}"
        )

        self.get_logger().info(
            "Fleet config: "
            f"{self.config_file}"
        )

    @staticmethod
    def load_config(
        path: str,
    ) -> Dict[str, Any]:
        file_path = Path(path)

        if not file_path.exists():
            raise FileNotFoundError(
                "Fleet config not found: "
                f"{path}"
            )

        with file_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            data = json.load(stream)

        for required_key in (
            "robots",
            "nodes",
            "connections",
        ):
            if required_key not in data:
                raise ValueError(
                    "Fleet config is missing "
                    f"'{required_key}'"
                )

        return data

    def create_robot_interfaces(
        self,
        robot_name: str,
    ) -> None:
        prefix = f"/{robot_name}"

        publishers = {
            "plan":
                self.create_publisher(
                    String,
                    f"{prefix}/mission/plan",
                    10,
                ),

            "start":
                self.create_publisher(
                    String,
                    f"{prefix}/mission/start",
                    10,
                ),

            "cancel":
                self.create_publisher(
                    Bool,
                    f"{prefix}/mission/cancel",
                    10,
                ),

            "route":
                self.create_publisher(
                    String,
                    f"{prefix}/fleet/route",
                    10,
                ),

            "release":
                self.create_publisher(
                    String,
                    f"{prefix}/fleet/release",
                    10,
                ),

            "resume":
                self.create_publisher(
                    Bool,
                    (
                        f"{prefix}/safety_hold/"
                        "resume_request"
                    ),
                    10,
                ),

            "estop":
                self.create_publisher(
                    Bool,
                    f"{prefix}/cmd/estop",
                    10,
                ),

            "reset":
                self.create_publisher(
                    Bool,
                    f"{prefix}/cmd/reset",
                    10,
                ),

            "set_current":
                self.create_publisher(
                    String,
                    (
                        f"{prefix}/mission/"
                        "set_current_node"
                    ),
                    10,
                ),
        }

        self.robot_publishers[
            robot_name
        ] = publishers

        self.create_subscription(
            String,
            f"{prefix}/mission/current_node",
            lambda msg, name=robot_name:
                self.current_node_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/mission/route_plan",
            lambda msg, name=robot_name:
                self.route_plan_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/mission/active",
            lambda msg, name=robot_name:
                self.mission_active_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/mission/state",
            lambda msg, name=robot_name:
                self.mission_state_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/bridge_connected",
            lambda msg, name=robot_name:
                self.bridge_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/status_raw",
            lambda msg, name=robot_name:
                self.status_raw_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            Bool,
            f"{prefix}/safety/ok",
            lambda msg, name=robot_name:
                self.safety_ok_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            Bool,
            (
                f"{prefix}/safety_hold/"
                "active"
            ),
            lambda msg, name=robot_name:
                self.safety_hold_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            Bool,
            (
                f"{prefix}/safety_hold/"
                "resume_allowed"
            ),
            lambda msg, name=robot_name:
                self.resume_allowed_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/obstacle/state",
            lambda msg, name=robot_name:
                self.obstacle_callback(
                    name,
                    msg,
                ),
            10,
        )

        self.create_subscription(
            String,
            f"{prefix}/fleet/gate_state",
            lambda msg, name=robot_name:
                self.gate_state_callback(
                    name,
                    msg,
                ),
            10,
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

    def publish_event(
        self,
        text: str,
    ) -> None:
        msg = String()
        msg.data = text

        self.event_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def publish_string(
        publisher: Any,
        text: str,
    ) -> None:
        msg = String()
        msg.data = text

        publisher.publish(msg)

    @staticmethod
    def publish_bool(
        publisher: Any,
        value: bool,
    ) -> None:
        msg = Bool()
        msg.data = bool(value)

        publisher.publish(msg)

    def dispatch_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            self.publish_event(
                "DISPATCH_REJECTED invalid_json"
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
            robot.pending_plan is not None
            or robot.mission_id
        ):
            self.publish_event(
                "DISPATCH_REJECTED "
                f"robot={robot_name} busy"
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

        robot.pending_plan = {
            "mission_id":
                mission_id,

            "destination":
                destination,

            "priority":
                priority,

            "requested_at":
                time.time(),
        }

        robot.status = "PLANNING"

        robot.waiting_reason = (
            "WAITING_FOR_ROUTE_PLAN"
        )

        self.publish_string(
            self.robot_publishers[
                robot_name
            ]["plan"],
            destination,
        )

        self.publish_event(
            "PLAN_REQUESTED "
            f"robot={robot_name} "
            f"mission={mission_id} "
            f"from={robot.current_node} "
            f"to={destination}"
        )

    def route_plan_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        if robot.pending_plan is None:
            return

        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            self.publish_event(
                "PLAN_REJECTED "
                f"robot={robot_name} "
                "invalid_route_json"
            )

            robot.pending_plan = None
            robot.status = "IDLE"
            return

        raw_path = payload.get("path")

        if (
            not isinstance(
                raw_path,
                list,
            )
            or not raw_path
        ):
            self.publish_event(
                "PLAN_REJECTED "
                f"robot={robot_name} "
                "empty_path"
            )

            robot.pending_plan = None
            robot.status = "IDLE"
            return

        path = [
            str(node).strip().lower()
            for node in raw_path
        ]

        pending = robot.pending_plan

        destination = str(
            pending["destination"]
        )

        if path[0] != robot.current_node:
            self.publish_event(
                "PLAN_REJECTED "
                f"robot={robot_name} "
                "expected_start="
                f"{robot.current_node} "
                "received_start="
                f"{path[0]}"
            )

            robot.pending_plan = None
            robot.status = "IDLE"
            return

        if path[-1] != destination:
            return

        if not self.validate_path(path):
            self.publish_event(
                "PLAN_REJECTED "
                f"robot={robot_name} "
                "invalid_graph_path"
            )

            robot.pending_plan = None
            robot.status = "IDLE"
            return

        robot.mission_id = str(
            pending["mission_id"]
        )

        robot.destination = destination
        robot.full_path = path

        robot.current_index = 0
        robot.released_until_index = 0

        robot.started = False
        robot.ever_active = False

        robot.status = (
            "WAITING_INITIAL_RELEASE"
        )

        robot.priority = int(
            pending["priority"]
        )

        robot.queued_at = time.time()

        robot.waiting_reason = (
            "WAITING_INITIAL_RELEASE"
        )

        robot.blocked_resource = ""
        robot.blocked_by = ""

        robot.pending_plan = None

        if len(path) == 1:
            self.publish_event(
                "MISSION_ALREADY_AT_DESTINATION "
                f"robot={robot_name} "
                f"node={destination}"
            )

            self.clear_completed_mission(
                robot
            )
            return

        self.try_extend_robot(robot)
        self.try_start_robot(robot)

    def validate_path(
        self,
        path: List[str],
    ) -> bool:
        if any(
            node not in self.nodes
            for node in path
        ):
            return False

        for node_a, node_b in zip(
            path,
            path[1:],
        ):
            if edge_key(
                node_a,
                node_b,
            ) not in self.connections:
                return False

        return True

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
            return

        if (
            old_node
            and self.occupied_nodes.get(
                old_node
            )
            == robot_name
        ):
            self.occupied_nodes.pop(
                old_node,
                None,
            )

        self.occupied_nodes[
            node
        ] = robot_name

        robot.previous_node = old_node
        robot.current_node = node

        if robot.full_path:
            new_index = self.find_path_index(
                robot,
                node,
            )

            if (
                new_index is not None
                and new_index
                >= robot.current_index
            ):
                self.release_traversed_resources(
                    robot,
                    new_index,
                )

                robot.current_index = (
                    new_index
                )

                if robot.mission_active:
                    robot.status = "MOVING"

                robot.waiting_reason = ""
                robot.blocked_resource = ""
                robot.blocked_by = ""

                if (
                    node
                    == robot.destination
                ):
                    robot.status = (
                        "ARRIVED_DESTINATION"
                    )

        self.try_progress_all()

    def find_path_index(
        self,
        robot: RobotState,
        node: str,
    ) -> Optional[int]:
        for index in range(
            robot.current_index,
            len(robot.full_path),
        ):
            if (
                robot.full_path[index]
                == node
            ):
                return index

        return None

    def release_traversed_resources(
        self,
        robot: RobotState,
        new_index: int,
    ) -> None:
        for index in range(
            robot.current_index + 1,
            new_index + 1,
        ):
            previous_node = (
                robot.full_path[
                    index - 1
                ]
            )

            current_node = (
                robot.full_path[index]
            )

            edge = edge_key(
                previous_node,
                current_node,
            )

            if (
                self.edge_reservations.get(
                    edge
                )
                == robot.name
            ):
                self.edge_reservations.pop(
                    edge,
                    None,
                )

            robot.reserved_edges.discard(
                edge
            )

            if (
                self.node_reservations.get(
                    current_node
                )
                == robot.name
            ):
                self.node_reservations.pop(
                    current_node,
                    None,
                )

            robot.reserved_nodes.discard(
                current_node
            )

        for index in range(
            0,
            new_index,
        ):
            old_node = robot.full_path[
                index
            ]

            if (
                self.node_reservations.get(
                    old_node
                )
                == robot.name
            ):
                self.node_reservations.pop(
                    old_node,
                    None,
                )

            robot.reserved_nodes.discard(
                old_node
            )

    def mission_active_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        previously_active = (
            robot.mission_active
        )

        robot.mission_active = bool(
            msg.data
        )

        if robot.mission_active:
            robot.ever_active = True

            if robot.status not in (
                "SAFETY_HOLD",
                "WAITING_FOR_RELEASE",
            ):
                robot.status = "MOVING"

            return

        if (
            previously_active
            and robot.ever_active
            and robot.mission_id
        ):
            if (
                robot.current_node
                == robot.destination
            ):
                self.publish_event(
                    "MISSION_COMPLETED "
                    f"robot={robot_name} "
                    "mission="
                    f"{robot.mission_id} "
                    "destination="
                    f"{robot.destination}"
                )

                self.clear_completed_mission(
                    robot
                )

            else:
                robot.status = (
                    "STOPPED_UNCONFIRMED"
                )

                robot.waiting_reason = (
                    "OPERATOR_CONFIRM_POSITION"
                )

                robot.locked_after_stop = True

                self.publish_event(
                    "MISSION_STOPPED_"
                    "POSITION_UNCONFIRMED "
                    f"robot={robot_name} "
                    "current_node="
                    f"{robot.current_node} "
                    "reservations_retained"
                )

        self.try_progress_all()

    def mission_state_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        self.robots[
            robot_name
        ].mission_state = (
            msg.data.strip()
        )

    def bridge_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        robot.bridge_connected = bool(
            msg.data
        )

        robot.last_status_time = (
            time.time()
        )

    def status_raw_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        del msg

        robot = self.robots[
            robot_name
        ]

        robot.bridge_connected = True
        robot.last_status_time = (
            time.time()
        )

    def safety_ok_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        robot.safety_ok = bool(
            msg.data
        )

        self.try_progress_all()

    def safety_hold_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        robot.safety_hold = bool(
            msg.data
        )

        if (
            robot.safety_hold
            and robot.mission_id
        ):
            robot.status = "SAFETY_HOLD"

            robot.waiting_reason = (
                "SAFETY_HOLD"
            )

        self.try_progress_all()

    def resume_allowed_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        self.robots[
            robot_name
        ].resume_allowed = bool(
            msg.data
        )

    def obstacle_callback(
        self,
        robot_name: str,
        msg: String,
    ) -> None:
        self.robots[
            robot_name
        ].obstacle_state = (
            msg.data.strip()
        )

    def gate_state_callback(
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

        robot.gate_state = payload

        if payload.get(
            "waiting_for_release"
        ):
            if robot.status not in (
                "SAFETY_HOLD",
                "STOPPED_UNCONFIRMED",
            ):
                robot.status = (
                    "WAITING_FOR_RELEASE"
                )

                robot.waiting_reason = (
                    "WAITING_FOR_ROUTE_RELEASE"
                )

    def resource_conflict(
        self,
        robot: RobotState,
        from_node: str,
        to_node: str,
    ) -> Optional[
        Tuple[str, str, str]
    ]:
        occupied_by = (
            self.occupied_nodes.get(
                to_node
            )
        )

        if occupied_by not in (
            None,
            robot.name,
        ):
            return (
                "NODE_OCCUPIED",
                to_node,
                occupied_by,
            )

        node_owner = (
            self.node_reservations.get(
                to_node
            )
        )

        if node_owner not in (
            None,
            robot.name,
        ):
            return (
                "NODE_RESERVED",
                to_node,
                node_owner,
            )

        edge = edge_key(
            from_node,
            to_node,
        )

        edge_owner = (
            self.edge_reservations.get(
                edge
            )
        )

        if edge_owner not in (
            None,
            robot.name,
        ):
            return (
                "EDGE_RESERVED",
                (
                    f"{edge[0]}--"
                    f"{edge[1]}"
                ),
                edge_owner,
            )

        return None

    def node_holding_allowed(
        self,
        node: str,
    ) -> bool:
        return bool(
            self.nodes.get(
                node,
                {},
            ).get(
                "holding_allowed",
                False,
            )
        )

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

        index = start_index + 1

        while index < len(
            robot.full_path
        ):
            segment.append(index)

            node = robot.full_path[
                index
            ]

            if self.node_holding_allowed(
                node
            ):
                break

            index += 1

        return segment

    def try_extend_robot(
        self,
        robot: RobotState,
    ) -> bool:
        if (
            not robot.mission_id
            or not robot.full_path
        ):
            return False

        if robot.locked_after_stop:
            return False

        if not robot.bridge_connected:
            return False

        if not robot.safety_ok:
            return False

        if robot.safety_hold:
            return False

        changed = False

        target_index = min(
            len(robot.full_path) - 1,
            (
                robot.current_index
                + self.lookahead_edges
            ),
        )

        while (
            robot.released_until_index
            < target_index
        ):
            segment = (
                self.next_atomic_segment(
                    robot
                )
            )

            if not segment:
                break

            conflict = None

            for index in segment:
                node_a = robot.full_path[
                    index - 1
                ]

                node_b = robot.full_path[
                    index
                ]

                conflict = (
                    self.resource_conflict(
                        robot,
                        node_a,
                        node_b,
                    )
                )

                if conflict is not None:
                    break

            if conflict is not None:
                (
                    reason,
                    resource,
                    owner,
                ) = conflict

                robot.waiting_reason = (
                    reason
                )

                robot.blocked_resource = (
                    resource
                )

                robot.blocked_by = owner

                if robot.started:
                    robot.status = (
                        "WAITING_FOR_RELEASE"
                    )
                else:
                    robot.status = (
                        "WAITING_INITIAL_RELEASE"
                    )

                break

            for index in segment:
                node_a = robot.full_path[
                    index - 1
                ]

                node_b = robot.full_path[
                    index
                ]

                edge = edge_key(
                    node_a,
                    node_b,
                )

                self.edge_reservations[
                    edge
                ] = robot.name

                robot.reserved_edges.add(
                    edge
                )

                if (
                    node_b
                    != robot.current_node
                ):
                    self.node_reservations[
                        node_b
                    ] = robot.name

                    robot.reserved_nodes.add(
                        node_b
                    )

                robot.released_until_index = (
                    index
                )

                changed = True

            robot.waiting_reason = ""
            robot.blocked_resource = ""
            robot.blocked_by = ""

            if (
                robot.released_until_index
                >= target_index
            ):
                break

        if changed:
            self.publish_release(robot)

        return changed

    def try_start_robot(
        self,
        robot: RobotState,
    ) -> None:
        if robot.started:
            return

        if not robot.mission_id:
            return

        if (
            robot.released_until_index
            <= robot.current_index
        ):
            return

        route_payload = {
            "mission_id":
                robot.mission_id,

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
                route_payload,
                separators=(",", ":"),
            ),
        )

        self.publish_string(
            self.robot_publishers[
                robot.name
            ]["start"],
            robot.destination,
        )

        robot.started = True

        robot.status = (
            "START_COMMAND_SENT"
        )

        robot.waiting_reason = ""

        self.publish_event(
            "MISSION_START_SENT "
            f"robot={robot.name} "
            "mission="
            f"{robot.mission_id} "
            "released_until="
            f"{robot.released_until_index}"
        )

    def publish_release(
        self,
        robot: RobotState,
    ) -> None:
        payload = {
            "mission_id":
                robot.mission_id,

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
            "index="
            f"{robot.released_until_index} "
            "node="
            f"{robot.full_path[robot.released_until_index]}"
        )

    def sorted_progress_order(
        self,
    ) -> List[RobotState]:
        return sorted(
            self.robots.values(),
            key=lambda robot: (
                (
                    0
                    if (
                        robot.started
                        and robot.mission_active
                    )
                    else 1
                ),
                -robot.priority,
                (
                    robot.queued_at
                    if robot.queued_at > 0.0
                    else float("inf")
                ),
                robot.name,
            ),
        )

    def try_progress_all(
        self,
    ) -> None:
        for robot in (
            self.sorted_progress_order()
        ):
            self.try_extend_robot(robot)
            self.try_start_robot(robot)

        self.detect_deadlock()

    def detect_deadlock(
        self,
    ) -> None:
        wait_pairs = []

        for robot in self.robots.values():
            if robot.blocked_by:
                wait_pairs.append(
                    (
                        robot.name,
                        robot.blocked_by,
                    )
                )

        deadlocks = []

        for robot_a, robot_b in wait_pairs:
            if (
                robot_b,
                robot_a,
            ) in wait_pairs:
                deadlocks.append(
                    tuple(
                        sorted(
                            (
                                robot_a,
                                robot_b,
                            )
                        )
                    )
                )

        unique_deadlocks = sorted(
            set(deadlocks)
        )

        signature = json.dumps(
            unique_deadlocks
        )

        if (
            unique_deadlocks
            and signature
            != self.last_deadlock_signature
        ):
            self.publish_event(
                "DEADLOCK_DETECTED "
                f"pairs={unique_deadlocks}"
            )

        self.last_deadlock_signature = (
            signature
            if unique_deadlocks
            else ""
        )

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

        if robot_name not in self.robots:
            self.publish_event(
                "CANCEL_REJECTED "
                f"unknown_robot={robot_name}"
            )
            return

        robot = self.robots[
            robot_name
        ]

        self.publish_bool(
            self.robot_publishers[
                robot_name
            ]["cancel"],
            True,
        )

        if robot.mission_id:
            robot.status = "CANCELLING"

            robot.waiting_reason = (
                "RESERVATIONS_RETAINED_"
                "UNTIL_POSITION_CONFIRMED"
            )

            self.publish_event(
                "CANCEL_SENT "
                f"robot={robot_name} "
                "reservations_retained"
            )

    def resume_callback(
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

        if robot_name not in self.robots:
            return

        robot = self.robots[
            robot_name
        ]

        if robot.safety_hold:
            self.publish_event(
                "RESUME_REJECTED "
                f"robot={robot_name} "
                "hold_still_active"
            )
            return

        if (
            not robot.safety_ok
            or not robot.resume_allowed
        ):
            self.publish_event(
                "RESUME_REJECTED "
                f"robot={robot_name} "
                "not_allowed"
            )
            return

        if (
            robot.released_until_index
            <= robot.current_index
        ):
            self.publish_event(
                "RESUME_REJECTED "
                f"robot={robot_name} "
                "no_released_edge"
            )
            return

        self.publish_bool(
            self.robot_publishers[
                robot_name
            ]["resume"],
            True,
        )

        self.publish_event(
            "RESUME_SENT "
            f"robot={robot_name}"
        )

    def estop_callback(
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
                    "all",
                )
            )
        else:
            robot_name = (
                msg.data.strip()
            )

        if robot_name in (
            "",
            "all",
        ):
            targets = list(
                self.robots.keys()
            )
        else:
            targets = [
                robot_name
            ]

        for target in targets:
            if target in self.robots:
                self.publish_bool(
                    self.robot_publishers[
                        target
                    ]["estop"],
                    True,
                )

        self.publish_event(
            "ESTOP_SENT "
            f"targets={targets}"
        )

    def reset_callback(
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
                    "all",
                )
            )
        else:
            robot_name = (
                msg.data.strip()
            )

        if robot_name in (
            "",
            "all",
        ):
            targets = list(
                self.robots.keys()
            )
        else:
            targets = [
                robot_name
            ]

        for target in targets:
            if target in self.robots:
                self.publish_bool(
                    self.robot_publishers[
                        target
                    ]["reset"],
                    True,
                )

        self.publish_event(
            "RESET_SENT "
            f"targets={targets}"
        )

    def set_current_node_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            return

        robot_name = str(
            payload.get(
                "robot",
                "",
            )
        ).strip()

        node = str(
            payload.get(
                "node",
                "",
            )
        ).strip().lower()

        if (
            robot_name not in self.robots
            or node not in self.nodes
        ):
            self.publish_event(
                "SET_NODE_REJECTED "
                "invalid_robot_or_node"
            )
            return

        self.publish_string(
            self.robot_publishers[
                robot_name
            ]["set_current"],
            node,
        )

        self.publish_event(
            "SET_NODE_SENT "
            f"robot={robot_name} "
            f"node={node}"
        )

    def clear_reservations_callback(
        self,
        msg: String,
    ) -> None:
        payload = self.parse_json(
            msg.data
        )

        if payload is None:
            self.publish_event(
                "CLEAR_RESERVATIONS_REJECTED "
                "invalid_json"
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
            or confirmed_node not in self.nodes
        ):
            self.publish_event(
                "CLEAR_RESERVATIONS_REJECTED "
                "invalid_robot_or_node"
            )
            return

        robot = self.robots[
            robot_name
        ]

        if robot.mission_active:
            self.publish_event(
                "CLEAR_RESERVATIONS_REJECTED "
                f"robot={robot_name} "
                "mission_active"
            )
            return

        self.release_all_reservations(
            robot
        )

        if (
            robot.current_node
            and self.occupied_nodes.get(
                robot.current_node
            )
            == robot_name
        ):
            self.occupied_nodes.pop(
                robot.current_node,
                None,
            )

        robot.previous_node = (
            robot.current_node
        )

        robot.current_node = (
            confirmed_node
        )

        self.occupied_nodes[
            confirmed_node
        ] = robot_name

        self.clear_mission_fields(
            robot
        )

        robot.locked_after_stop = False
        robot.status = "IDLE"

        self.publish_string(
            self.robot_publishers[
                robot_name
            ]["set_current"],
            confirmed_node,
        )

        self.publish_event(
            "RESERVATIONS_CLEARED "
            f"robot={robot_name} "
            "confirmed_node="
            f"{confirmed_node}"
        )

        self.try_progress_all()

    def release_all_reservations(
        self,
        robot: RobotState,
    ) -> None:
        for node in list(
            robot.reserved_nodes
        ):
            if (
                self.node_reservations.get(
                    node
                )
                == robot.name
            ):
                self.node_reservations.pop(
                    node,
                    None,
                )

        for edge in list(
            robot.reserved_edges
        ):
            if (
                self.edge_reservations.get(
                    edge
                )
                == robot.name
            ):
                self.edge_reservations.pop(
                    edge,
                    None,
                )

        robot.reserved_nodes.clear()
        robot.reserved_edges.clear()

    def clear_mission_fields(
        self,
        robot: RobotState,
    ) -> None:
        robot.pending_plan = None

        robot.mission_id = ""
        robot.destination = ""

        robot.full_path = []

        robot.current_index = 0
        robot.released_until_index = 0

        robot.started = False
        robot.ever_active = False

        robot.priority = 10
        robot.queued_at = 0.0

        robot.waiting_reason = ""
        robot.blocked_resource = ""
        robot.blocked_by = ""

    def clear_completed_mission(
        self,
        robot: RobotState,
    ) -> None:
        self.release_all_reservations(
            robot
        )

        self.clear_mission_fields(
            robot
        )

        robot.locked_after_stop = False
        robot.status = "IDLE"

    def timer_callback(
        self,
    ) -> None:
        current_time = time.time()

        for robot in self.robots.values():
            if (
                robot.last_status_time > 0.0
                and (
                    current_time
                    - robot.last_status_time
                )
                > self.status_timeout_sec
            ):
                robot.bridge_connected = False

        self.try_progress_all()
        self.publish_state()

    def publish_state(
        self,
    ) -> None:
        robots_payload: Dict[
            str,
            Any,
        ] = {}

        queue_payload = []

        for (
            robot_name,
            robot,
        ) in self.robots.items():
            next_node = ""

            if (
                robot.full_path
                and robot.current_index
                < len(robot.full_path) - 1
            ):
                next_node = robot.full_path[
                    robot.current_index + 1
                ]

            robot_data = {
                "current_node":
                    robot.current_node,

                "previous_node":
                    robot.previous_node,

                "bridge_connected":
                    robot.bridge_connected,

                "safety_ok":
                    robot.safety_ok,

                "safety_hold":
                    robot.safety_hold,

                "resume_allowed":
                    robot.resume_allowed,

                "obstacle_state":
                    robot.obstacle_state,

                "mission_active":
                    robot.mission_active,

                "mission_state":
                    robot.mission_state,

                "mission_id":
                    robot.mission_id,

                "destination":
                    robot.destination,

                "full_path":
                    robot.full_path,

                "current_index":
                    robot.current_index,

                "released_until_index":
                    robot.released_until_index,

                "next_node":
                    next_node,

                "status":
                    robot.status,

                "priority":
                    robot.priority,

                "waiting_reason":
                    robot.waiting_reason,

                "blocked_resource":
                    robot.blocked_resource,

                "blocked_by":
                    robot.blocked_by,

                "locked_after_stop":
                    robot.locked_after_stop,

                "reserved_nodes":
                    sorted(
                        robot.reserved_nodes
                    ),

                "reserved_edges": [
                    list(edge)
                    for edge in sorted(
                        robot.reserved_edges
                    )
                ],

                "gate":
                    robot.gate_state,
            }

            robots_payload[
                robot_name
            ] = robot_data

            if robot.waiting_reason:
                queue_payload.append(
                    {
                        "robot":
                            robot_name,

                        "mission_id":
                            robot.mission_id,

                        "destination":
                            robot.destination,

                        "reason":
                            robot.waiting_reason,

                        "resource":
                            robot.blocked_resource,

                        "blocked_by":
                            robot.blocked_by,

                        "priority":
                            robot.priority,

                        "queued_at":
                            robot.queued_at,
                    }
                )

        reservations = {
            "occupied_nodes":
                self.occupied_nodes,

            "node_reservations":
                self.node_reservations,

            "edge_reservations": [
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
                    self.edge_reservations.items()
                )
            ],
        }

        state = {
            "stamp":
                time.time(),

            "lookahead_edges":
                self.lookahead_edges,

            "robots":
                robots_payload,

            "reservations":
                reservations,

            "queue":
                queue_payload,
        }

        state_msg = String()

        state_msg.data = json.dumps(
            state,
            separators=(",", ":"),
        )

        self.state_pub.publish(
            state_msg
        )

        status_msg = String()

        status_msg.data = json.dumps(
            robots_payload,
            separators=(",", ":"),
        )

        self.status_pub.publish(
            status_msg
        )

        reservations_msg = String()

        reservations_msg.data = json.dumps(
            reservations,
            separators=(",", ":"),
        )

        self.reservations_pub.publish(
            reservations_msg
        )

        queue_msg = String()

        queue_msg.data = json.dumps(
            queue_payload,
            separators=(",", ":"),
        )

        self.queue_pub.publish(
            queue_msg
        )

        ready_msg = Bool()

        ready_msg.data = all(
            robot.bridge_connected
            and robot.safety_ok
            for robot
            in self.robots.values()
        )

        self.ready_pub.publish(
            ready_msg
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FleetManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
