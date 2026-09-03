#!/usr/bin/env python3

import json
import os
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32, Int32


class FeatureActionNode(Node):
    def __init__(self):
        super().__init__("feature_action_node")

        self.declare_parameter("ticks_per_mm", 27.25)

        self.declare_parameter(
            "graph_file",
            "/home/seniru/agv_ws/src/agv_mission_manager/config/track_graph_fleet.json",
        )

        self.declare_parameter("junction_center_offset_mm", 190.0)
        self.declare_parameter("station_stop_offset_mm", 200.0)

        self.declare_parameter("approach_raw_left_pwm", 30)
        self.declare_parameter("approach_raw_right_pwm", 30)

        self.declare_parameter("exit_raw_left_pwm", 30)
        self.declare_parameter("exit_raw_right_pwm", 30)

        self.declare_parameter("clear_junction_ignore_distance_mm", 300.0)
        self.declare_parameter("normal_line_speed", 30)
        self.declare_parameter("junction_line_speed", 22)
        self.declare_parameter("stop_at_junction_command_point", False)

        self.declare_parameter("event_cooldown_sec", 1.0)
        self.declare_parameter("move_timeout_sec", 20.0)

        self.declare_parameter("left_turn_angle_deg", 100.0)
        self.declare_parameter("right_turn_angle_deg", -100.0)
        self.declare_parameter("uturn_angle_deg", 180.0)

        self.declare_parameter("require_mission_active", True)

        # RFID mode.
        self.declare_parameter("use_rfid_node_events", True)
        self.declare_parameter("use_optical_feature_events", False)
        self.declare_parameter("rfid_arrival_debounce_sec", 1.0)

        self.ticks_per_mm = float(self.get_parameter("ticks_per_mm").value)
        self.graph_file = str(self.get_parameter("graph_file").value).strip()

        self.junction_center_offset_mm = float(
            self.get_parameter("junction_center_offset_mm").value
        )
        self.station_stop_offset_mm = float(
            self.get_parameter("station_stop_offset_mm").value
        )

        self.approach_raw_left_pwm = int(
            self.get_parameter("approach_raw_left_pwm").value
        )
        self.approach_raw_right_pwm = int(
            self.get_parameter("approach_raw_right_pwm").value
        )

        self.exit_raw_left_pwm = int(self.get_parameter("exit_raw_left_pwm").value)
        self.exit_raw_right_pwm = int(self.get_parameter("exit_raw_right_pwm").value)

        self.clear_junction_ignore_distance_mm = float(
            self.get_parameter("clear_junction_ignore_distance_mm").value
        )
        self.normal_line_speed = max(0, min(255, int(self.get_parameter("normal_line_speed").value)))
        self.junction_line_speed = max(0, min(255, int(self.get_parameter("junction_line_speed").value)))
        self.stop_at_junction_command_point = bool(
            self.get_parameter("stop_at_junction_command_point").value
        )

        self.event_cooldown_sec = float(
            self.get_parameter("event_cooldown_sec").value
        )
        self.move_timeout_sec = float(
            self.get_parameter("move_timeout_sec").value
        )

        self.left_turn_angle_deg = float(
            self.get_parameter("left_turn_angle_deg").value
        )
        self.right_turn_angle_deg = float(
            self.get_parameter("right_turn_angle_deg").value
        )
        self.uturn_angle_deg = float(
            self.get_parameter("uturn_angle_deg").value
        )

        self.require_mission_active = bool(
            self.get_parameter("require_mission_active").value
        )
        self.use_rfid_node_events = bool(
            self.get_parameter("use_rfid_node_events").value
        )
        self.use_optical_feature_events = bool(
            self.get_parameter("use_optical_feature_events").value
        )
        self.rfid_arrival_debounce_sec = float(
            self.get_parameter("rfid_arrival_debounce_sec").value
        )

        self.junction_center_offset_ticks = (
            self.junction_center_offset_mm * self.ticks_per_mm
        )
        self.station_stop_offset_ticks = (
            self.station_stop_offset_mm * self.ticks_per_mm
        )
        self.clear_junction_ignore_ticks = (
            self.clear_junction_ignore_distance_mm * self.ticks_per_mm
        )

        self.nodes: Dict[str, Dict[str, Any]] = {}
        self.load_graph()

        self.state = "NORMAL"

        self.mission_active = False
        self.manual_enable = False
        self.ignore_station_until_junction = False

        self.latest_left_ticks: Optional[int] = None
        self.latest_right_ticks: Optional[int] = None

        self.move_start_left_ticks: Optional[int] = None
        self.move_start_right_ticks: Optional[int] = None
        self.move_target_ticks = 0.0
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.move_start_time = 0.0

        self.clear_start_left_ticks: Optional[int] = None
        self.clear_start_right_ticks: Optional[int] = None
        self.clear_mode = "NONE"

        self.last_event_time = 0.0
        self.pending_turn_command = "NONE"

        # Route tracking from mission manager.
        self.route_nodes: List[str] = []
        self.route_index = 0
        self.current_node = ""
        self.last_rfid_node = ""
        self.last_rfid_time = 0.0

        # Publishers
        self.raw_cmd_pub = self.create_publisher(String, "/agv_1/cmd/raw", 10)
        self.stop_pub = self.create_publisher(Bool, "/agv_1/cmd/stop", 10)
        self.start_pub = self.create_publisher(Bool, "/agv_1/cmd/start", 10)

        self.turn_angle_pub = self.create_publisher(
            Float32, "/agv_1/cmd/turn_angle_deg", 10
        )

        self.clear_junction_pub = self.create_publisher(
            Bool, "/agv_1/feature/cmd/clear_junction", 10
        )

        self.state_pub = self.create_publisher(
            String, "/agv_1/feature_action/state", 10
        )
        self.event_pub = self.create_publisher(
            String, "/agv_1/feature_action/event", 10
        )

        self.junction_reached_pub = self.create_publisher(
            Bool, "/agv_1/junction_reached", 10
        )
        self.station_reached_pub = self.create_publisher(
            Bool, "/agv_1/station_reached", 10
        )

        # Subscribers
        self.create_subscription(
            String, "/agv_1/status_raw", self.status_raw_callback, 10
        )
        self.create_subscription(
            String, "/agv_1/feature/event", self.feature_event_callback, 10
        )
        self.create_subscription(
            String, "/agv_1/rfid/node", self.rfid_node_callback, 10
        )
        self.create_subscription(
            String, "/agv_1/mission/route_plan", self.route_plan_callback, 10
        )
        self.create_subscription(
            Int32, "/agv_1/mission/route_index", self.route_index_callback, 10
        )
        self.create_subscription(
            String, "/agv_1/mission/current_node", self.current_node_callback, 10
        )
        self.create_subscription(
            String, "/agv_1/junction_cmd", self.junction_cmd_callback, 10
        )
        self.create_subscription(
            Bool, "/agv_1/turn/done", self.turn_done_callback, 10
        )
        self.create_subscription(
            Bool, "/agv_1/feature_action/cmd/reset", self.reset_callback, 10
        )
        self.create_subscription(
            Bool, "/agv_1/mission/active", self.mission_active_callback, 10
        )
        self.create_subscription(
            Bool, "/agv_1/feature_action/cmd/enable", self.manual_enable_callback, 10
        )
        self.create_subscription(
            Bool,
            "/agv_1/feature_action/cmd/ignore_station_until_junction",
            self.ignore_station_until_junction_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Feature action node started in AGV1 RFID fleet mode")
        self.get_logger().info(f"Graph file: {self.graph_file}")
        self.get_logger().info(
            f"use_rfid_node_events={self.use_rfid_node_events}, "
            f"use_optical_feature_events={self.use_optical_feature_events}"
        )
        self.get_logger().info(
            f"junction_center_offset_mm={self.junction_center_offset_mm}, "
            f"ticks={self.junction_center_offset_ticks:.1f}"
        )
        self.get_logger().info(
            f"station_stop_offset_mm={self.station_stop_offset_mm}, "
            f"ticks={self.station_stop_offset_ticks:.1f}"
        )
        self.get_logger().info(
            f"clear_junction_ignore_distance_mm={self.clear_junction_ignore_distance_mm}, "
            f"ticks={self.clear_junction_ignore_ticks:.1f}"
        )
        self.get_logger().info(f"require_mission_active={self.require_mission_active}")

    # ==================================================
    # Graph
    # ==================================================

    def load_graph(self):
        graph_file = os.path.expanduser(self.graph_file)

        if not os.path.exists(graph_file):
            self.get_logger().warn(f"Graph file not found: {graph_file}")
            self.nodes = {}
            return

        try:
            with open(graph_file, "r", encoding="utf-8") as f:
                graph = json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"Failed to read graph file: {exc}")
            self.nodes = {}
            return

        raw_nodes = graph.get("nodes", {})
        if not isinstance(raw_nodes, dict):
            self.nodes = {}
            return

        nodes: Dict[str, Dict[str, Any]] = {}

        for name, info in raw_nodes.items():
            clean_name = str(name).strip().lower()
            if not clean_name:
                continue

            if not isinstance(info, dict):
                info = {}

            nodes[clean_name] = {
                "type": str(info.get("type", "unknown")).strip().lower(),
                "dead_end": bool(info.get("dead_end", False)),
                "junction_group": str(info.get("junction_group", clean_name)).strip().lower(),
                "junction_role": str(info.get("junction_role", "")).strip().lower(),
                "approach_offset_mm": self.safe_float(
                    info.get("approach_offset_mm", 0.0),
                    0.0,
                ),
                "approach_offsets_mm": self.clean_offset_map(
                    info.get("approach_offsets_mm", {}),
                ),
            }

        self.nodes = nodes

    def clean_offset_map(self, raw_value: Any) -> Dict[str, float]:
        if not isinstance(raw_value, dict):
            return {}

        result: Dict[str, float] = {}

        for node_name, offset_mm in raw_value.items():
            clean_name = str(node_name).strip().lower()

            if not clean_name:
                continue

            result[clean_name] = max(
                0.0,
                self.safe_float(offset_mm, 0.0),
            )

        return result

    def is_junction_node(self, node_name: str) -> bool:
        node = self.nodes.get(node_name.strip().lower(), {})
        return str(node.get("type", "")).strip().lower() == "junction"

    def is_station_node(self, node_name: str) -> bool:
        return not self.is_junction_node(node_name)

    def junction_role(self, node_name: str) -> str:
        node_name = node_name.strip().lower()
        node = self.nodes.get(node_name, {})
        role = str(node.get("junction_role", "")).strip().lower()

        if role:
            return role

        if self.is_junction_node(node_name) and node_name.endswith("_center"):
            return "center"

        return ""

    def is_junction_center_node(self, node_name: str) -> bool:
        return self.is_junction_node(node_name) and self.junction_role(node_name) in [
            "center",
            "decision",
        ]

    def is_junction_exit_node(self, node_name: str) -> bool:
        return self.is_junction_node(node_name) and self.junction_role(node_name) in [
            "exit",
            "verification",
        ]

    def same_junction_group(self, node_a: str, node_b: str) -> bool:
        node_a = node_a.strip().lower()
        node_b = node_b.strip().lower()

        if not self.is_junction_node(node_a) or not self.is_junction_node(node_b):
            return False

        group_a = str(self.nodes.get(node_a, {}).get("junction_group", node_a)).strip().lower()
        group_b = str(self.nodes.get(node_b, {}).get("junction_group", node_b)).strip().lower()

        return bool(group_a and group_a == group_b)

    def is_junction_exit_transition(self, previous_node: str, exit_node: str) -> bool:
        previous_node = previous_node.strip().lower()
        exit_node = exit_node.strip().lower()

        return (
            self.is_junction_center_node(previous_node)
            and self.is_junction_exit_node(exit_node)
            and self.same_junction_group(previous_node, exit_node)
        )

    def should_ignore_repeated_previous_rfid(self, seen_node: str) -> bool:
        seen_node = seen_node.strip().lower()
        previous_node = self.previous_route_node()
        current_node = self.current_node.strip().lower()

        return bool(
            seen_node
            and (
                seen_node == previous_node
                or seen_node == current_node
            )
        )

    def should_ignore_approach_exit_rfid(self, expected_node: str, seen_node: str) -> bool:
        expected_node = expected_node.strip().lower()
        seen_node = seen_node.strip().lower()

        return (
            seen_node == self.previous_route_node()
            and
            self.is_junction_center_node(expected_node)
            and self.is_junction_exit_node(seen_node)
            and self.same_junction_group(expected_node, seen_node)
        )

    def should_ignore_clearing_center_rfid(self, expected_node: str, seen_node: str) -> bool:
        expected_node = expected_node.strip().lower()
        seen_node = seen_node.strip().lower()

        return (
            self.is_junction_exit_node(expected_node)
            and self.is_junction_center_node(seen_node)
            and self.same_junction_group(expected_node, seen_node)
        )

    def approach_offset_mm_for_node(self, node_name: str) -> float:
        node_name = node_name.strip().lower()
        node = self.nodes.get(node_name, {})
        previous = self.previous_route_node()

        offsets = node.get("approach_offsets_mm", {})

        if isinstance(offsets, dict):
            if previous in offsets:
                return max(0.0, float(offsets[previous]))

            if "*" in offsets:
                return max(0.0, float(offsets["*"]))

        return max(0.0, float(node.get("approach_offset_mm", 0.0)))

    # ==================================================
    # Main callbacks
    # ==================================================

    def status_raw_callback(self, msg: String):
        data = self.parse_status_line(msg.data)

        if data is None:
            return

        if "L" not in data or "R" not in data:
            return

        self.latest_left_ticks = self.safe_int(data["L"])
        self.latest_right_ticks = self.safe_int(data["R"])

        self.update_motion_state()

    def mission_active_callback(self, msg: Bool):
        self.mission_active = bool(msg.data)

    def manual_enable_callback(self, msg: Bool):
        self.manual_enable = bool(msg.data)

        if self.manual_enable:
            self.publish_event("FEATURE_ACTION_MANUAL_ENABLE")
            self.get_logger().warn("Feature action manually enabled")
        else:
            self.publish_event("FEATURE_ACTION_MANUAL_DISABLE")
            self.get_logger().warn("Feature action manually disabled")

    def ignore_station_until_junction_callback(self, msg: Bool):
        self.ignore_station_until_junction = bool(msg.data)

        if self.ignore_station_until_junction:
            self.publish_event("IGNORE_STATION_UNTIL_JUNCTION_ON")
            self.get_logger().warn("Ignoring station events until next junction")
        else:
            self.publish_event("IGNORE_STATION_UNTIL_JUNCTION_OFF")
            self.get_logger().info("Station event ignore disabled")

    def route_plan_callback(self, msg: String):
        try:
            plan = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid route plan JSON: {exc}")
            return

        path = plan.get("path", [])

        if not isinstance(path, list):
            self.get_logger().warn("Route plan had no valid path list")
            return

        self.route_nodes = [str(x).strip().lower() for x in path if str(x).strip()]
        self.route_index = 0

        self.get_logger().info(f"RFID expected route updated: {self.route_nodes}")

    def route_index_callback(self, msg: Int32):
        self.route_index = int(msg.data)

    def current_node_callback(self, msg: String):
        self.current_node = msg.data.strip().lower()

    def expected_next_node(self) -> str:
        if not self.route_nodes:
            return ""

        next_index = self.route_index + 1

        if next_index < 0 or next_index >= len(self.route_nodes):
            return ""

        return self.route_nodes[next_index]

    def previous_route_node(self) -> str:
        if not self.route_nodes:
            return self.current_node

        if 0 <= self.route_index < len(self.route_nodes):
            return self.route_nodes[self.route_index]

        return self.current_node

    def expected_node_is_junction_exit(self, expected_node: str) -> bool:
        previous_node = self.previous_route_node()
        return self.is_junction_exit_transition(previous_node, expected_node)

    def publish_wrong_rfid(self, expected: str, got: str):
        self.stop_robot()
        self.send_branch_command("AUTO")

        self.state = "RFID_MISMATCH"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"

        self.publish_event(
            f"FEATURE_ACTION_TIMEOUT_WRONG_RFID_EXPECTED_{expected}_GOT_{got}"
        )
        self.publish_state()

        self.get_logger().error(
            f"Wrong RFID detected. expected={expected}, got={got}"
        )

    def confirm_junction_exit_rfid(self, node_name: str):
        self.send_branch_command("AUTO")

        self.state = "NORMAL"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"

        self.publish_junction_reached(True)
        self.publish_event("JUNCTION_REACHED")
        self.publish_event(f"JUNCTION_EXIT_CONFIRMED_{node_name}")
        self.publish_event("LINE_FOLLOW_RESUMED")
        self.publish_state()

        self.get_logger().info(
            f"RFID junction exit confirmed without stopping: {node_name}"
        )

    def handle_expected_junction_rfid(self, node_name: str):
        offset_mm = self.approach_offset_mm_for_node(node_name)

        if offset_mm > 0.0:
            self.start_move_to_junction_center(
                offset_mm=offset_mm,
                target_node=node_name,
                use_line_follow=True,
            )
            return

        self.stop_robot()

        self.state = "WAITING_FOR_JUNCTION_COMMAND"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"

        self.publish_junction_reached(True)
        self.publish_event("JUNCTION_REACHED")
        self.publish_state()

        self.get_logger().info(f"RFID junction reached: {node_name}")

    def rfid_node_callback(self, msg: String):
        if not self.use_rfid_node_events:
            return

        node_name = msg.data.strip().lower()

        if not node_name:
            return

        if not self.feature_actions_allowed():
            return

        now = time.time()

        if (
            node_name == self.last_rfid_node
            and (now - self.last_rfid_time) < self.rfid_arrival_debounce_sec
        ):
            return

        self.last_rfid_node = node_name
        self.last_rfid_time = now

        expected = self.expected_next_node()

        if not expected:
            self.get_logger().warn(
                f"Ignored RFID node {node_name}: no expected next node. "
                f"route={self.route_nodes}, index={self.route_index}"
            )
            return

        expected_is_exit = self.expected_node_is_junction_exit(expected)

        if self.state != "NORMAL":
            if self.state == "MOVING_TO_JUNCTION_CENTER":
                if (
                    node_name == self.move_target_node
                    or self.should_ignore_repeated_previous_rfid(node_name)
                ):
                    self.get_logger().info(
                        f"Ignored RFID node {node_name}: "
                        "already measuring distance to junction point"
                    )
                    return

                if self.should_ignore_approach_exit_rfid(
                    self.move_target_node,
                    node_name,
                ):
                    self.get_logger().info(
                        f"Ignored approach-side junction exit RFID {node_name}: "
                        f"moving to {self.move_target_node}"
                    )
                    return

                if self.is_junction_node(node_name):
                    target = self.move_target_node or expected
                    self.publish_wrong_rfid(target, node_name)
                else:
                    self.get_logger().info(
                        f"Ignored non-route station RFID {node_name}: "
                        f"moving to junction point for {self.move_target_node}"
                    )
                return

            if self.state == "CLEARING_JUNCTION":
                if node_name != expected:
                    if self.should_ignore_repeated_previous_rfid(node_name):
                        self.get_logger().info(
                            f"Ignored repeated RFID {node_name}: "
                            f"clearing toward {expected}"
                        )
                        return

                    if self.should_ignore_clearing_center_rfid(expected, node_name):
                        self.get_logger().info(
                            f"Ignored junction center RFID {node_name}: "
                            f"clearing toward exit {expected}"
                        )
                        return

                    if self.is_junction_node(node_name):
                        self.publish_wrong_rfid(expected, node_name)
                    else:
                        self.get_logger().info(
                            f"Ignored non-route station RFID {node_name}: "
                            f"expected {expected}"
                        )
                    return

                if expected_is_exit:
                    self.confirm_junction_exit_rfid(node_name)
                    return

                if self.is_junction_node(node_name):
                    self.send_branch_command("AUTO")
                    self.handle_expected_junction_rfid(node_name)
                    return

            self.get_logger().info(
                f"Ignored RFID node {node_name}: state={self.state}"
            )
            return

        if node_name != expected:
            if self.should_ignore_repeated_previous_rfid(node_name):
                self.get_logger().info(
                    f"Ignored repeated RFID {node_name}: expected {expected}"
                )
                return

            if self.should_ignore_approach_exit_rfid(expected, node_name):
                self.get_logger().info(
                    f"Ignored approach-side junction exit RFID {node_name}: "
                    f"expected center {expected}"
                )
                return

            if self.is_junction_node(node_name):
                self.publish_wrong_rfid(expected, node_name)
            else:
                self.get_logger().info(
                    f"Ignored non-route station RFID {node_name}: "
                    f"expected {expected}"
                )
            return

        if self.ignore_station_until_junction and self.is_junction_node(node_name):
            self.ignore_station_until_junction = False
            self.publish_event("IGNORE_STATION_UNTIL_JUNCTION_AUTO_CLEARED")
            self.get_logger().info(
                "Station ignore auto-cleared because expected junction RFID was detected"
            )

        if expected_is_exit:
            self.confirm_junction_exit_rfid(node_name)
            return

        if self.is_junction_node(node_name):
            self.handle_expected_junction_rfid(node_name)
            return

        self.stop_robot()

        if self.ignore_station_until_junction:
            self.get_logger().warn(
                f"Ignored RFID station {node_name}: ignore_station_until_junction is active"
            )
            return

        self.state = "STATION_REACHED"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"

        self.publish_station_reached(True)
        self.publish_event("STATION_REACHED")
        self.publish_state()

        self.get_logger().info(f"RFID station reached: {node_name}")

    def feature_event_callback(self, msg: String):
        if not self.use_optical_feature_events:
            return

        event = msg.data.strip()

        if not event:
            return

        if not self.feature_actions_allowed():
            return

        if self.state != "NORMAL":
            return

        now = time.time()
        if now - self.last_event_time < self.event_cooldown_sec:
            return

        if event == "JUNCTION_APPROACH_CONFIRMED":
            if self.ignore_station_until_junction:
                self.ignore_station_until_junction = False
                self.publish_event("IGNORE_STATION_UNTIL_JUNCTION_AUTO_CLEARED")
                self.get_logger().info(
                    "Station ignore auto-cleared because junction was detected"
                )

            self.last_event_time = now
            self.start_move_to_junction_center()
            return

        if event == "STATION_APPROACH_CONFIRMED":
            if self.ignore_station_until_junction:
                return

            self.last_event_time = now
            self.handle_station_approach()
            return

    def junction_cmd_callback(self, msg: String):
        command = msg.data.strip().lower()

        if self.state != "WAITING_FOR_JUNCTION_COMMAND":
            self.get_logger().warn(
                f"Ignored junction command {command}: state={self.state}"
            )
            return

        if command == "straight":
            self.command_straight_exit()
            return

        if command == "left":
            self.command_smooth_arc_exit("LEFT")
            return

        if command == "right":
            self.command_smooth_arc_exit("RIGHT")
            return

        if command in ["uturn", "u-turn", "u_turn"]:
            self.command_turn("UTURN", self.uturn_angle_deg)
            return

        self.get_logger().warn(f"Unknown junction command: {command}")

    def turn_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state != "TURNING_AT_JUNCTION":
            return

        self.publish_event(f"TURN_DONE_{self.pending_turn_command}")

        self.start_junction_clearing_with_line_follow()

        self.publish_event("EXITING_JUNCTION_AFTER_TURN")

        self.get_logger().info(
            f"Turn done after {self.pending_turn_command}. "
            "Starting line-follow junction clearing."
        )

    def reset_callback(self, msg: Bool):
        if not msg.data:
            return

        self.stop_robot()
        self.send_branch_command("AUTO")

        self.state = "NORMAL"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"

        self.ignore_station_until_junction = False

        self.move_start_left_ticks = None
        self.move_start_right_ticks = None
        self.clear_start_left_ticks = None
        self.clear_start_right_ticks = None

        self.publish_junction_reached(False)
        self.publish_station_reached(False)
        self.publish_event("FEATURE_ACTION_RESET")
        self.publish_state()

        self.get_logger().info("Feature action reset")

    def timer_callback(self):
        self.publish_state()
        self.check_timeout()

    # ==================================================
    # Optical backup state machine
    # ==================================================

    def start_move_to_junction_center(
        self,
        offset_mm: Optional[float] = None,
        target_node: str = "",
        use_line_follow: bool = False,
    ):
        if not self.have_ticks():
            self.get_logger().warn("Cannot move to junction center: no encoder ticks yet")
            return

        if offset_mm is None:
            target_ticks = self.junction_center_offset_ticks
            offset_mm = self.junction_center_offset_mm
        else:
            offset_mm = max(0.0, float(offset_mm))
            target_ticks = offset_mm * self.ticks_per_mm

        self.move_start_left_ticks = self.latest_left_ticks
        self.move_start_right_ticks = self.latest_right_ticks
        self.move_target_ticks = target_ticks
        self.move_target_node = target_node.strip().lower()
        self.move_mode = "LINE_FOLLOW" if use_line_follow else "RAW_DRIVE"
        self.move_start_time = time.time()

        self.state = "MOVING_TO_JUNCTION_CENTER"
        self.pending_turn_command = "NONE"
        self.clear_mode = "NONE"

        self.publish_junction_reached(False)
        self.publish_event("MOVING_TO_JUNCTION_CENTER")

        if use_line_follow:
            self.apply_junction_line_speed()
            self.send_raw_start()
        else:
            self.send_raw_drive(
                self.approach_raw_left_pwm,
                self.approach_raw_right_pwm,
            )

        self.get_logger().info(
            f"Moving to junction center for {offset_mm:.1f} mm "
            f"({target_ticks:.1f} ticks), mode={self.move_mode}"
        )

    def finish_move_to_junction_center(self, rfid_confirmed: bool = False):
        reached_node = self.move_target_node
        was_line_follow = self.move_mode == "LINE_FOLLOW"

        if self.stop_at_junction_command_point or not was_line_follow:
            self.stop_robot()
        else:
            self.apply_junction_line_speed()
            self.send_raw_start()


        self.state = "WAITING_FOR_JUNCTION_COMMAND"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"

        self.publish_junction_reached(True)
        self.publish_event("JUNCTION_REACHED")

        if rfid_confirmed:
            self.publish_event(f"JUNCTION_CENTER_RFID_CONFIRMED_{reached_node}")

        self.publish_state()

    def handle_station_approach(self):
        if self.station_stop_offset_ticks <= 0.0:
            self.stop_robot()
            self.state = "STATION_REACHED"
            self.publish_station_reached(True)
            self.publish_event("STATION_REACHED")
            self.publish_state()
            return

        if not self.have_ticks():
            self.get_logger().warn("Cannot move to station stop point: no encoder ticks yet")
            return

        self.move_start_left_ticks = self.latest_left_ticks
        self.move_start_right_ticks = self.latest_right_ticks
        self.move_target_ticks = self.station_stop_offset_ticks
        self.move_start_time = time.time()

        self.state = "MOVING_TO_STATION_STOP"
        self.move_target_node = ""
        self.move_mode = "RAW_DRIVE"
        self.publish_event("MOVING_TO_STATION_STOP")

        self.send_raw_drive(
            self.approach_raw_left_pwm,
            self.approach_raw_right_pwm,
        )

    # ==================================================
    # Junction command handling
    # ==================================================

    def command_straight_exit(self):
        self.pending_turn_command = "STRAIGHT"

        self.apply_junction_line_speed()
        self.send_branch_command("STRAIGHT")
        self.start_junction_clearing_with_line_follow()
        self.publish_event("EXITING_JUNCTION_STRAIGHT_LINE_FOLLOW")

        self.get_logger().info("Junction command STRAIGHT: exiting with line follow")

    def command_smooth_arc_exit(self, name: str):
        self.pending_turn_command = name

        self.apply_junction_line_speed()
        self.send_branch_command(name)
        self.start_junction_clearing_with_line_follow()
        self.publish_event(f"EXITING_JUNCTION_{name}_ARC_LINE_FOLLOW")

        self.get_logger().info(
            f"Junction command {name}: exiting smooth arc with line follow"
        )

    def command_turn(self, name: str, angle_deg: float):
        self.pending_turn_command = name
        self.state = "TURNING_AT_JUNCTION"
        self.move_start_time = time.time()
        self.send_branch_command("AUTO")

        msg = Float32()
        msg.data = float(angle_deg)
        self.turn_angle_pub.publish(msg)

        self.publish_event(f"TURN_COMMAND_SENT_{name}")
        self.publish_state()

        self.get_logger().info(
            f"Junction command {name}: sent turn angle {angle_deg:.1f}"
        )

    def start_junction_clearing_with_raw_drive(self):
        if not self.have_ticks():
            self.get_logger().warn("Cannot start junction clearing: no encoder ticks yet")
            return

        self.publish_clear_junction_ignore()

        self.clear_start_left_ticks = self.latest_left_ticks
        self.clear_start_right_ticks = self.latest_right_ticks
        self.clear_mode = "RAW_DRIVE"
        self.move_mode = "NONE"

        self.state = "CLEARING_JUNCTION"
        self.move_start_time = time.time()

        self.publish_junction_reached(False)

        self.send_raw_drive(
            self.exit_raw_left_pwm,
            self.exit_raw_right_pwm,
        )

    def start_junction_clearing_with_line_follow(self):
        if not self.have_ticks():
            self.get_logger().warn("Cannot start junction clearing: no encoder ticks yet")
            return

        self.publish_clear_junction_ignore()

        self.clear_start_left_ticks = self.latest_left_ticks
        self.clear_start_right_ticks = self.latest_right_ticks
        self.clear_mode = "LINE_FOLLOW"
        self.move_mode = "NONE"

        self.state = "CLEARING_JUNCTION"
        self.move_start_time = time.time()

        self.publish_junction_reached(False)

        self.apply_junction_line_speed()
        self.send_raw_start()

    def update_motion_state(self):
        if self.state == "MOVING_TO_JUNCTION_CENTER":
            distance = self.average_delta_ticks(
                self.move_start_left_ticks,
                self.move_start_right_ticks,
            )

            if distance >= self.move_target_ticks:
                self.finish_move_to_junction_center(rfid_confirmed=False)

            return

        if self.state == "MOVING_TO_STATION_STOP":
            distance = self.average_delta_ticks(
                self.move_start_left_ticks,
                self.move_start_right_ticks,
            )

            if distance >= self.move_target_ticks:
                self.stop_robot()

                self.state = "STATION_REACHED"
                self.move_target_node = ""
                self.move_mode = "NONE"
                self.publish_station_reached(True)
                self.publish_event("STATION_REACHED")
                self.publish_state()

            return

        if self.state == "CLEARING_JUNCTION":
            distance = self.average_delta_ticks(
                self.clear_start_left_ticks,
                self.clear_start_right_ticks,
            )

            if distance >= self.clear_junction_ignore_ticks:
                mode_before_reset = self.clear_mode

                if self.clear_mode == "RAW_DRIVE":
                    self.send_raw_start()

                self.send_branch_command("AUTO")
                self.restore_line_speed()

                self.state = "NORMAL"
                self.pending_turn_command = "NONE"
                self.clear_mode = "NONE"

                self.publish_event("JUNCTION_CLEARED")
                self.publish_event("LINE_FOLLOW_RESUMED")
                self.publish_state()

                self.get_logger().info(
                    f"Junction cleared. avg_delta={distance:.1f} ticks. "
                    f"mode={mode_before_reset}"
                )

            return

    def check_timeout(self):
        if self.state not in [
            "MOVING_TO_JUNCTION_CENTER",
            "MOVING_TO_STATION_STOP",
            "CLEARING_JUNCTION",
            "TURNING_AT_JUNCTION",
        ]:
            return

        if self.move_start_time <= 0.0:
            return

        elapsed = time.time() - self.move_start_time

        if elapsed <= self.move_timeout_sec:
            return

        self.stop_robot()
        self.send_branch_command("AUTO")

        old_state = self.state
        self.state = "NORMAL"
        self.pending_turn_command = "NONE"
        self.move_target_node = ""
        self.move_mode = "NONE"
        self.clear_mode = "NONE"
        self.ignore_station_until_junction = False

        self.publish_event(f"FEATURE_ACTION_TIMEOUT_{old_state}")
        self.publish_state()

        self.get_logger().warn(
            f"Feature action timeout in state={old_state}. Robot stopped."
        )

    # ==================================================
    # Permission
    # ==================================================

    def feature_actions_allowed(self) -> bool:
        if not self.require_mission_active:
            return True

        return self.mission_active or self.manual_enable

    # ==================================================
    # Publishers / commands
    # ==================================================

    def send_raw_command(self, command: str):
        msg = String()
        msg.data = command
        self.raw_cmd_pub.publish(msg)

    def send_raw_drive(self, left_pwm: int, right_pwm: int):
        left_pwm = max(-255, min(255, int(left_pwm)))
        right_pwm = max(-255, min(255, int(right_pwm)))
        self.send_raw_command(f"C:RAW_DRIVE,{left_pwm},{right_pwm}")

    def send_raw_start(self):
        self.send_raw_command("C:START")

    def send_line_speed(self, speed: int):
        speed = max(0, min(255, int(speed)))
        self.send_raw_command(f"C:SET_SPEED,{speed}")

    def apply_junction_line_speed(self):
        self.send_line_speed(self.junction_line_speed)

    def restore_line_speed(self):
        self.send_line_speed(self.normal_line_speed)

    def send_branch_command(self, branch: str):
        branch = str(branch).strip().upper()

        if branch in ["", "AUTO", "NONE"]:
            self.send_raw_command("C:CLEAR_BRANCH")
            return

        if branch not in ["LEFT", "RIGHT", "STRAIGHT"]:
            self.get_logger().warn(f"Unknown Arduino branch command: {branch}")
            return

        self.send_raw_command(f"C:SET_BRANCH,{branch}")

    def stop_robot(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

    def publish_clear_junction_ignore(self):
        msg = Bool()
        msg.data = True
        self.clear_junction_pub.publish(msg)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_event(self, event: str):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)

    def publish_junction_reached(self, reached: bool):
        msg = Bool()
        msg.data = bool(reached)
        self.junction_reached_pub.publish(msg)

    def publish_station_reached(self, reached: bool):
        msg = Bool()
        msg.data = bool(reached)
        self.station_reached_pub.publish(msg)

    # ==================================================
    # Helpers
    # ==================================================

    def have_ticks(self) -> bool:
        return self.latest_left_ticks is not None and self.latest_right_ticks is not None

    def average_delta_ticks(
        self,
        start_left: Optional[int],
        start_right: Optional[int],
    ) -> float:
        if (
            start_left is None
            or start_right is None
            or self.latest_left_ticks is None
            or self.latest_right_ticks is None
        ):
            return 0.0

        left_delta = abs(self.latest_left_ticks - start_left)
        right_delta = abs(self.latest_right_ticks - start_right)

        return 0.5 * (left_delta + right_delta)

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

        return data

    @staticmethod
    def safe_int(value: str) -> int:
        try:
            return int(value)
        except ValueError:
            return 0

    @staticmethod
    def safe_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default


def main(args=None):
    rclpy.init(args=args)

    node = FeatureActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
