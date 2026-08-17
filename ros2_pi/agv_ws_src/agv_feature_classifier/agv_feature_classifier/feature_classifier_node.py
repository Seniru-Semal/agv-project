#!/usr/bin/env python3

import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class FeatureClassifierNode(Node):
    def __init__(self):
        super().__init__("feature_classifier_node")

        # ==================================================
        # Parameters
        # ==================================================
        #
        # Compact marker logic:
        #
        #   Station:
        #     marker/crossbar -> WIDE/SOLID continues
        #
        #   Junction:
        #     marker/crossbar -> marker clears -> normal narrow line returns
        #
        # This is designed for your compact layout:
        #   junction: marker -> 100 mm line -> junction center
        #   station: marker -> solid pad immediately

        self.declare_parameter("station_wide_confirm_frames", 12)
        self.declare_parameter("station_solid_confirm_frames", 5)
        self.declare_parameter("junction_narrow_confirm_frames", 2)
        self.declare_parameter("marker_clear_confirm_frames", 2)
        self.declare_parameter("classification_timeout_sec", 2.0)
        self.declare_parameter("event_cooldown_sec", 0.5)

        # Encoder-based marker-ignore support.
        # If ticks_per_mm is 0.0, ignore mode will not auto-clear by distance.
        self.declare_parameter("ticks_per_mm", 27.25)
        self.declare_parameter("clear_junction_ignore_distance_mm", 300.0)

        self.station_wide_confirm_frames = int(
            self.get_parameter("station_wide_confirm_frames").value
        )
        self.station_solid_confirm_frames = int(
            self.get_parameter("station_solid_confirm_frames").value
        )
        self.junction_narrow_confirm_frames = int(
            self.get_parameter("junction_narrow_confirm_frames").value
        )
        self.marker_clear_confirm_frames = int(
            self.get_parameter("marker_clear_confirm_frames").value
        )
        self.classification_timeout_sec = float(
            self.get_parameter("classification_timeout_sec").value
        )
        self.event_cooldown_sec = float(
            self.get_parameter("event_cooldown_sec").value
        )

        self.ticks_per_mm = float(self.get_parameter("ticks_per_mm").value)
        self.clear_junction_ignore_distance_mm = float(
            self.get_parameter("clear_junction_ignore_distance_mm").value
        )

        # ==================================================
        # Internal state
        # ==================================================

        self.state = "NORMAL"

        # This is the important new lockout.
        # If the feature action node is busy, this classifier ignores markers.
        self.action_state = "NORMAL"

        self.last_event_time = 0.0
        self.classification_start_time = 0.0

        self.continuous_wide_frames = 0
        self.continuous_solid_frames = 0
        self.marker_clear_frames = 0
        self.narrow_line_frames = 0

        self.marker_has_cleared = False

        self.current_feature_type = "NONE"
        self.last_event = "NONE"

        self.latest_left_ticks: Optional[int] = None
        self.latest_right_ticks: Optional[int] = None

        self.ignore_markers = False
        self.ignore_start_left_ticks: Optional[int] = None
        self.ignore_start_right_ticks: Optional[int] = None

        # Latest parsed floor state
        self.latest_marker = False
        self.latest_wide = False
        self.latest_solid = False
        self.latest_valid = False
        self.latest_active = 0
        self.latest_mactive = 0
        self.latest_pos = 0

        # ==================================================
        # ROS I/O
        # ==================================================

        self.create_subscription(
            String,
            "/agv_1/status_raw",
            self.status_callback,
            20,
        )

        self.create_subscription(
            String,
            "/agv_1/feature_action/state",
            self.feature_action_state_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/agv_1/feature/cmd/clear_junction",
            self.clear_junction_callback,
            10,
        )

        self.feature_state_pub = self.create_publisher(
            String,
            "/agv_1/feature/state",
            10,
        )

        self.feature_type_pub = self.create_publisher(
            String,
            "/agv_1/feature/type",
            10,
        )

        self.feature_event_pub = self.create_publisher(
            String,
            "/agv_1/feature/event",
            10,
        )

        self.marker_detected_pub = self.create_publisher(
            Bool,
            "/agv_1/feature/marker_detected",
            10,
        )

        self.station_detected_pub = self.create_publisher(
            Bool,
            "/agv_1/feature/station_detected",
            10,
        )

        self.junction_detected_pub = self.create_publisher(
            Bool,
            "/agv_1/feature/junction_detected",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("AGV feature classifier started")
        self.get_logger().info("Using compact marker classifier with action-state lockout")
        self.get_logger().info(
            f"station_wide_confirm_frames={self.station_wide_confirm_frames}"
        )
        self.get_logger().info(
            f"station_solid_confirm_frames={self.station_solid_confirm_frames}"
        )
        self.get_logger().info(
            f"junction_narrow_confirm_frames={self.junction_narrow_confirm_frames}"
        )
        self.get_logger().info(
            f"marker_clear_confirm_frames={self.marker_clear_confirm_frames}"
        )
        self.get_logger().info(
            f"classification_timeout_sec={self.classification_timeout_sec}"
        )
        self.get_logger().info(f"ticks_per_mm={self.ticks_per_mm}")

    # ==================================================
    # Feature action state lockout
    # ==================================================

    def feature_action_state_callback(self, msg: String):
        self.action_state = msg.data.strip()

        # If the action node is doing something, cancel any in-progress
        # classifier state so it cannot publish more fake station/junction events.
        if self.action_state != "NORMAL":
            if self.state != "NORMAL":
                self.state = "NORMAL"
                self.current_feature_type = "IGNORED_DURING_ACTION"

    # ==================================================
    # Input parsing
    # ==================================================

    def status_callback(self, msg: String):
        data = self.parse_status_line(msg.data)

        if data is None:
            return

        self.latest_marker = self.safe_int(data.get("MARKER", "0")) == 1
        self.latest_wide = self.safe_int(data.get("WIDE", "0")) == 1
        self.latest_solid = self.safe_int(data.get("SOLID", "0")) == 1
        self.latest_valid = self.safe_int(data.get("VALID", "0")) == 1
        self.latest_active = self.safe_int(data.get("ACTIVE", "0"))
        self.latest_mactive = self.safe_int(data.get("MACTIVE", "0"))
        self.latest_pos = self.safe_int(data.get("POS", "0"))

        self.latest_left_ticks = self.safe_int(data.get("L", "0"))
        self.latest_right_ticks = self.safe_int(data.get("R", "0"))

        self.update_ignore_distance()

        # Main fix:
        # The classifier only classifies when the action node is NORMAL.
        # When the robot is moving to a junction, sitting at a junction,
        # turning, clearing a junction, or stopped at a station, raw marker
        # detections are ignored.
        if self.action_state != "NORMAL":
            return

        if self.ignore_markers:
            return

        if self.state == "NORMAL":
            self.handle_normal_state()
        elif self.state == "CLASSIFYING":
            self.handle_classifying_state()

    def parse_status_line(self, line: str) -> Optional[Dict[str, str]]:
        line = line.strip()

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

    # ==================================================
    # Classifier state machine
    # ==================================================

    def handle_normal_state(self):
        now = time.time()

        if now - self.last_event_time < self.event_cooldown_sec:
            return

        if self.latest_marker or self.latest_wide:
            self.start_classification()

    def start_classification(self):
        self.state = "CLASSIFYING"
        self.classification_start_time = time.time()

        self.continuous_wide_frames = 0
        self.continuous_solid_frames = 0
        self.marker_clear_frames = 0
        self.narrow_line_frames = 0
        self.marker_has_cleared = False

        self.current_feature_type = "MARKER_DETECTED"
        self.publish_event("MARKER_DETECTED")

        self.get_logger().info(
            "Marker detected. Classifying compact station/junction pattern."
        )

    def handle_classifying_state(self):
        now = time.time()
        elapsed = now - self.classification_start_time

        # --------------------------------------------------
        # Station pattern:
        #   marker/crossbar -> WIDE/SOLID continues as solid pad
        # --------------------------------------------------

        if self.latest_wide or self.latest_marker:
            self.continuous_wide_frames += 1
        else:
            self.continuous_wide_frames = 0

        if self.latest_solid:
            self.continuous_solid_frames += 1
        else:
            self.continuous_solid_frames = 0

        station_wide_ok = (
            self.continuous_wide_frames >= self.station_wide_confirm_frames
        )
        station_solid_ok = (
            self.continuous_solid_frames >= self.station_solid_confirm_frames
        )

        if station_wide_ok and station_solid_ok:
            self.confirm_station()
            return

        # --------------------------------------------------
        # Junction pattern:
        #   marker/crossbar -> marker clears -> narrow line returns
        # --------------------------------------------------

        if not self.latest_wide and not self.latest_marker:
            self.marker_clear_frames += 1
        else:
            self.marker_clear_frames = 0

        if self.marker_clear_frames >= self.marker_clear_confirm_frames:
            self.marker_has_cleared = True

        is_narrow_line = (
            self.latest_valid
            and not self.latest_wide
            and not self.latest_marker
            and 1 <= self.latest_active <= 5
        )

        if self.marker_has_cleared and is_narrow_line:
            self.narrow_line_frames += 1
        else:
            if self.latest_wide or self.latest_marker:
                self.narrow_line_frames = 0

        if self.narrow_line_frames >= self.junction_narrow_confirm_frames:
            self.confirm_junction()
            return

        # --------------------------------------------------
        # Timeout fallback.
        #
        # If the marker remains wide/solid, classify as station.
        # Otherwise classify as junction.
        # --------------------------------------------------

        if elapsed >= self.classification_timeout_sec:
            if self.latest_wide or self.latest_solid:
                self.confirm_station()
            else:
                self.confirm_junction()
            return

    def confirm_station(self):
        self.state = "NORMAL"
        self.current_feature_type = "STATION_APPROACH"
        self.last_event_time = time.time()

        self.publish_event("STATION_APPROACH_CONFIRMED")
        self.get_logger().info("Station approach confirmed")

    def confirm_junction(self):
        self.state = "NORMAL"
        self.current_feature_type = "JUNCTION_APPROACH"
        self.last_event_time = time.time()

        self.publish_event("JUNCTION_APPROACH_CONFIRMED")
        self.get_logger().info("Junction approach confirmed")

    # ==================================================
    # Junction clearing support
    # ==================================================

    def clear_junction_callback(self, msg: Bool):
        if not msg.data:
            return

        self.start_junction_clear_ignore()

    def start_junction_clear_ignore(self):
        if self.latest_left_ticks is None or self.latest_right_ticks is None:
            self.get_logger().warn("Cannot start marker ignore: no encoder ticks yet")
            return

        self.ignore_markers = True
        self.ignore_start_left_ticks = self.latest_left_ticks
        self.ignore_start_right_ticks = self.latest_right_ticks

        self.state = "NORMAL"
        self.current_feature_type = "CLEARING_JUNCTION"
        self.publish_event("MARKER_IGNORE_STARTED")

        self.get_logger().info("Started junction-clearing marker ignore zone")

    def update_ignore_distance(self):
        if not self.ignore_markers:
            return

        if self.ticks_per_mm <= 0.0:
            # If not calibrated, do not auto-clear by distance.
            return

        if (
            self.latest_left_ticks is None
            or self.latest_right_ticks is None
            or self.ignore_start_left_ticks is None
            or self.ignore_start_right_ticks is None
        ):
            return

        left_delta = abs(self.latest_left_ticks - self.ignore_start_left_ticks)
        right_delta = abs(self.latest_right_ticks - self.ignore_start_right_ticks)

        avg_ticks = (left_delta + right_delta) / 2.0
        distance_mm = avg_ticks / self.ticks_per_mm

        if distance_mm >= self.clear_junction_ignore_distance_mm:
            self.ignore_markers = False
            self.ignore_start_left_ticks = None
            self.ignore_start_right_ticks = None
            self.current_feature_type = "NONE"

            self.publish_event("MARKER_IGNORE_FINISHED")
            self.get_logger().info("Finished junction-clearing marker ignore zone")

    # ==================================================
    # Periodic publishing
    # ==================================================

    def timer_callback(self):
        state_text = self.state

        if self.ignore_markers:
            state_text = "CLEARING_JUNCTION_IGNORE_MARKERS"

        if self.action_state != "NORMAL":
            state_text = "IGNORED_DURING_ACTION"

        state_msg = String()
        state_msg.data = state_text
        self.feature_state_pub.publish(state_msg)

        type_msg = String()
        type_msg.data = self.current_feature_type
        self.feature_type_pub.publish(type_msg)

        marker_msg = Bool()
        marker_msg.data = bool(self.latest_marker)
        self.marker_detected_pub.publish(marker_msg)

        station_msg = Bool()
        station_msg.data = self.current_feature_type == "STATION_APPROACH"
        self.station_detected_pub.publish(station_msg)

        junction_msg = Bool()
        junction_msg.data = self.current_feature_type == "JUNCTION_APPROACH"
        self.junction_detected_pub.publish(junction_msg)

    def publish_event(self, event: str):
        self.last_event = event

        msg = String()
        msg.data = event
        self.feature_event_pub.publish(msg)

    # ==================================================
    # Helpers
    # ==================================================

    @staticmethod
    def safe_int(value: str) -> int:
        try:
            return int(value)
        except Exception:
            return 0


def main(args=None):
    rclpy.init(args=args)

    node = FeatureClassifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
