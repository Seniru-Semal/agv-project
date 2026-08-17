#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Bool, String


class SafetyHoldNode(Node):
    def __init__(self):
        super().__init__("safety_hold_node")

        self.declare_parameter(
            "robot_ns",
            "agv_1",
        )

        self.declare_parameter(
            "clear_stable_sec",
            2.0,
        )

        self.declare_parameter(
            "stop_repeat_sec",
            0.25,
        )

        self.declare_parameter(
            "auto_clear_when_mission_inactive",
            True,
        )

        self.robot_ns = str(
            self.get_parameter(
                "robot_ns"
            ).value
        ).strip("/")

        self.clear_stable_sec = float(
            self.get_parameter(
                "clear_stable_sec"
            ).value
        )

        self.stop_repeat_sec = float(
            self.get_parameter(
                "stop_repeat_sec"
            ).value
        )

        self.auto_clear_when_mission_inactive = bool(
            self.get_parameter(
                "auto_clear_when_mission_inactive"
            ).value
        )

        self.obstacle_state = "UNKNOWN"

        self.mission_active = False
        self.mission_state = "UNKNOWN"
        self.feature_state = "UNKNOWN"
        self.turn_state = "UNKNOWN"

        self.safety_ok = False
        self.bridge_connected = False

        self.hold_active = False
        self.hold_reason = "NONE"

        self.held_mission_state = "UNKNOWN"
        self.held_feature_state = "UNKNOWN"
        self.held_turn_state = "UNKNOWN"

        self.clear_since = None
        self.last_stop_publish_time = 0.0
        self.last_status_publish_time = 0.0

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            String,
            f"/{self.robot_ns}/obstacle/state",
            self.obstacle_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"/{self.robot_ns}/mission/active",
            self.mission_active_callback,
            10,
        )

        self.create_subscription(
            String,
            f"/{self.robot_ns}/mission/state",
            self.mission_state_callback,
            10,
        )

        self.create_subscription(
            String,
            f"/{self.robot_ns}/feature_action/state",
            self.feature_state_callback,
            10,
        )

        self.create_subscription(
            String,
            f"/{self.robot_ns}/turn/state",
            self.turn_state_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"/{self.robot_ns}/safety/ok",
            self.safety_ok_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f"/{self.robot_ns}/bridge_connected",
            self.bridge_callback,
            10,
        )

        self.create_subscription(
            Bool,
            (
                f"/{self.robot_ns}/"
                "safety_hold/resume_request"
            ),
            self.resume_callback,
            10,
        )

        self.create_subscription(
            Bool,
            (
                f"/{self.robot_ns}/"
                "safety_hold/reset_request"
            ),
            self.reset_callback,
            10,
        )

        self.stop_pub = self.create_publisher(
            Bool,
            f"/{self.robot_ns}/cmd/stop",
            10,
        )

        self.start_pub = self.create_publisher(
            Bool,
            f"/{self.robot_ns}/cmd/start",
            10,
        )

        self.active_pub = self.create_publisher(
            Bool,
            (
                f"/{self.robot_ns}/"
                "safety_hold/active"
            ),
            status_qos,
        )

        self.state_pub = self.create_publisher(
            String,
            (
                f"/{self.robot_ns}/"
                "safety_hold/state"
            ),
            status_qos,
        )

        self.resume_allowed_pub = (
            self.create_publisher(
                Bool,
                (
                    f"/{self.robot_ns}/"
                    "safety_hold/resume_allowed"
                ),
                status_qos,
            )
        )

        self.event_pub = self.create_publisher(
            String,
            (
                f"/{self.robot_ns}/"
                "safety_hold/event"
            ),
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.publish_status(force=True)

        self.get_logger().info(
            "Safety hold node started"
        )

    @staticmethod
    def normalize(value):
        return str(value).strip().upper()

    def publish_event(self, event):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)

    def publish_stop(self, force=False):
        now = time.monotonic()

        if not force:
            elapsed = (
                now - self.last_stop_publish_time
            )

            if elapsed < self.stop_repeat_sec:
                return

        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

        self.last_stop_publish_time = now

    def publish_start(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)

    def obstacle_callback(self, msg):
        self.obstacle_state = self.normalize(
            msg.data
        )

        if self.obstacle_state == "CLEAR":
            if self.clear_since is None:
                self.clear_since = (
                    time.monotonic()
                )
        else:
            self.clear_since = None

        stop_states = {
            "STOPPED",
            "SENSOR_TIMEOUT",
            "LATCHED_STOP",
        }

        if (
            self.mission_active
            and self.obstacle_state
            in stop_states
        ):
            self.engage_hold(
                self.obstacle_state
            )

    def mission_active_callback(self, msg):
        self.mission_active = bool(msg.data)

        stop_states = {
            "STOPPED",
            "SENSOR_TIMEOUT",
            "LATCHED_STOP",
        }

        if (
            self.mission_active
            and self.obstacle_state
            in stop_states
        ):
            self.engage_hold(
                self.obstacle_state
            )

    def mission_state_callback(self, msg):
        self.mission_state = self.normalize(
            msg.data
        )

    def feature_state_callback(self, msg):
        self.feature_state = self.normalize(
            msg.data
        )

    def turn_state_callback(self, msg):
        self.turn_state = self.normalize(
            msg.data
        )

    def safety_ok_callback(self, msg):
        self.safety_ok = bool(msg.data)

    def bridge_callback(self, msg):
        self.bridge_connected = bool(
            msg.data
        )

    def engage_hold(self, reason):
        if not self.hold_active:
            self.hold_active = True
            self.hold_reason = reason

            self.held_mission_state = (
                self.mission_state
            )

            self.held_feature_state = (
                self.feature_state
            )

            self.held_turn_state = (
                self.turn_state
            )

            self.publish_stop(force=True)

            event = (
                f"HOLD_LATCHED_{reason}"
                f"_MISSION_{self.held_mission_state}"
                f"_FEATURE_{self.held_feature_state}"
                f"_TURN_{self.held_turn_state}"
            )

            self.publish_event(event)

            self.get_logger().warn(
                event
            )

        else:
            self.publish_stop()

        self.publish_status(force=True)

    def clear_is_stable(self):
        if self.obstacle_state != "CLEAR":
            return False

        if self.clear_since is None:
            return False

        return (
            time.monotonic()
            - self.clear_since
            >= self.clear_stable_sec
        )

    def resume_allowed(self):
        if not self.hold_active:
            return False, "NO_ACTIVE_HOLD"

        if not self.mission_active:
            return False, "MISSION_NOT_ACTIVE"

        if not self.clear_is_stable():
            return (
                False,
                "OBSTACLE_NOT_STABLY_CLEAR",
            )

        if not self.bridge_connected:
            return (
                False,
                "BRIDGE_NOT_CONNECTED",
            )

        if not self.safety_ok:
            return (
                False,
                "ROBOT_SAFETY_NOT_OK",
            )

        if (
            self.held_mission_state
            != "RUNNING_TO_NEXT_NODE"
        ):
            return (
                False,
                (
                    "HELD_MISSION_STATE_"
                    "REQUIRES_MANUAL_RECOVERY"
                ),
            )

        if self.held_feature_state != "NORMAL":
            return (
                False,
                (
                    "HELD_FEATURE_STATE_"
                    "REQUIRES_MANUAL_RECOVERY"
                ),
            )

        if self.held_turn_state not in {
            "IDLE",
            "DONE",
        }:
            return (
                False,
                (
                    "HELD_TURN_STATE_"
                    "REQUIRES_MANUAL_RECOVERY"
                ),
            )

        if (
            self.mission_state
            != "RUNNING_TO_NEXT_NODE"
        ):
            return (
                False,
                "CURRENT_MISSION_STATE_CHANGED",
            )

        if self.feature_state != "NORMAL":
            return (
                False,
                "CURRENT_FEATURE_STATE_CHANGED",
            )

        if self.turn_state not in {
            "IDLE",
            "DONE",
        }:
            return (
                False,
                "CURRENT_TURN_STATE_UNSAFE",
            )

        return (
            True,
            "LINE_FOLLOW_RESUME_ALLOWED",
        )

    def resume_callback(self, msg):
        if not msg.data:
            return

        allowed, reason = (
            self.resume_allowed()
        )

        if not allowed:
            event = (
                f"RESUME_REJECTED_{reason}"
            )

            self.publish_event(event)
            self.get_logger().warn(event)
            return

        self.hold_active = False
        self.hold_reason = "NONE"

        self.publish_start()

        self.publish_event(
            "RESUME_ACCEPTED_LINE_FOLLOW"
        )

        self.clear_held_states()
        self.publish_status(force=True)

    def reset_callback(self, msg):
        if not msg.data:
            return

        if not self.hold_active:
            return

        if self.mission_active:
            self.publish_event(
                "RESET_REJECTED_MISSION_ACTIVE"
            )
            return

        if not self.clear_is_stable():
            self.publish_event(
                (
                    "RESET_REJECTED_"
                    "OBSTACLE_NOT_CLEAR"
                )
            )
            return

        if (
            not self.bridge_connected
            or not self.safety_ok
        ):
            self.publish_event(
                (
                    "RESET_REJECTED_"
                    "ROBOT_NOT_HEALTHY"
                )
            )
            return

        self.clear_hold_without_start(
            "RESET_ACCEPTED"
        )

    def clear_held_states(self):
        self.held_mission_state = "UNKNOWN"
        self.held_feature_state = "UNKNOWN"
        self.held_turn_state = "UNKNOWN"

    def clear_hold_without_start(self, event):
        self.hold_active = False
        self.hold_reason = "NONE"

        self.clear_held_states()
        self.publish_event(event)
        self.publish_status(force=True)

    def hold_state(self):
        if not self.hold_active:
            return "CLEAR"

        if (
            self.obstacle_state
            == "SENSOR_TIMEOUT"
        ):
            return "HOLD_SENSOR_TIMEOUT"

        if self.obstacle_state in {
            "STOPPED",
            "LATCHED_STOP",
        }:
            return "HOLD_OBSTACLE_PRESENT"

        if not self.clear_is_stable():
            return (
                "HOLD_WAITING_FOR_STABLE_CLEAR"
            )

        allowed, _ = self.resume_allowed()

        if allowed:
            return "HOLD_READY_TO_RESUME"

        return (
            "HOLD_MANUAL_RECOVERY_REQUIRED"
        )

    def publish_status(self, force=False):
        now = time.monotonic()

        if not force:
            elapsed = (
                now - self.last_status_publish_time
            )

            if elapsed < 0.5:
                return

        active_msg = Bool()
        active_msg.data = self.hold_active
        self.active_pub.publish(active_msg)

        state_msg = String()
        state_msg.data = self.hold_state()
        self.state_pub.publish(state_msg)

        allowed, _ = self.resume_allowed()

        allowed_msg = Bool()
        allowed_msg.data = allowed
        self.resume_allowed_pub.publish(
            allowed_msg
        )

        self.last_status_publish_time = now

    def timer_callback(self):
        if self.hold_active:
            self.publish_stop()

            if (
                self.auto_clear_when_mission_inactive
                and not self.mission_active
                and self.clear_is_stable()
                and self.bridge_connected
                and self.safety_ok
            ):
                self.clear_hold_without_start(
                    "AUTO_CLEAR_MISSION_INACTIVE"
                )
                return

        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    node = SafetyHoldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
