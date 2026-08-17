#!/usr/bin/env python3

import time

import rclpy
from std_msgs.msg import Bool, String

from .dock_safe_fleet_manager_node import DockSafeFleetManagerNode


class AutoResumeDockSafeFleetManagerNode(DockSafeFleetManagerNode):

    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter(
            "auto_resume_enabled",
            True,
        )

        self.declare_parameter(
            "auto_resume_check_period_sec",
            0.25,
        )

        self.declare_parameter(
            "auto_resume_retry_sec",
            1.0,
        )

        self.auto_resume_enabled = bool(
            self.get_parameter(
                "auto_resume_enabled"
            ).value
        )

        self.auto_resume_check_period_sec = max(
            0.1,
            float(
                self.get_parameter(
                    "auto_resume_check_period_sec"
                ).value
            ),
        )

        self.auto_resume_retry_sec = max(
            0.5,
            float(
                self.get_parameter(
                    "auto_resume_retry_sec"
                ).value
            ),
        )

        self.last_resume_request_time = {
            robot_name: 0.0
            for robot_name in self.robots
        }

        self.create_timer(
            self.auto_resume_check_period_sec,
            self.auto_resume_timer_callback,
        )

        self.publish_event(
            "AUTO_RESUME_MANAGER_STARTED "
            f"enabled={self.auto_resume_enabled} "
            f"check_period_sec="
            f"{self.auto_resume_check_period_sec} "
            f"retry_sec="
            f"{self.auto_resume_retry_sec}"
        )

    def safety_hold_callback(
        self,
        robot_name: str,
        msg: Bool,
    ) -> None:
        robot = self.robots[
            robot_name
        ]

        was_active = bool(
            robot.safety_hold
        )

        super().safety_hold_callback(
            robot_name,
            msg,
        )

        is_active = bool(
            robot.safety_hold
        )

        if (
            is_active
            and not was_active
        ):
            self.last_resume_request_time[
                robot_name
            ] = 0.0

            self.publish_event(
                "SAFETY_HOLD_DETECTED "
                f"robot={robot_name}"
            )

        if (
            was_active
            and not is_active
        ):
            self.last_resume_request_time[
                robot_name
            ] = 0.0

            if robot.mission_active:
                robot.status = "MOVING"

                if (
                    robot.waiting_reason
                    == "SAFETY_HOLD"
                ):
                    robot.waiting_reason = ""

            self.publish_event(
                "SAFETY_HOLD_CLEARED "
                f"robot={robot_name}"
            )

    def get_resume_robot_name(
        self,
        msg: String,
    ) -> str:
        payload = self.parse_json(
            msg.data
        )

        if payload is not None:
            return str(
                payload.get(
                    "robot",
                    "",
                )
            ).strip()

        return msg.data.strip()

    def resume_ready(
        self,
        robot_name: str,
    ) -> tuple[bool, str]:
        if robot_name not in self.robots:
            return (
                False,
                "unknown_robot",
            )

        robot = self.robots[
            robot_name
        ]

        if (
            not robot.mission_id
            or not robot.mission_active
        ):
            return (
                False,
                "no_active_mission",
            )

        if not robot.safety_hold:
            return (
                False,
                "no_active_hold",
            )

        if not robot.resume_allowed:
            return (
                False,
                "local_guard_not_ready",
            )

        if (
            robot.released_until_index
            <= robot.current_index
        ):
            return (
                False,
                "no_released_edge",
            )

        return (
            True,
            "",
        )

    def send_resume_request(
        self,
        robot_name: str,
        source: str,
    ) -> None:
        self.publish_bool(
            self.robot_publishers[
                robot_name
            ]["resume"],
            True,
        )

        self.last_resume_request_time[
            robot_name
        ] = time.monotonic()

        self.publish_event(
            "RESUME_REQUEST_SENT "
            f"robot={robot_name} "
            f"source={source}"
        )

    def resume_callback(
        self,
        msg: String,
    ) -> None:
        robot_name = (
            self.get_resume_robot_name(
                msg
            )
        )

        ready, reason = (
            self.resume_ready(
                robot_name
            )
        )

        if not ready:
            self.publish_event(
                "RESUME_REJECTED "
                f"robot={robot_name} "
                f"reason={reason}"
            )
            return

        self.send_resume_request(
            robot_name,
            source="manual",
        )

    def auto_resume_timer_callback(
        self,
    ) -> None:
        if not self.auto_resume_enabled:
            return

        now = time.monotonic()

        for robot_name in self.robots:
            ready, _ = (
                self.resume_ready(
                    robot_name
                )
            )

            if not ready:
                continue

            last_request = (
                self.last_resume_request_time[
                    robot_name
                ]
            )

            if (
                last_request > 0.0
                and (
                    now - last_request
                    < self.auto_resume_retry_sec
                )
            ):
                continue

            self.send_resume_request(
                robot_name,
                source="automatic",
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = (
        AutoResumeDockSafeFleetManagerNode()
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
