#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="agv_delivery_manager",
                executable="delivery_manager_node",
                name="delivery_manager_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "dispatch_retry_sec": 8.0,
                        "delivery_state_publish_period_sec": 1.0,
                        "publish_delivery_tasks_topic": False,
                        "state_event_history_limit": 0,
                        "fleet_state_stale_sec": 2.5,
                    }
                ],
            ),
        ]
    )
