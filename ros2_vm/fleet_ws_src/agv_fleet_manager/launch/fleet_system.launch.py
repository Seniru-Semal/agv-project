#!/usr/bin/env python3

from launch import (
    LaunchDescription,
)

from launch_ros.actions import (
    Node,
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=(
                    "agv_fleet_manager"
                ),

                executable=(
                    "planned_route_"
                    "fleet_manager_node"
                ),

                name=(
                    "fleet_manager_node"
                ),

                output="screen",

                parameters=[
                    {
                        "auto_resume_enabled":
                            True,

                        "auto_resume_check_period_sec":
                            0.25,

                        "auto_resume_retry_sec":
                            1.0,

                        "exact_path_retry_sec":
                            1.0,

                        "controlled_retreat_delay_sec":
                            2.0,

                        "allow_idle_egress_borrowing":
                            True,
                    }
                ],
            ),

            Node(
                package=(
                    "agv_fleet_manager"
                ),

                executable=(
                    "fleet_hmi_node"
                ),

                name=(
                    "fleet_hmi_node"
                ),

                output="screen",
            ),
        ]
    )
