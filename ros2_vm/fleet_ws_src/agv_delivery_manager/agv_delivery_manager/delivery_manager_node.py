#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import time
import uuid

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional

import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


REQUESTED = "REQUESTED"
ASSIGNED = "ASSIGNED"
GOING_TO_STORES = "GOING_TO_STORES"
WAITING_FOR_LOAD = "WAITING_FOR_LOAD"
GOING_TO_WORKBENCH = "GOING_TO_WORKBENCH"
WAITING_FOR_RECEIVE = "WAITING_FOR_RECEIVE"
RETURNING_TO_CHARGER = "RETURNING_TO_CHARGER"
COMPLETE = "COMPLETE"
FAULTED = "FAULTED"

ACTIVE_TASK_STATES = {
    REQUESTED,
    ASSIGNED,
    GOING_TO_STORES,
    WAITING_FOR_LOAD,
    GOING_TO_WORKBENCH,
    WAITING_FOR_RECEIVE,
    RETURNING_TO_CHARGER,
}


@dataclass
class DeliveryTask:
    task_id: str
    workbench: str
    item: str
    priority: int
    state: str = REQUESTED
    robot: str = ""
    target_node: str = ""
    active_leg: str = ""
    fleet_mission_id: str = ""
    message: str = ""
    created_at: float = 0.0
    updated_at: float = 0.0


class DeliveryManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("delivery_manager_node")

        default_config = os.path.join(
            get_package_share_directory("agv_delivery_manager"),
            "config",
            "delivery_config.json",
        )

        self.declare_parameter("config_file", default_config)
        self.declare_parameter("dispatch_retry_sec", 3.0)

        self.config_file = os.path.expanduser(
            str(self.get_parameter("config_file").value)
        )
        self.dispatch_retry_sec = max(
            1.0,
            float(self.get_parameter("dispatch_retry_sec").value),
        )

        self.config = self.load_config(self.config_file)
        self.stores_node = str(self.config.get("stores_node", "stores")).lower()
        self.workbenches = [
            str(value).strip().lower()
            for value in self.config.get("workbenches", [])
            if str(value).strip()
        ]
        self.home_nodes = {
            str(robot).strip(): str(node).strip().lower()
            for robot, node in self.config.get("home_nodes", {}).items()
            if str(robot).strip() and str(node).strip()
        }
        self.default_priority = int(self.config.get("default_priority", 10))

        self.tasks: Dict[str, DeliveryTask] = {}
        self.events: List[str] = []
        self.fleet_state: Dict[str, Any] = {}
        self.last_dispatch_at: Dict[str, float] = {}

        self.dispatch_pub = self.create_publisher(String, "/fleet/dispatch", 10)
        self.cancel_pub = self.create_publisher(String, "/fleet/cancel", 10)
        self.tasks_pub = self.create_publisher(String, "/delivery/tasks", 10)
        self.state_pub = self.create_publisher(String, "/delivery/state", 10)
        self.events_pub = self.create_publisher(String, "/delivery/events", 20)

        self.create_subscription(
            String,
            "/workbench/request_delivery",
            self.request_delivery_callback,
            10,
        )
        self.create_subscription(
            String,
            "/stores/confirm_loaded",
            self.confirm_loaded_callback,
            10,
        )
        self.create_subscription(
            String,
            "/workbench/confirm_received",
            self.confirm_received_callback,
            10,
        )
        self.create_subscription(
            String,
            "/delivery/cancel_task",
            self.cancel_task_callback,
            10,
        )
        self.create_subscription(String, "/fleet/state", self.fleet_state_callback, 10)
        self.create_subscription(String, "/fleet/event", self.fleet_event_callback, 20)

        self.create_timer(0.5, self.timer_callback)

        self.publish_event(
            "DELIVERY_MANAGER_STARTED "
            f"stores={self.stores_node} benches={self.workbenches}"
        )

    @staticmethod
    def parse_json(text: str) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None

    @staticmethod
    def publish_json(publisher: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    @staticmethod
    def load_config(path: str) -> Dict[str, Any]:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                value = json.load(handle)
        except OSError:
            return {}

        return value if isinstance(value, dict) else {}

    def publish_event(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        event = f"{stamp} {text}"
        self.events.append(event)
        self.events = self.events[-300:]

        msg = String()
        msg.data = event
        self.events_pub.publish(msg)
        self.get_logger().info(text)

    def fleet_state_callback(self, msg: String) -> None:
        value = self.parse_json(msg.data)
        if value is not None:
            self.fleet_state = value

    def fleet_event_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if text.startswith("DISPATCH_REJECTED"):
            self.publish_event(f"FLEET_{text}")

    def active_task_for_workbench(self, workbench: str) -> Optional[DeliveryTask]:
        candidates = [
            task
            for task in self.tasks.values()
            if task.workbench == workbench and task.state in ACTIVE_TASK_STATES
        ]
        candidates.sort(key=lambda task: task.created_at)
        return candidates[0] if candidates else None

    def active_task_robot_names(self) -> set[str]:
        return {
            task.robot
            for task in self.tasks.values()
            if task.robot and task.state in ACTIVE_TASK_STATES
        }

    def request_delivery_callback(self, msg: String) -> None:
        payload = self.parse_json(msg.data)
        if payload is None:
            self.publish_event("REQUEST_REJECTED invalid_json")
            return

        workbench = str(payload.get("workbench", "")).strip().lower()
        if not workbench:
            self.publish_event("REQUEST_REJECTED missing_workbench")
            return

        if self.workbenches and workbench not in self.workbenches:
            self.publish_event(f"REQUEST_REJECTED unknown_workbench={workbench}")
            return

        active_task = self.active_task_for_workbench(workbench)
        if active_task is not None:
            self.publish_event(
                "REQUEST_REJECTED active_task_for_workbench="
                f"{workbench} task={active_task.task_id} state={active_task.state}"
            )
            return

        item = str(payload.get("item", "default_item")).strip() or "default_item"

        try:
            priority = int(payload.get("priority", self.default_priority))
        except (TypeError, ValueError):
            priority = self.default_priority

        robot = str(payload.get("robot", "")).strip()
        now = time.time()
        task_id = str(payload.get("task_id", "")).strip()
        if not task_id:
            task_id = "task_" + uuid.uuid4().hex[:8]

        if task_id in self.tasks:
            self.publish_event(f"REQUEST_REJECTED duplicate_task={task_id}")
            return

        self.tasks[task_id] = DeliveryTask(
            task_id=task_id,
            workbench=workbench,
            item=item,
            priority=priority,
            robot=robot,
            created_at=now,
            updated_at=now,
            message="Waiting for AGV assignment",
        )

        self.publish_event(
            f"TASK_REQUESTED task={task_id} workbench={workbench} item={item}"
        )

    def confirm_loaded_callback(self, msg: String) -> None:
        payload = self.parse_json(msg.data)
        if payload is None:
            self.publish_event("LOAD_CONFIRM_REJECTED invalid_json")
            return

        task = self.find_task(
            payload=payload,
            state=WAITING_FOR_LOAD,
            robot=str(payload.get("robot", "")).strip(),
        )
        if task is None:
            self.publish_event("LOAD_CONFIRM_REJECTED no_waiting_task")
            return

        task.message = "Stores confirmed load"
        self.publish_event(f"TASK_LOADED task={task.task_id} robot={task.robot}")
        self.start_leg(
            task,
            state=GOING_TO_WORKBENCH,
            target=task.workbench,
            leg="to_workbench",
            message=f"Going to {task.workbench}",
        )

    def confirm_received_callback(self, msg: String) -> None:
        payload = self.parse_json(msg.data)
        if payload is None:
            self.publish_event("RECEIVE_CONFIRM_REJECTED invalid_json")
            return

        task = self.find_task(
            payload=payload,
            state=WAITING_FOR_RECEIVE,
            workbench=str(payload.get("workbench", "")).strip().lower(),
        )
        if task is None:
            self.publish_event("RECEIVE_CONFIRM_REJECTED no_waiting_task")
            return

        home = self.home_nodes.get(task.robot, "")
        if not home:
            task.state = FAULTED
            task.message = f"No home node configured for {task.robot}"
            task.updated_at = time.time()
            self.publish_event(f"TASK_FAULTED task={task.task_id} no_home_node")
            return

        self.publish_event(f"TASK_RECEIVED task={task.task_id} workbench={task.workbench}")
        self.start_leg(
            task,
            state=RETURNING_TO_CHARGER,
            target=home,
            leg="to_home",
            message=f"Returning to {home}",
        )

    def cancel_task_callback(self, msg: String) -> None:
        payload = self.parse_json(msg.data)
        if payload is None:
            self.publish_event("CANCEL_REJECTED invalid_json")
            return

        task_id = str(payload.get("task_id", "")).strip()
        task = self.tasks.get(task_id)
        if task is None:
            self.publish_event(f"CANCEL_REJECTED unknown_task={task_id}")
            return

        task.state = FAULTED
        task.message = "Cancelled by operator"
        task.updated_at = time.time()

        if task.robot:
            self.publish_json(self.cancel_pub, {"robot": task.robot})

        self.publish_event(f"TASK_CANCELLED task={task.task_id}")

    def find_task(
        self,
        payload: Dict[str, Any],
        state: str,
        robot: str = "",
        workbench: str = "",
    ) -> Optional[DeliveryTask]:
        task_id = str(payload.get("task_id", "")).strip()
        if task_id:
            task = self.tasks.get(task_id)
            return task if task and task.state == state else None

        candidates = [
            task
            for task in self.tasks.values()
            if task.state == state
            and (not robot or task.robot == robot)
            and (not workbench or task.workbench == workbench)
        ]

        candidates.sort(key=lambda task: task.created_at)
        return candidates[0] if candidates else None

    def timer_callback(self) -> None:
        self.progress_tasks()
        self.publish_state()

    def progress_tasks(self) -> None:
        for task in list(self.tasks.values()):
            if task.state in (COMPLETE, FAULTED):
                continue

            if task.state == REQUESTED:
                self.assign_and_start(task)

            elif task.state == GOING_TO_STORES:
                if self.robot_arrived(task.robot, self.stores_node):
                    task.state = WAITING_FOR_LOAD
                    task.message = "Waiting for stores to confirm loading"
                    task.updated_at = time.time()
                    self.publish_event(f"TASK_WAITING_FOR_LOAD task={task.task_id}")
                else:
                    self.retry_dispatch_if_needed(task)

            elif task.state == GOING_TO_WORKBENCH:
                if self.robot_arrived(task.robot, task.workbench):
                    task.state = WAITING_FOR_RECEIVE
                    task.message = "Waiting for workbench to confirm receive"
                    task.updated_at = time.time()
                    self.publish_event(f"TASK_WAITING_FOR_RECEIVE task={task.task_id}")
                else:
                    self.retry_dispatch_if_needed(task)

            elif task.state == RETURNING_TO_CHARGER:
                home = self.home_nodes.get(task.robot, "")
                if home and self.robot_arrived(task.robot, home):
                    task.state = COMPLETE
                    task.message = "Delivery complete"
                    task.updated_at = time.time()
                    self.publish_event(f"TASK_COMPLETE task={task.task_id}")
                else:
                    self.retry_dispatch_if_needed(task)

    def assign_and_start(self, task: DeliveryTask) -> None:
        if not task.robot:
            task.robot = self.select_available_robot()

        if not task.robot:
            task.message = "Waiting for available AGV"
            task.updated_at = time.time()
            return

        task.state = ASSIGNED
        task.message = f"Assigned to {task.robot}"
        task.updated_at = time.time()
        self.publish_event(f"TASK_ASSIGNED task={task.task_id} robot={task.robot}")

        self.start_leg(
            task,
            state=GOING_TO_STORES,
            target=self.stores_node,
            leg="to_stores",
            message=f"Going to {self.stores_node}",
        )

    def start_leg(
        self,
        task: DeliveryTask,
        state: str,
        target: str,
        leg: str,
        message: str,
    ) -> None:
        task.state = state
        task.target_node = target
        task.active_leg = leg
        task.fleet_mission_id = f"{task.task_id}_{leg}"
        task.message = message
        task.updated_at = time.time()
        self.publish_dispatch(task)

    def publish_dispatch(self, task: DeliveryTask) -> None:
        if not task.robot or not task.target_node:
            return

        payload = {
            "robot": task.robot,
            "destination": task.target_node,
            "priority": task.priority,
            "mission_id": task.fleet_mission_id,
        }

        self.publish_json(self.dispatch_pub, payload)
        self.last_dispatch_at[task.task_id] = time.time()

        self.publish_event(
            "TASK_DISPATCH_SENT "
            f"task={task.task_id} robot={task.robot} target={task.target_node}"
        )

    def retry_dispatch_if_needed(self, task: DeliveryTask) -> None:
        last_sent = self.last_dispatch_at.get(task.task_id, 0.0)
        if time.time() - last_sent < self.dispatch_retry_sec:
            return

        robot = self.robot_data(task.robot)
        if not robot:
            return

        if robot.get("destination") or robot.get("mission_id") or robot.get("mission_active"):
            return

        self.publish_dispatch(task)

    def select_available_robot(self) -> str:
        robots = self.fleet_state.get("robots", {})
        if not isinstance(robots, dict):
            return ""

        reserved_robots = self.active_task_robot_names()

        candidates = []
        for robot_name, data in robots.items():
            if str(robot_name) in reserved_robots:
                continue

            if not isinstance(data, dict):
                continue

            if not data.get("bridge_connected"):
                continue

            if not data.get("safety_ok") or data.get("safety_hold"):
                continue

            if data.get("mission_active") or data.get("mission_id") or data.get("destination"):
                continue

            candidates.append((int(data.get("priority", 10)), str(robot_name)))

        candidates.sort(reverse=True)
        return candidates[0][1] if candidates else ""

    def robot_data(self, robot: str) -> Dict[str, Any]:
        robots = self.fleet_state.get("robots", {})
        if not isinstance(robots, dict):
            return {}

        data = robots.get(robot, {})
        return data if isinstance(data, dict) else {}

    def robot_arrived(self, robot: str, node: str) -> bool:
        data = self.robot_data(robot)
        if not data:
            return False

        current_node = str(data.get("current_node", "")).strip().lower()
        mission_active = bool(data.get("mission_active"))

        return current_node == node and not mission_active

    def publish_state(self) -> None:
        tasks = [
            asdict(task)
            for task in sorted(
                self.tasks.values(),
                key=lambda item: item.created_at,
            )
        ]

        state = {
            "stamp": time.time(),
            "stores_node": self.stores_node,
            "workbenches": self.workbenches,
            "home_nodes": self.home_nodes,
            "tasks": tasks,
            "events": self.events[-100:],
        }

        msg = String()
        msg.data = json.dumps(tasks, separators=(",", ":"))
        self.tasks_pub.publish(msg)

        self.publish_json(self.state_pub, state)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DeliveryManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
