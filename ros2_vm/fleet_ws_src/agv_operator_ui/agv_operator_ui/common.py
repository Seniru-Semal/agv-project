#!/usr/bin/env python3

from __future__ import annotations

import json
from typing import Any, Dict, List

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OperatorRosNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.delivery_state: Dict[str, Any] = {}
        self.delivery_tasks: List[Dict[str, Any]] = []
        self.delivery_events: List[str] = []
        self.fleet_state: Dict[str, Any] = {}
        self.fleet_events: List[str] = []

        self.request_pub = self.create_publisher(
            String,
            "/workbench/request_delivery",
            10,
        )
        self.loaded_pub = self.create_publisher(
            String,
            "/stores/confirm_loaded",
            10,
        )
        self.received_pub = self.create_publisher(
            String,
            "/workbench/confirm_received",
            10,
        )
        self.cancel_pub = self.create_publisher(
            String,
            "/delivery/cancel_task",
            10,
        )
        self.fleet_resume_pub = self.create_publisher(
            String,
            "/fleet/resume",
            10,
        )
        self.fleet_reset_pub = self.create_publisher(
            String,
            "/fleet/reset",
            10,
        )
        self.fleet_clear_pub = self.create_publisher(
            String,
            "/fleet/clear_reservations",
            10,
        )
        self.fleet_set_node_pub = self.create_publisher(
            String,
            "/fleet/set_current_node",
            10,
        )

        self.create_subscription(String, "/delivery/state", self.delivery_state_cb, 10)
        self.create_subscription(String, "/delivery/tasks", self.delivery_tasks_cb, 10)
        self.create_subscription(String, "/delivery/events", self.delivery_event_cb, 50)
        self.create_subscription(String, "/fleet/state", self.fleet_state_cb, 10)
        self.create_subscription(String, "/fleet/event", self.fleet_event_cb, 50)

    @staticmethod
    def parse_json(text: str) -> Any:
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return None

    @staticmethod
    def publish_json(publisher: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def delivery_state_cb(self, msg: String) -> None:
        value = self.parse_json(msg.data)
        if isinstance(value, dict):
            self.delivery_state = value
            tasks = value.get("tasks", [])
            if isinstance(tasks, list):
                self.delivery_tasks = [
                    task for task in tasks if isinstance(task, dict)
                ]

    def delivery_tasks_cb(self, msg: String) -> None:
        value = self.parse_json(msg.data)
        if isinstance(value, list):
            self.delivery_tasks = [
                task for task in value if isinstance(task, dict)
            ]

    def delivery_event_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self.delivery_events.append(text)
            self.delivery_events = self.delivery_events[-300:]

    def fleet_state_cb(self, msg: String) -> None:
        value = self.parse_json(msg.data)
        if isinstance(value, dict):
            self.fleet_state = value

    def fleet_event_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self.fleet_events.append(text)
            self.fleet_events = self.fleet_events[-300:]

    def request_delivery(
        self,
        workbench: str,
        item: str,
        priority: int,
    ) -> None:
        self.publish_json(
            self.request_pub,
            {
                "workbench": workbench,
                "item": item,
                "priority": int(priority),
            },
        )

    def confirm_loaded(self, task_id: str, robot: str) -> None:
        payload = {"task_id": task_id}
        if robot:
            payload["robot"] = robot
        self.publish_json(self.loaded_pub, payload)

    def confirm_received(self, task_id: str, workbench: str) -> None:
        self.publish_json(
            self.received_pub,
            {
                "task_id": task_id,
                "workbench": workbench,
            },
        )

    def cancel_task(self, task_id: str) -> None:
        self.publish_json(self.cancel_pub, {"task_id": task_id})

    def resume_robot(self, robot: str) -> None:
        self.publish_json(self.fleet_resume_pub, {"robot": robot})

    def reset_robot(self, robot: str) -> None:
        self.publish_json(self.fleet_reset_pub, {"robot": robot})

    def set_robot_current_node(self, robot: str, node: str) -> None:
        self.publish_json(
            self.fleet_set_node_pub,
            {
                "robot": robot,
                "node": node,
            },
        )

    def clear_robot_reservations(self, robot: str, confirmed_node: str) -> None:
        self.publish_json(
            self.fleet_clear_pub,
            {
                "robot": robot,
                "confirmed_node": confirmed_node,
            },
        )


def make_spin_timer(node: Node, interval_ms: int = 30) -> QTimer:
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(interval_ms)
    return timer


def table_selected_row_payload(table: QTableWidget) -> Dict[str, str]:
    row = table.currentRow()
    if row < 0:
        return {}

    payload: Dict[str, str] = {}
    for column in range(table.columnCount()):
        header = table.horizontalHeaderItem(column)
        item = table.item(row, column)
        if header is not None and item is not None:
            payload[header.text()] = item.text()

    return payload


def set_table_rows(
    table: QTableWidget,
    columns: List[str],
    rows: List[Dict[str, Any]],
) -> None:
    selected_task_id = ""
    selected = table_selected_row_payload(table)
    if selected:
        selected_task_id = selected.get("task_id", "")

    table.setColumnCount(len(columns))
    table.setHorizontalHeaderLabels(columns)
    table.setRowCount(len(rows))

    new_selected_row = -1

    for row_index, row in enumerate(rows):
        for column_index, column in enumerate(columns):
            value = row.get(column, "")
            item = QTableWidgetItem(str(value))
            table.setItem(row_index, column_index, item)

        if row.get("task_id") == selected_task_id:
            new_selected_row = row_index

    table.resizeColumnsToContents()

    if new_selected_row >= 0:
        table.selectRow(new_selected_row)
