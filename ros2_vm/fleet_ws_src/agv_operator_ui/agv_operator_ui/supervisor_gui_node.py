#!/usr/bin/env python3

from __future__ import annotations

from typing import Any, Dict, List, Optional

import rclpy

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .common import (
    OperatorRosNode,
    make_spin_timer,
    set_table_rows,
    table_selected_row_payload,
)


TASK_COLUMNS = [
    "task_id",
    "state",
    "robot",
    "workbench",
    "item",
    "target_node",
    "message",
]

ROBOT_COLUMNS = [
    "robot",
    "status",
    "current_node",
    "destination",
    "bridge",
    "safety",
    "obstacle",
]


class SupervisorWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode) -> None:
        super().__init__()

        self.ros_node = ros_node
        self.setWindowTitle("AGV Supervisor")
        self.resize(1100, 700)

        central = QWidget()
        layout = QVBoxLayout(central)

        title_row = QHBoxLayout()
        title = QLabel("AGV Supervisor")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        title_row.addWidget(title)
        title_row.addStretch()

        self.status_label = QLabel("Waiting for delivery state")
        title_row.addWidget(self.status_label)
        layout.addLayout(title_row)

        splitter = QSplitter()

        task_box = QGroupBox("Delivery Tasks")
        task_layout = QVBoxLayout(task_box)
        self.task_table = QTableWidget()
        task_layout.addWidget(self.task_table)

        task_buttons = QHBoxLayout()
        cancel_button = QPushButton("Cancel Selected Task")
        cancel_button.clicked.connect(self.cancel_selected_task)
        task_buttons.addWidget(cancel_button)
        task_buttons.addStretch()
        task_layout.addLayout(task_buttons)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        robot_box = QGroupBox("Robot State")
        robot_layout = QVBoxLayout(robot_box)
        self.robot_table = QTableWidget()
        robot_layout.addWidget(self.robot_table)
        right_layout.addWidget(robot_box)

        event_box = QGroupBox("Events")
        event_layout = QVBoxLayout(event_box)
        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        event_layout.addWidget(self.event_log)
        right_layout.addWidget(event_box)

        splitter.addWidget(task_box)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(splitter, 1)
        self.setCentralWidget(central)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(250)

    def cancel_selected_task(self) -> None:
        row = table_selected_row_payload(self.task_table)
        task_id = row.get("task_id", "")
        if not task_id:
            return

        result = QMessageBox.question(
            self,
            "Cancel task",
            f"Cancel task {task_id}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if result == QMessageBox.Yes:
            self.ros_node.cancel_task(task_id)

    def update_ui(self) -> None:
        tasks = self.ros_node.delivery_tasks
        set_table_rows(self.task_table, TASK_COLUMNS, tasks)

        robot_rows: List[Dict[str, Any]] = []
        robots = self.ros_node.fleet_state.get("robots", {})
        if isinstance(robots, dict):
            for robot, data in sorted(robots.items()):
                if not isinstance(data, dict):
                    continue
                safety = "OK" if data.get("safety_ok") else "NOT READY"
                if data.get("safety_hold"):
                    safety += " / HOLD"
                robot_rows.append(
                    {
                        "robot": robot,
                        "status": data.get("status", ""),
                        "current_node": data.get("current_node", ""),
                        "destination": data.get("destination", ""),
                        "bridge": "ONLINE" if data.get("bridge_connected") else "OFFLINE",
                        "safety": safety,
                        "obstacle": data.get("obstacle_state", ""),
                    }
                )

        set_table_rows(self.robot_table, ROBOT_COLUMNS, robot_rows)

        active_count = len(
            [
                task
                for task in tasks
                if task.get("state") not in ("COMPLETE", "FAULTED")
            ]
        )
        self.status_label.setText(f"Active tasks: {active_count}")

        events = self.ros_node.delivery_events[-80:] + self.ros_node.fleet_events[-80:]
        self.event_log.setPlainText("\n".join(events[-120:]))
        self.event_log.verticalScrollBar().setValue(
            self.event_log.verticalScrollBar().maximum()
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    app = QApplication([])

    ros_node = OperatorRosNode("agv_supervisor_gui_node")
    window = SupervisorWindow(ros_node)
    window.show()

    spin_timer = make_spin_timer(ros_node)

    try:
        app.exec_()
    finally:
        spin_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
