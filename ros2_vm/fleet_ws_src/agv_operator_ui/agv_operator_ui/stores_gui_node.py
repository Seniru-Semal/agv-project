#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
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
    "message",
]


class StoresWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode) -> None:
        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle("AGV Stores")
        self.resize(860, 520)

        central = QWidget()
        layout = QVBoxLayout(central)

        title = QLabel("Stores Loading Station")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(title)

        self.task_table = QTableWidget()
        layout.addWidget(self.task_table, 1)

        button_row = QHBoxLayout()
        loaded_button = QPushButton("Confirm Loaded")
        loaded_button.clicked.connect(self.confirm_loaded)
        button_row.addWidget(loaded_button)
        button_row.addStretch()
        layout.addLayout(button_row)

        self.setCentralWidget(central)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(250)

    def confirm_loaded(self) -> None:
        row = table_selected_row_payload(self.task_table)
        task_id = row.get("task_id", "")
        state = row.get("state", "")
        robot = row.get("robot", "")

        if not task_id:
            return

        if state != "WAITING_FOR_LOAD":
            QMessageBox.warning(
                self,
                "Task not ready",
                "Only confirm when the selected task is WAITING_FOR_LOAD.",
            )
            return

        self.ros_node.confirm_loaded(task_id, robot)

    def update_ui(self) -> None:
        rows: List[Dict[str, str]] = [
            task
            for task in self.ros_node.delivery_tasks
            if task.get("state") in ("GOING_TO_STORES", "WAITING_FOR_LOAD")
        ]
        set_table_rows(self.task_table, TASK_COLUMNS, rows)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    app = QApplication([])

    ros_node = OperatorRosNode("agv_stores_gui_node")
    window = StoresWindow(ros_node)
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
