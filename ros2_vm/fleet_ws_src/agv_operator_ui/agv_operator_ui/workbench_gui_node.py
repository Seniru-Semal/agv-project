#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
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
    "item",
    "target_node",
    "message",
]


class WorkbenchWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode, workbench_id: str) -> None:
        super().__init__()

        self.ros_node = ros_node
        self.workbench_id = workbench_id

        self.setWindowTitle(f"AGV Workbench - {workbench_id}")
        self.resize(820, 520)

        central = QWidget()
        layout = QVBoxLayout(central)

        title = QLabel(f"Workbench: {workbench_id}")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(title)

        request_box = QGroupBox("Request Delivery")
        request_layout = QFormLayout(request_box)

        self.item_edit = QLineEdit("default_item")
        self.priority_spin = QSpinBox()
        self.priority_spin.setRange(0, 100)
        self.priority_spin.setValue(10)

        request_layout.addRow("Item", self.item_edit)
        request_layout.addRow("Priority", self.priority_spin)

        request_button = QPushButton("Request Delivery")
        request_button.clicked.connect(self.request_delivery)
        request_layout.addRow(request_button)
        layout.addWidget(request_box)

        self.task_table = QTableWidget()
        layout.addWidget(self.task_table, 1)

        button_row = QHBoxLayout()
        receive_button = QPushButton("Confirm Received")
        receive_button.clicked.connect(self.confirm_received)
        button_row.addWidget(receive_button)
        button_row.addStretch()
        layout.addLayout(button_row)

        self.setCentralWidget(central)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(250)

    def request_delivery(self) -> None:
        item = self.item_edit.text().strip() or "default_item"
        self.ros_node.request_delivery(
            self.workbench_id,
            item,
            self.priority_spin.value(),
        )

    def confirm_received(self) -> None:
        row = table_selected_row_payload(self.task_table)
        task_id = row.get("task_id", "")
        state = row.get("state", "")

        if not task_id:
            return

        if state != "WAITING_FOR_RECEIVE":
            QMessageBox.warning(
                self,
                "Task not ready",
                "Only confirm when the selected task is WAITING_FOR_RECEIVE.",
            )
            return

        self.ros_node.confirm_received(task_id, self.workbench_id)

    def update_ui(self) -> None:
        rows: List[Dict[str, str]] = [
            task
            for task in self.ros_node.delivery_tasks
            if task.get("workbench") == self.workbench_id
        ]
        set_table_rows(self.task_table, TASK_COLUMNS, rows)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    app = QApplication([])

    ros_node = OperatorRosNode("agv_workbench_gui_node")
    ros_node.declare_parameter("workbench_id", "bench_1")
    workbench_id = str(ros_node.get_parameter("workbench_id").value).strip().lower()

    window = WorkbenchWindow(ros_node, workbench_id)
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
