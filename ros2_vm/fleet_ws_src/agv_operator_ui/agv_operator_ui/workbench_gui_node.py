#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QFormLayout,
    QGroupBox,
    QHeaderView,
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


WORKBENCH_STYLE = """
QMainWindow {
    background: #f5f7fb;
}
QGroupBox {
    font-size: 15px;
    font-weight: 700;
    border: 1px solid #cdd6e4;
    border-radius: 8px;
    margin-top: 10px;
    padding: 12px;
    background: white;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 4px;
}
QLineEdit, QSpinBox {
    min-height: 44px;
    font-size: 18px;
    padding: 6px;
}
QPushButton {
    min-height: 52px;
    border-radius: 8px;
    font-size: 17px;
    font-weight: 700;
    padding: 10px 18px;
}
QPushButton#requestButton {
    background: #1667d9;
    color: white;
}
QPushButton#requestButton:hover {
    background: #0f56ba;
}
QPushButton#receiveButton {
    background: #a8b3c2;
    color: white;
}
QPushButton#receiveButton[ready="true"] {
    background: #11823b;
}
QPushButton#receiveButton[ready="true"]:hover {
    background: #0b6b2f;
}
QTableWidget {
    background: white;
    gridline-color: #d7deea;
    font-size: 13px;
    selection-background-color: #b9d7ff;
}
QHeaderView::section {
    background: #e8eef7;
    font-weight: 700;
    padding: 6px;
    border: 0;
}
"""


class WorkbenchWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode, workbench_id: str) -> None:
        super().__init__()

        self.ros_node = ros_node
        self.workbench_id = workbench_id

        self.setWindowTitle(f"AGV Workbench - {workbench_id}")
        self.resize(980, 640)
        self.setStyleSheet(WORKBENCH_STYLE)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)

        title = QLabel(f"Workbench: {workbench_id}")
        title.setStyleSheet("font-size: 24px; font-weight: 800; color: #172033;")
        layout.addWidget(title)

        self.status_label = QLabel("Ready to request an item")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(58)
        self.status_label.setAlignment(Qt.AlignVCenter)
        self.status_label.setStyleSheet(
            "background: #e8f0fe; border-radius: 8px; padding: 12px; "
            "font-size: 18px; font-weight: 700; color: #17345c;"
        )
        layout.addWidget(self.status_label)

        request_box = QGroupBox("Request Delivery")
        request_layout = QFormLayout(request_box)
        request_layout.setLabelAlignment(Qt.AlignLeft)
        request_layout.setFormAlignment(Qt.AlignLeft)
        request_layout.setHorizontalSpacing(16)
        request_layout.setVerticalSpacing(12)

        self.item_edit = QLineEdit("default_item")
        self.priority_spin = QSpinBox()
        self.priority_spin.setRange(0, 100)
        self.priority_spin.setValue(10)

        request_layout.addRow("Item", self.item_edit)
        request_layout.addRow("Priority", self.priority_spin)

        self.request_button = QPushButton("REQUEST ITEM DELIVERY")
        self.request_button.setObjectName("requestButton")
        self.request_button.clicked.connect(self.request_delivery)
        request_layout.addRow(self.request_button)
        layout.addWidget(request_box)

        self.task_table = QTableWidget()
        self.task_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.task_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.task_table.verticalHeader().setVisible(False)
        self.task_table.horizontalHeader().setStretchLastSection(True)
        self.task_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.task_table.setAlternatingRowColors(True)
        layout.addWidget(self.task_table, 1)

        button_row = QHBoxLayout()
        self.receive_button = QPushButton("CONFIRM ITEM RECEIVED")
        self.receive_button.setObjectName("receiveButton")
        self.receive_button.setProperty("ready", "false")
        self.receive_button.setEnabled(False)
        self.receive_button.clicked.connect(self.confirm_received)
        self.receive_button.setMinimumHeight(72)
        button_row.addWidget(self.receive_button, 1)
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
        row = self.selected_or_ready_task()
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

    def selected_or_ready_task(self) -> Dict[str, str]:
        row = table_selected_row_payload(self.task_table)
        if row.get("state") == "WAITING_FOR_RECEIVE":
            return row

        for task in self.workbench_tasks():
            if task.get("state") == "WAITING_FOR_RECEIVE":
                return {key: str(task.get(key, "")) for key in TASK_COLUMNS}

        return row

    def workbench_tasks(self) -> List[Dict[str, str]]:
        rows: List[Dict[str, str]] = [
            task
            for task in self.ros_node.delivery_tasks
            if task.get("workbench") == self.workbench_id
        ]

        rows.sort(
            key=lambda task: (
                task.get("state") in ("COMPLETE", "FAULTED"),
                float(task.get("created_at", 0.0) or 0.0),
            )
        )

        return rows

    def update_ui(self) -> None:
        rows = self.workbench_tasks()
        set_table_rows(self.task_table, TASK_COLUMNS, rows)
        self.highlight_rows(rows)
        self.update_status(rows)

    def highlight_rows(self, rows: List[Dict[str, str]]) -> None:
        for row_index, row in enumerate(rows):
            state = row.get("state", "")
            color = QColor("#ffffff")

            if state == "WAITING_FOR_RECEIVE":
                color = QColor("#d8f5df")
                if self.task_table.currentRow() < 0:
                    self.task_table.selectRow(row_index)
            elif state in ("GOING_TO_STORES", "GOING_TO_WORKBENCH", "RETURNING_TO_CHARGER"):
                color = QColor("#fff3c4")
            elif state == "COMPLETE":
                color = QColor("#eef3f8")
            elif state == "FAULTED":
                color = QColor("#ffd6d6")

            for column in range(self.task_table.columnCount()):
                item = self.task_table.item(row_index, column)
                if item is not None:
                    item.setBackground(color)

    def update_status(self, rows: List[Dict[str, str]]) -> None:
        ready = any(task.get("state") == "WAITING_FOR_RECEIVE" for task in rows)
        active = [
            task
            for task in rows
            if task.get("state") not in ("COMPLETE", "FAULTED")
        ]

        self.receive_button.setEnabled(ready)
        self.receive_button.setProperty("ready", "true" if ready else "false")
        self.receive_button.style().unpolish(self.receive_button)
        self.receive_button.style().polish(self.receive_button)

        if ready:
            ready_task = next(task for task in rows if task.get("state") == "WAITING_FOR_RECEIVE")
            self.status_label.setText(
                "AGV arrived. Check the item, then press CONFIRM ITEM RECEIVED. "
                f"Task: {ready_task.get('task_id', '-')}"
            )
            self.status_label.setStyleSheet(
                "background: #d8f5df; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 800; color: #0b5126;"
            )
        elif active:
            task = active[-1]
            self.status_label.setText(
                f"Current task: {task.get('state', '-')} - {task.get('message', '-')}"
            )
            self.status_label.setStyleSheet(
                "background: #fff3c4; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 800; color: #604500;"
            )
        else:
            self.status_label.setText("Ready to request an item")
            self.status_label.setStyleSheet(
                "background: #e8f0fe; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 700; color: #17345c;"
            )


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
