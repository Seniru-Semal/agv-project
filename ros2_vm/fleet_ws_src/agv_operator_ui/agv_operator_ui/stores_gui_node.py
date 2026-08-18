#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QGroupBox,
    QHeaderView,
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


STORES_STYLE = """
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
QPushButton {
    min-height: 64px;
    border-radius: 8px;
    font-size: 18px;
    font-weight: 800;
    padding: 12px 18px;
}
QPushButton#loadedButton {
    background: #a8b3c2;
    color: white;
}
QPushButton#loadedButton[ready="true"] {
    background: #11823b;
}
QPushButton#loadedButton[ready="true"]:hover {
    background: #0b6b2f;
}
QTableWidget {
    background: white;
    gridline-color: #d7deea;
    font-size: 14px;
    selection-background-color: #b9d7ff;
}
QHeaderView::section {
    background: #e8eef7;
    font-weight: 700;
    padding: 6px;
    border: 0;
}
"""


class StoresWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode) -> None:
        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle("AGV Stores")
        self.resize(980, 640)
        self.setStyleSheet(STORES_STYLE)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)

        title = QLabel("Stores Loading Station")
        title.setStyleSheet("font-size: 24px; font-weight: 800; color: #172033;")
        layout.addWidget(title)

        self.status_label = QLabel("Waiting for AGV arrival at stores")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(70)
        self.status_label.setAlignment(Qt.AlignVCenter)
        self.status_label.setStyleSheet(
            "background: #e8f0fe; border-radius: 8px; padding: 12px; "
            "font-size: 18px; font-weight: 800; color: #17345c;"
        )
        layout.addWidget(self.status_label)

        self.load_card = QLabel("No AGV ready for loading")
        self.load_card.setWordWrap(True)
        self.load_card.setMinimumHeight(92)
        self.load_card.setAlignment(Qt.AlignVCenter)
        self.load_card.setStyleSheet(
            "background: white; border: 1px solid #cdd6e4; border-radius: 8px; "
            "padding: 16px; font-size: 20px; font-weight: 800; color: #172033;"
        )
        layout.addWidget(self.load_card)

        table_box = QGroupBox("Stores Queue")
        table_layout = QVBoxLayout(table_box)
        self.task_table = QTableWidget()
        self.task_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.task_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.task_table.verticalHeader().setVisible(False)
        self.task_table.horizontalHeader().setStretchLastSection(True)
        self.task_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.task_table.setAlternatingRowColors(True)
        table_layout.addWidget(self.task_table)
        layout.addWidget(table_box, 1)

        button_row = QHBoxLayout()
        self.loaded_button = QPushButton("CONFIRM LOADED - SEND TO WORKBENCH")
        self.loaded_button.setObjectName("loadedButton")
        self.loaded_button.setProperty("ready", "false")
        self.loaded_button.setEnabled(False)
        self.loaded_button.clicked.connect(self.confirm_loaded)
        self.loaded_button.setMinimumHeight(78)
        button_row.addWidget(self.loaded_button, 1)
        layout.addLayout(button_row)

        self.setCentralWidget(central)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(250)

    def confirm_loaded(self) -> None:
        row = self.selected_or_ready_task()
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

    def selected_or_ready_task(self) -> Dict[str, str]:
        row = table_selected_row_payload(self.task_table)
        if row.get("state") == "WAITING_FOR_LOAD":
            return row

        for task in self.stores_tasks():
            if task.get("state") == "WAITING_FOR_LOAD":
                return {key: str(task.get(key, "")) for key in TASK_COLUMNS}

        return row

    def stores_tasks(self) -> List[Dict[str, str]]:
        rows: List[Dict[str, str]] = [
            task
            for task in self.ros_node.delivery_tasks
            if task.get("state") in ("GOING_TO_STORES", "WAITING_FOR_LOAD")
        ]

        rows.sort(
            key=lambda task: (
                task.get("state") != "WAITING_FOR_LOAD",
                float(task.get("created_at", 0.0) or 0.0),
            )
        )

        return rows

    def update_ui(self) -> None:
        rows = self.stores_tasks()
        set_table_rows(self.task_table, TASK_COLUMNS, rows)
        self.highlight_rows(rows)
        self.update_status(rows)

    def highlight_rows(self, rows: List[Dict[str, str]]) -> None:
        for row_index, row in enumerate(rows):
            state = row.get("state", "")
            color = QColor("#ffffff")

            if state == "WAITING_FOR_LOAD":
                color = QColor("#d8f5df")
                if self.task_table.currentRow() < 0:
                    self.task_table.selectRow(row_index)
            elif state == "GOING_TO_STORES":
                color = QColor("#fff3c4")

            for column in range(self.task_table.columnCount()):
                item = self.task_table.item(row_index, column)
                if item is not None:
                    item.setBackground(color)

    def update_status(self, rows: List[Dict[str, str]]) -> None:
        ready_tasks = [
            task for task in rows if task.get("state") == "WAITING_FOR_LOAD"
        ]
        incoming_tasks = [
            task for task in rows if task.get("state") == "GOING_TO_STORES"
        ]

        ready = bool(ready_tasks)

        self.loaded_button.setEnabled(ready)
        self.loaded_button.setProperty("ready", "true" if ready else "false")
        self.loaded_button.style().unpolish(self.loaded_button)
        self.loaded_button.style().polish(self.loaded_button)

        if ready:
            task = ready_tasks[0]
            self.status_label.setText(
                "AGV is at stores. Load the item, then press CONFIRM LOADED."
            )
            self.status_label.setStyleSheet(
                "background: #d8f5df; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 800; color: #0b5126;"
            )
            self.load_card.setText(
                f"READY TO LOAD\n\nRobot: {task.get('robot', '-')}\n"
                f"Item: {task.get('item', '-')}\n"
                f"Destination: {task.get('workbench', '-')}\n"
                f"Task: {task.get('task_id', '-')}"
            )
        elif incoming_tasks:
            task = incoming_tasks[0]
            self.status_label.setText(
                f"AGV is coming to stores for {task.get('workbench', '-')}"
            )
            self.status_label.setStyleSheet(
                "background: #fff3c4; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 800; color: #604500;"
            )
            self.load_card.setText(
                f"INCOMING\n\nRobot: {task.get('robot', '-')}\n"
                f"Item: {task.get('item', '-')}\n"
                f"Destination: {task.get('workbench', '-')}"
            )
        else:
            self.status_label.setText("Waiting for AGV arrival at stores")
            self.status_label.setStyleSheet(
                "background: #e8f0fe; border-radius: 8px; padding: 12px; "
                "font-size: 18px; font-weight: 800; color: #17345c;"
            )
            self.load_card.setText("No AGV ready for loading")


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
