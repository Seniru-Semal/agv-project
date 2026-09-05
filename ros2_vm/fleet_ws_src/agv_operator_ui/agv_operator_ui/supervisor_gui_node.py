#!/usr/bin/env python3

from __future__ import annotations

from typing import Any, Dict, List, Optional

import rclpy

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
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

RECOVERY_NODE_HINTS = [
    "agv1_home",
    "stores",
    "junction1_1",
    "junction1_center",
    "junction1_2",
    "junction2_1",
    "junction2_2",
    "bench_1",
    "bench_2",
    "bench_3",
]


class SupervisorWindow(QMainWindow):
    def __init__(self, ros_node: OperatorRosNode) -> None:
        super().__init__()

        self.ros_node = ros_node
        self.setWindowTitle("AGV Supervisor")
        self.resize(1100, 700)

        central = QWidget()
        layout = QVBoxLayout(central)
        central.setStyleSheet(
            """
            QGroupBox {
                font-weight: 600;
                border: 1px solid #d0d7de;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                min-height: 28px;
                padding: 4px 10px;
            }
            QComboBox {
                min-height: 28px;
                padding: 2px 6px;
            }
            QLabel#RecoveryNote {
                color: #57606a;
            }
            QFrame#RecoveryPanel {
                background: #f6f8fa;
                border: 1px solid #d0d7de;
                border-radius: 8px;
            }
            """
        )

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

        self.recovery_toggle = QPushButton("Show Recovery")
        self.recovery_toggle.setCheckable(True)
        self.recovery_toggle.toggled.connect(self.set_recovery_visible)
        right_layout.addWidget(self.recovery_toggle)

        self.recovery_panel = QFrame()
        self.recovery_panel.setObjectName("RecoveryPanel")
        self.recovery_panel.setVisible(False)
        recovery_layout = QVBoxLayout(self.recovery_panel)

        recovery_title = QLabel("Recovery")
        recovery_title.setStyleSheet("font-size: 15px; font-weight: bold;")
        recovery_layout.addWidget(recovery_title)

        recovery_note = QLabel(
            "Use after a cancelled mission or manual reposition. Confirm the "
            "robot is physically on the selected RFID/node before clearing."
        )
        recovery_note.setObjectName("RecoveryNote")
        recovery_note.setWordWrap(True)
        recovery_layout.addWidget(recovery_note)

        selector_row = QHBoxLayout()
        self.recovery_robot_combo = QComboBox()
        self.recovery_robot_combo.setEditable(True)
        self.recovery_robot_combo.addItem("agv_1")
        self.recovery_node_combo = QComboBox()
        self.recovery_node_combo.setEditable(True)
        self.recovery_node_combo.addItems(RECOVERY_NODE_HINTS)

        selector_row.addWidget(QLabel("Robot"))
        selector_row.addWidget(self.recovery_robot_combo, 1)
        selector_row.addWidget(QLabel("Confirmed node"))
        selector_row.addWidget(self.recovery_node_combo, 2)
        recovery_layout.addLayout(selector_row)

        recovery_buttons = QHBoxLayout()
        resume_button = QPushButton("Resume Hold")
        resume_button.clicked.connect(self.resume_robot)
        set_node_button = QPushButton("Set Node")
        set_node_button.clicked.connect(self.set_robot_node)
        clear_button = QPushButton("Clear Reservations")
        clear_button.clicked.connect(self.clear_robot_reservations)
        reset_button = QPushButton("Reset Robot")
        reset_button.clicked.connect(self.reset_robot)

        recovery_buttons.addWidget(resume_button)
        recovery_buttons.addWidget(set_node_button)
        recovery_buttons.addWidget(clear_button)
        recovery_buttons.addWidget(reset_button)
        recovery_layout.addLayout(recovery_buttons)

        right_layout.addWidget(self.recovery_panel)

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

    def set_recovery_visible(self, visible: bool) -> None:
        self.recovery_panel.setVisible(visible)
        self.recovery_toggle.setText(
            "Hide Recovery" if visible else "Show Recovery"
        )

    def selected_recovery_robot(self) -> str:
        return self.recovery_robot_combo.currentText().strip()

    def selected_recovery_node(self) -> str:
        return self.recovery_node_combo.currentText().strip().lower()

    def resume_robot(self) -> None:
        robot = self.selected_recovery_robot()
        if robot:
            self.ros_node.resume_robot(robot)

    def set_robot_node(self) -> None:
        robot = self.selected_recovery_robot()
        node = self.selected_recovery_node()
        if robot and node:
            self.ros_node.set_robot_current_node(robot, node)

    def clear_robot_reservations(self) -> None:
        robot = self.selected_recovery_robot()
        node = self.selected_recovery_node()
        if not robot or not node:
            return

        result = QMessageBox.question(
            self,
            "Confirm physical position",
            (
                f"Only continue if {robot} is physically stopped at "
                f"{node}.\n\nClear retained reservations and set this "
                "as the confirmed node?"
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if result == QMessageBox.Yes:
            self.ros_node.clear_robot_reservations(robot, node)

    def reset_robot(self) -> None:
        robot = self.selected_recovery_robot()
        if not robot:
            return

        result = QMessageBox.question(
            self,
            "Reset robot",
            f"Send reset command to {robot}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if result == QMessageBox.Yes:
            self.ros_node.reset_robot(robot)

    @staticmethod
    def replace_combo_items(combo: QComboBox, values: List[str]) -> None:
        existing = [
            combo.itemText(index)
            for index in range(combo.count())
        ]

        if existing == values:
            return

        current = combo.currentText().strip()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(values)
        if current:
            index = combo.findText(current)
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setEditText(current)
        combo.blockSignals(False)

    def update_recovery_selectors(self, robot_rows: List[Dict[str, Any]]) -> None:
        robot_names = [
            str(row.get("robot", "")).strip()
            for row in robot_rows
            if str(row.get("robot", "")).strip()
        ]
        if not robot_names:
            robot_names = ["agv_1"]

        nodes = list(RECOVERY_NODE_HINTS)

        for row in robot_rows:
            for key in ("current_node", "destination"):
                value = str(row.get(key, "")).strip().lower()
                if value and value not in nodes:
                    nodes.append(value)

        self.replace_combo_items(
            self.recovery_robot_combo,
            sorted(set(robot_names)),
        )
        self.replace_combo_items(
            self.recovery_node_combo,
            nodes,
        )

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
        self.update_recovery_selectors(robot_rows)

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
