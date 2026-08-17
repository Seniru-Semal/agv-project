#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import sys

from typing import (
    Any,
    Dict,
    List,
)

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from PyQt5.QtCore import (
    QPointF,
    Qt,
    QTimer,
)

from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPen,
)

from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from rclpy.node import Node

from std_msgs.msg import String


class FleetRosNode(Node):

    def __init__(self) -> None:
        super().__init__(
            "fleet_hmi_node"
        )

        self.dispatch_pub = (
            self.create_publisher(
                String,
                "/fleet/dispatch",
                10,
            )
        )

        self.cancel_pub = (
            self.create_publisher(
                String,
                "/fleet/cancel",
                10,
            )
        )

        self.resume_pub = (
            self.create_publisher(
                String,
                "/fleet/resume",
                10,
            )
        )

        self.estop_pub = (
            self.create_publisher(
                String,
                "/fleet/estop",
                10,
            )
        )

        self.reset_pub = (
            self.create_publisher(
                String,
                "/fleet/reset",
                10,
            )
        )

        self.clear_pub = (
            self.create_publisher(
                String,
                "/fleet/clear_reservations",
                10,
            )
        )

        self.set_node_pub = (
            self.create_publisher(
                String,
                "/fleet/set_current_node",
                10,
            )
        )

        self.latest_state: Dict[
            str,
            Any,
        ] = {}

        self.events: List[str] = []

        self.create_subscription(
            String,
            "/fleet/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            String,
            "/fleet/event",
            self.event_callback,
            50,
        )

    def state_callback(
        self,
        msg: String,
    ) -> None:

        try:
            value = json.loads(
                msg.data
            )
        except json.JSONDecodeError:
            return

        if isinstance(
            value,
            dict,
        ):
            self.latest_state = value

    def event_callback(
        self,
        msg: String,
    ) -> None:

        text = msg.data.strip()

        if text:
            self.events.append(text)

            self.events = (
                self.events[-300:]
            )

    @staticmethod
    def publish_json(
        publisher: Any,
        payload: Dict[str, Any],
    ) -> None:

        msg = String()

        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )

        publisher.publish(msg)


class TrackWidget(QWidget):

    ROBOT_COLORS = {
        "agv_1":
            QColor(
                55,
                150,
                255,
            ),

        "agv_2":
            QColor(
                255,
                155,
                55,
            ),
    }

    def __init__(
        self,
        config: Dict[str, Any],
    ) -> None:

        super().__init__()

        self.config = config
        self.state: Dict[str, Any] = {}

        self.setMinimumSize(
            420,
            300,
        )

        positions = [
            info["pos"]
            for info in self.config[
                "nodes"
            ].values()
            if (
                isinstance(
                    info.get("pos"),
                    list,
                )
                and len(info["pos"]) == 2
            )
        ]

        self.min_x = min(
            float(position[0])
            for position in positions
        )

        self.max_x = max(
            float(position[0])
            for position in positions
        )

        self.min_y = min(
            float(position[1])
            for position in positions
        )

        self.max_y = max(
            float(position[1])
            for position in positions
        )

    def set_state(
        self,
        state: Dict[str, Any],
    ) -> None:

        self.state = state
        self.update()

    def map_point(
        self,
        node: str,
    ) -> QPointF:

        info = self.config[
            "nodes"
        ][node]

        map_x = float(
            info["pos"][0]
        )

        map_y = float(
            info["pos"][1]
        )

        margin = 55.0

        available_width = max(
            1.0,
            self.width()
            - 2.0 * margin,
        )

        available_height = max(
            1.0,
            self.height()
            - 2.0 * margin,
        )

        span_x = max(
            0.001,
            self.max_x - self.min_x,
        )

        span_y = max(
            0.001,
            self.max_y - self.min_y,
        )

        scale = min(
            available_width / span_x,
            available_height / span_y,
        )

        drawing_width = (
            span_x * scale
        )

        drawing_height = (
            span_y * scale
        )

        offset_x = (
            self.width()
            - drawing_width
        ) / 2.0

        offset_y = (
            self.height()
            - drawing_height
        ) / 2.0

        screen_x = (
            offset_x
            + (
                map_x
                - self.min_x
            )
            * scale
        )

        screen_y = (
            offset_y
            + (
                self.max_y
                - map_y
            )
            * scale
        )

        return QPointF(
            screen_x,
            screen_y,
        )

    def draw_path_section(
        self,
        painter: QPainter,
        path: List[str],
        start_edge: int,
        end_edge: int,
        color: QColor,
        style: Any,
        width: float,
    ) -> None:

        if len(path) < 2:
            return

        pen = QPen(
            color,
            width,
            style,
            Qt.RoundCap,
            Qt.RoundJoin,
        )

        painter.setPen(pen)

        for edge_index in range(
            start_edge,
            min(
                end_edge,
                len(path) - 1,
            ),
        ):
            node_a = path[
                edge_index
            ]

            node_b = path[
                edge_index + 1
            ]

            if (
                node_a
                in self.config["nodes"]
                and node_b
                in self.config["nodes"]
            ):
                painter.drawLine(
                    self.map_point(node_a),
                    self.map_point(node_b),
                )

    def paintEvent(
        self,
        event: Any,
    ) -> None:

        del event

        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.Antialiasing
        )

        painter.fillRect(
            self.rect(),
            QColor(
                24,
                28,
                34,
            ),
        )

        # Physical graph.
        painter.setPen(
            QPen(
                QColor(
                    80,
                    88,
                    98,
                ),
                5.0,
                Qt.SolidLine,
                Qt.RoundCap,
            )
        )

        for node_a, node_b in (
            self.config[
                "connections"
            ]
        ):
            painter.drawLine(
                self.map_point(node_a),
                self.map_point(node_b),
            )

        robots = self.state.get(
            "robots",
            {},
        )

        # Route overlays.
        for (
            robot_name,
            robot,
        ) in robots.items():

            path = robot.get(
                "full_path",
                [],
            )

            if (
                not isinstance(
                    path,
                    list,
                )
                or len(path) < 2
            ):
                continue

            color = self.ROBOT_COLORS.get(
                robot_name,
                QColor(
                    180,
                    180,
                    180,
                ),
            )

            current_index = max(
                0,
                int(
                    robot.get(
                        "current_index",
                        0,
                    )
                ),
            )

            released_index = max(
                current_index,
                int(
                    robot.get(
                        "released_until_index",
                        0,
                    )
                ),
            )

            # Completed section.
            self.draw_path_section(
                painter,
                path,
                0,
                current_index,
                QColor(
                    70,
                    75,
                    82,
                ),
                Qt.SolidLine,
                4.0,
            )

            # Released base.
            self.draw_path_section(
                painter,
                path,
                current_index,
                released_index,
                color,
                Qt.SolidLine,
                8.0,
            )

            # Unreleased horizon.
            self.draw_path_section(
                painter,
                path,
                released_index,
                len(path) - 1,
                color,
                Qt.DashLine,
                3.0,
            )

        painter.setFont(
            QFont(
                "Sans Serif",
                9,
            )
        )

        # Nodes.
        for (
            node_name,
            node_info,
        ) in self.config[
            "nodes"
        ].items():

            point = self.map_point(
                node_name
            )

            node_type = str(
                node_info.get(
                    "type",
                    "junction",
                )
            )

            holding_allowed = bool(
                node_info.get(
                    "holding_allowed",
                    False,
                )
            )

            if node_type in (
                "station",
                "home",
            ):
                brush = QBrush(
                    QColor(
                        66,
                        120,
                        85,
                    )
                )

                radius = 9.0

            elif holding_allowed:
                brush = QBrush(
                    QColor(
                        130,
                        110,
                        45,
                    )
                )

                radius = 7.0

            else:
                brush = QBrush(
                    QColor(
                        100,
                        108,
                        118,
                    )
                )

                radius = 6.0

            painter.setPen(
                QPen(
                    QColor(
                        220,
                        225,
                        230,
                    ),
                    1.5,
                )
            )

            painter.setBrush(brush)

            painter.drawEllipse(
                point,
                radius,
                radius,
            )

            painter.drawText(
                point
                + QPointF(
                    9.0,
                    -8.0,
                ),
                node_name,
            )

        # Current robot positions.
        for (
            robot_name,
            robot,
        ) in robots.items():

            current_node = str(
                robot.get(
                    "current_node",
                    "",
                )
            )

            if (
                current_node
                not in self.config["nodes"]
            ):
                continue

            point = self.map_point(
                current_node
            )

            color = self.ROBOT_COLORS.get(
                robot_name,
                QColor(
                    230,
                    230,
                    230,
                ),
            )

            painter.setPen(
                QPen(
                    QColor(
                        245,
                        245,
                        245,
                    ),
                    2.0,
                )
            )

            painter.setBrush(
                QBrush(color)
            )

            painter.drawEllipse(
                point,
                13.0,
                13.0,
            )

            painter.setPen(
                QPen(
                    QColor(
                        255,
                        255,
                        255,
                    ),
                    1.0,
                )
            )

            painter.drawText(
                point
                + QPointF(
                    -19.0,
                    28.0,
                ),
                robot_name.upper(),
            )

        painter.end()


class RobotCard(QGroupBox):

    def __init__(
        self,
        robot_name: str,
        destinations: List[str],
        ros_node: FleetRosNode,
    ) -> None:

        super().__init__(
            robot_name.upper()
        )

        self.robot_name = robot_name
        self.ros_node = ros_node

        self.labels: Dict[
            str,
            QLabel,
        ] = {}

        form = QFormLayout()

        fields = (
            (
                "status",
                "Traffic state",
            ),
            (
                "current_node",
                "Current node",
            ),
            (
                "next_node",
                "Next node",
            ),
            (
                "destination",
                "Destination",
            ),
            (
                "index",
                "Route index",
            ),
            (
                "release",
                "Released until",
            ),
            (
                "waiting",
                "Waiting reason",
            ),
            (
                "blocked",
                "Blocked resource",
            ),
            (
                "bridge",
                "Bridge",
            ),
            (
                "safety",
                "Safety",
            ),
        )

        for key, title in fields:
            label = QLabel("-")

            label.setTextInteractionFlags(
                Qt.TextSelectableByMouse
            )

            self.labels[key] = label

            form.addRow(
                title,
                label,
            )

        self.destination_combo = (
            QComboBox()
        )

        self.destination_combo.addItems(
            destinations
        )

        self.priority_spin = QSpinBox()

        self.priority_spin.setRange(
            0,
            100,
        )

        self.priority_spin.setValue(
            10
        )

        dispatch_button = QPushButton(
            "Dispatch"
        )

        dispatch_button.clicked.connect(
            self.dispatch
        )

        cancel_button = QPushButton(
            "Cancel"
        )

        cancel_button.clicked.connect(
            self.cancel
        )

        resume_button = QPushButton(
            "Resume"
        )

        resume_button.clicked.connect(
            self.resume
        )

        button_row = QHBoxLayout()

        button_row.addWidget(
            dispatch_button
        )

        button_row.addWidget(
            cancel_button
        )

        button_row.addWidget(
            resume_button
        )

        self.confirm_node_combo = (
            QComboBox()
        )

        self.confirm_node_combo.addItems(
            destinations
        )

        set_node_button = QPushButton(
            "Set current node"
        )

        set_node_button.clicked.connect(
            self.set_current_node
        )

        clear_button = QPushButton(
            "Confirm node and clear "
            "retained reservations"
        )

        clear_button.clicked.connect(
            self.clear_reservations
        )

        layout = QVBoxLayout()

        layout.addLayout(form)

        layout.addWidget(
            QLabel(
                "Mission destination"
            )
        )

        layout.addWidget(
            self.destination_combo
        )

        layout.addWidget(
            QLabel("Priority")
        )

        layout.addWidget(
            self.priority_spin
        )

        layout.addLayout(
            button_row
        )

        layout.addSpacing(8)

        layout.addWidget(
            QLabel(
                "Recovery / confirmed node"
            )
        )

        layout.addWidget(
            self.confirm_node_combo
        )

        layout.addWidget(
            set_node_button
        )

        layout.addWidget(
            clear_button
        )

        self.setLayout(layout)

    def dispatch(self) -> None:
        FleetRosNode.publish_json(
            self.ros_node.dispatch_pub,
            {
                "robot":
                    self.robot_name,

                "destination":
                    self.destination_combo
                    .currentText(),

                "priority":
                    self.priority_spin
                    .value(),
            },
        )

    def cancel(self) -> None:
        FleetRosNode.publish_json(
            self.ros_node.cancel_pub,
            {
                "robot":
                    self.robot_name,
            },
        )

    def resume(self) -> None:
        FleetRosNode.publish_json(
            self.ros_node.resume_pub,
            {
                "robot":
                    self.robot_name,
            },
        )

    def clear_reservations(
        self,
    ) -> None:

        result = QMessageBox.question(
            self,
            "Confirm physical position",
            (
                "Only continue if the robot "
                "is physically stopped at the "
                "selected node.\n\n"
                "Clear retained reservations?"
            ),
            (
                QMessageBox.Yes
                | QMessageBox.No
            ),
            QMessageBox.No,
        )

        if result != QMessageBox.Yes:
            return

        FleetRosNode.publish_json(
            self.ros_node.clear_pub,
            {
                "robot":
                    self.robot_name,

                "confirmed_node":
                    self.confirm_node_combo
                    .currentText(),
            },
        )

    def set_current_node(
        self,
    ) -> None:

        FleetRosNode.publish_json(
            self.ros_node.set_node_pub,
            {
                "robot":
                    self.robot_name,

                "node":
                    self.confirm_node_combo
                    .currentText(),
            },
        )

    def update_robot(
        self,
        data: Dict[str, Any],
    ) -> None:

        current_index = int(
            data.get(
                "current_index",
                0,
            )
        )

        released_index = int(
            data.get(
                "released_until_index",
                0,
            )
        )

        path = data.get(
            "full_path",
            [],
        )

        released_node = "-"

        if (
            isinstance(
                path,
                list,
            )
            and path
            and 0 <= released_index
            < len(path)
        ):
            released_node = str(
                path[released_index]
            )

        self.labels[
            "status"
        ].setText(
            str(
                data.get(
                    "status",
                    "-",
                )
            )
        )

        self.labels[
            "current_node"
        ].setText(
            str(
                data.get(
                    "current_node",
                    "-",
                )
            )
        )

        self.labels[
            "next_node"
        ].setText(
            str(
                data.get(
                    "next_node",
                    "-",
                )
            )
        )

        self.labels[
            "destination"
        ].setText(
            str(
                data.get(
                    "destination",
                    "-",
                )
            )
        )

        self.labels[
            "index"
        ].setText(
            str(current_index)
        )

        self.labels[
            "release"
        ].setText(
            (
                f"{released_index} : "
                f"{released_node}"
            )
        )

        self.labels[
            "waiting"
        ].setText(
            str(
                data.get(
                    "waiting_reason",
                    "-",
                )
            )
        )

        blocked_text = (
            f"{data.get('blocked_resource', '')} "
            f"{data.get('blocked_by', '')}"
        ).strip()

        self.labels[
            "blocked"
        ].setText(
            blocked_text or "-"
        )

        self.labels[
            "bridge"
        ].setText(
            (
                "ONLINE"
                if data.get(
                    "bridge_connected"
                )
                else "OFFLINE"
            )
        )

        safety_text = (
            "OK"
            if data.get("safety_ok")
            else "NOT READY"
        )

        if data.get(
            "safety_hold"
        ):
            safety_text += " / HOLD"

        self.labels[
            "safety"
        ].setText(
            safety_text
        )


class FleetWindow(QMainWindow):

    def __init__(
        self,
        ros_node: FleetRosNode,
        config: Dict[str, Any],
    ) -> None:

        super().__init__()

        self.ros_node = ros_node
        self.config = config

        self.last_event_count = 0

        self.setWindowTitle(
            "AGV Dynamic Fleet Manager"
        )

        self.resize(
            1000,
            620,
        )

        self.setMinimumSize(
            700,
            450,
        )

        central = QWidget()

        main_layout = QVBoxLayout(
            central
        )

        top_row = QHBoxLayout()

        title = QLabel(
            "AGV Fleet Traffic Control"
        )

        title.setFont(
            QFont(
                "Sans Serif",
                16,
                QFont.Bold,
            )
        )

        top_row.addWidget(title)
        top_row.addStretch()

        estop_all = QPushButton(
            "FLEET E-STOP"
        )

        estop_all.setObjectName(
            "danger"
        )

        estop_all.clicked.connect(
            lambda:
                FleetRosNode.publish_json(
                    self.ros_node.estop_pub,
                    {
                        "robot": "all",
                    },
                )
        )

        reset_all = QPushButton(
            "Fleet reset"
        )

        reset_all.clicked.connect(
            lambda:
                FleetRosNode.publish_json(
                    self.ros_node.reset_pub,
                    {
                        "robot": "all",
                    },
                )
        )

        top_row.addWidget(
            estop_all
        )

        top_row.addWidget(
            reset_all
        )

        main_layout.addLayout(
            top_row
        )

        splitter = QSplitter(
            Qt.Horizontal
        )

        self.track = TrackWidget(
            config
        )

        splitter.addWidget(
            self.track
        )

        right_panel = QWidget()

        right_layout = QVBoxLayout(
            right_panel
        )

        destinations = sorted(
            config["nodes"].keys()
        )

        self.cards: Dict[
            str,
            RobotCard,
        ] = {}

        for robot_name in (
            config["robots"].keys()
        ):
            card = RobotCard(
                robot_name,
                destinations,
                ros_node,
            )

            self.cards[
                robot_name
            ] = card

            right_layout.addWidget(
                card
            )

        right_layout.addStretch()

        splitter.addWidget(
            right_panel
        )

        splitter.setStretchFactor(
            0,
            3,
        )

        splitter.setStretchFactor(
            1,
            2,
        )

        main_layout.addWidget(
            splitter,
            1,
        )

        tabs = QTabWidget()

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)

        self.queue_log = QTextEdit()
        self.queue_log.setReadOnly(True)

        self.reservation_log = QTextEdit()
        self.reservation_log.setReadOnly(
            True
        )

        tabs.addTab(
            self.event_log,
            "Events",
        )

        tabs.addTab(
            self.queue_log,
            "Traffic queue",
        )

        tabs.addTab(
            self.reservation_log,
            "Reservations",
        )

        tabs.setMinimumHeight(
            100
        )

        tabs.setMaximumHeight(
            160
        )

        main_layout.addWidget(tabs)

        central_scroll = QScrollArea()

        central_scroll.setWidgetResizable(
            True
        )

        central_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        central_scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        central_scroll.setWidget(
            central
        )

        self.setCentralWidget(
            central_scroll
        )

        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #181c22;
                color: #e8edf2;
            }

            QGroupBox {
                border: 1px solid #46505c;
                margin-top: 9px;
                padding-top: 8px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
            }

            QPushButton {
                background: #313945;
                border: 1px solid #596675;
                padding: 7px;
            }

            QPushButton:hover {
                background: #3e4958;
            }

            QPushButton#danger {
                background: #8f2424;
                font-weight: bold;
            }

            QComboBox,
            QSpinBox,
            QTextEdit {
                background: #222831;
                border: 1px solid #4d5968;
                padding: 4px;
            }

            QTabWidget::pane {
                border: 1px solid #46505c;
            }
            """
        )

        self.timer = QTimer(self)

        self.timer.timeout.connect(
            self.update_ui
        )

        self.timer.start(100)

    def update_ui(self) -> None:

        state = (
            self.ros_node.latest_state
        )

        if state:
            self.track.set_state(state)

            robots = state.get(
                "robots",
                {},
            )

            for (
                robot_name,
                card,
            ) in self.cards.items():

                robot_data = robots.get(
                    robot_name,
                    {},
                )

                if isinstance(
                    robot_data,
                    dict,
                ):
                    card.update_robot(
                        robot_data
                    )

            queue = state.get(
                "queue",
                [],
            )

            reservations = state.get(
                "reservations",
                {},
            )

            self.queue_log.setPlainText(
                json.dumps(
                    queue,
                    indent=2,
                )
            )

            self.reservation_log.setPlainText(
                json.dumps(
                    reservations,
                    indent=2,
                )
            )

        if (
            len(self.ros_node.events)
            != self.last_event_count
        ):
            self.event_log.setPlainText(
                "\n".join(
                    self.ros_node.events
                )
            )

            scrollbar = (
                self.event_log
                .verticalScrollBar()
            )

            scrollbar.setValue(
                scrollbar.maximum()
            )

            self.last_event_count = len(
                self.ros_node.events
            )

    def closeEvent(
        self,
        event: Any,
    ) -> None:

        self.timer.stop()
        event.accept()


def load_config() -> Dict[str, Any]:

    config_path = os.path.join(
        get_package_share_directory(
            "agv_fleet_manager"
        ),
        "config",
        "fleet_config.json",
    )

    with open(
        config_path,
        "r",
        encoding="utf-8",
    ) as stream:
        return json.load(stream)


def main(args=None) -> None:
    rclpy.init(args=args)

    ros_node = FleetRosNode()

    app = QApplication(
        sys.argv
    )

    window = FleetWindow(
        ros_node,
        load_config(),
    )

    window.showMaximized()

    ros_timer = QTimer()

    ros_timer.timeout.connect(
        lambda:
            rclpy.spin_once(
                ros_node,
                timeout_sec=0.0,
            )
    )

    ros_timer.start(20)

    try:
        app.exec_()
    finally:
        ros_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
