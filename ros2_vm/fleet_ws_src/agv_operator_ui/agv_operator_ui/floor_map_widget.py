from __future__ import annotations

from typing import Any, Dict, List, Optional, Set, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QBrush
from PyQt5.QtWidgets import QSizePolicy, QWidget


Point = Tuple[float, float]
Edge = Tuple[str, str]

# Your GUI was visually opposite to the physical floor view.
# If you later want the drawing exactly like the PDF orientation, set both to False.
MIRROR_X = True
MIRROR_Y = True

MAP_NODE_ALIASES = {
    "home": "agv1_home",
    "workbench_1": "bench_1",
    "workbench_2": "bench_2",
    "workbench_3": "bench_3",
    "junction1_3": "junction2_1",
    "junction1_4": "junction2_2",
}

MAP_NODE_POINTS: Dict[str, Point] = {
    "agv1_home": (4560.0, 6550.0),
    "stores": (6550.0, 5550.0),
    "junction1_1": (5050.0, 5000.0),
    "junction1_center": (4500.0, 5000.0),
    "junction1_2": (4680.0, 5300.0),

    # Corrected for your newer naming:
    # center -> benches uses junction2_1
    # benches -> center uses junction2_2
    "junction2_1": (3600.0, 4560.0),
    "junction2_2": (3260.0, 4920.0),

    "bench_1": (3150.0, 3000.0),
    "bench_2": (3150.0, 1450.0),
    "bench_3": (520.0, 3000.0),
}

MAP_NODE_LABELS = {
    "agv1_home": "Home",
    "stores": "Stores",
    "junction1_1": "J1-1",
    "junction1_center": "J1 Center",
    "junction1_2": "J1-2",
    "junction2_1": "J2-1",
    "junction2_2": "J2-2",
    "bench_1": "Bench 1",
    "bench_2": "Bench 2",
    "bench_3": "Bench 3",
}

MAP_BASE_TRACKS: List[List[Point]] = [
    [
        (3260.0, 4920.0),
        (1850.0, 4920.0),
        (760.0, 4700.0),
        (520.0, 4050.0),
        (520.0, 3000.0),
        (520.0, 1100.0),
        (920.0, 520.0),
        (2180.0, 520.0),
        (2920.0, 820.0),
        (3150.0, 1450.0),
        (3150.0, 3000.0),
        (3150.0, 4200.0),
        (3400.0, 4520.0),
        (3600.0, 4560.0),
        (4500.0, 5000.0),
    ],
    [
        (4500.0, 5000.0),
        (5050.0, 5000.0),
        (6100.0, 5000.0),
        (6550.0, 5280.0),
        (6550.0, 5550.0),
        (6550.0, 6850.0),
        (6160.0, 7450.0),
        (5000.0, 7450.0),
        (4560.0, 7020.0),
        (4560.0, 6550.0),
        (4680.0, 5300.0),
        (4500.0, 5000.0),
    ],
    [
        (4500.0, 5000.0),
        (3900.0, 4860.0),
        (3600.0, 4560.0),
    ],
    [
        (3260.0, 4920.0),
        (3900.0, 4920.0),
        (4500.0, 5000.0),
    ],
]

MAP_EDGE_POINTS: Dict[Edge, List[Point]] = {
    ("agv1_home", "stores"): [
        (4560.0, 6550.0),
        (4560.0, 7020.0),
        (5000.0, 7450.0),
        (6160.0, 7450.0),
        (6550.0, 6850.0),
        (6550.0, 5550.0),
    ],
    ("stores", "junction1_1"): [
        (6550.0, 5550.0),
        (6550.0, 5280.0),
        (6100.0, 5000.0),
        (5050.0, 5000.0),
    ],
    ("junction1_1", "junction1_center"): [
        (5050.0, 5000.0),
        (4500.0, 5000.0),
    ],
    ("junction1_center", "junction2_1"): [
        (4500.0, 5000.0),
        (3900.0, 4860.0),
        (3600.0, 4560.0),
    ],
    ("junction2_1", "bench_1"): [
        (3600.0, 4560.0),
        (3150.0, 4200.0),
        (3150.0, 3000.0),
    ],
    ("junction2_1", "bench_2"): [
        (3600.0, 4560.0),
        (3150.0, 4200.0),
        (3150.0, 3000.0),
        (3150.0, 1450.0),
    ],
    ("junction2_1", "bench_3"): [
        (3600.0, 4560.0),
        (3150.0, 4200.0),
        (3150.0, 3000.0),
        (3150.0, 1450.0),
        (2920.0, 820.0),
        (2180.0, 520.0),
        (920.0, 520.0),
        (520.0, 1100.0),
        (520.0, 3000.0),
    ],
    ("bench_1", "junction2_2"): [
        (3150.0, 3000.0),
        (3150.0, 4200.0),
        (3260.0, 4920.0),
    ],
    ("bench_2", "junction2_2"): [
        (3150.0, 1450.0),
        (3150.0, 3000.0),
        (3150.0, 4200.0),
        (3260.0, 4920.0),
    ],
    ("bench_3", "junction2_2"): [
        (520.0, 3000.0),
        (520.0, 4050.0),
        (760.0, 4700.0),
        (1850.0, 4920.0),
        (3260.0, 4920.0),
    ],
    ("junction2_2", "junction1_center"): [
        (3260.0, 4920.0),
        (3900.0, 4920.0),
        (4500.0, 5000.0),
    ],
    ("junction1_center", "junction1_2"): [
        (4500.0, 5000.0),
        (4680.0, 5300.0),
    ],
    ("junction1_2", "agv1_home"): [
        (4680.0, 5300.0),
        (4560.0, 6550.0),
    ],
}


def canonical_node(node: Any) -> str:
    text = str(node or "").strip().lower()
    return MAP_NODE_ALIASES.get(text, text)


class FloorMapWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.fleet_state: Dict[str, Any] = {}
        self.setMinimumHeight(360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_fleet_state(self, fleet_state: Dict[str, Any]) -> None:
        self.fleet_state = fleet_state if isinstance(fleet_state, dict) else {}
        self.update()

    def map_bounds(self) -> QRectF:
        return QRectF(250.0, 250.0, 6750.0, 7300.0)

    def point(self, raw_point: Point) -> QPointF:
        bounds = self.map_bounds()
        margin = 18.0

        usable_width = max(1.0, self.width() - margin * 2.0)
        usable_height = max(1.0, self.height() - margin * 2.0)
        scale = min(usable_width / bounds.width(), usable_height / bounds.height())

        draw_width = bounds.width() * scale
        draw_height = bounds.height() * scale
        origin_x = (self.width() - draw_width) / 2.0
        origin_y = (self.height() - draw_height) / 2.0

        x = bounds.right() - raw_point[0] if MIRROR_X else raw_point[0]
        y = bounds.bottom() - raw_point[1] if MIRROR_Y else raw_point[1]

        return QPointF(
            origin_x + ((x - bounds.left()) * scale),
            origin_y + ((y - bounds.top()) * scale),
        )

    def path_from_points(self, points: List[Point]) -> QPainterPath:
        path = QPainterPath()
        if not points:
            return path

        path.moveTo(self.point(points[0]))
        for point in points[1:]:
            path.lineTo(self.point(point))
        return path

    def edge_points(self, start: Any, end: Any) -> List[Point]:
        node_a = canonical_node(start)
        node_b = canonical_node(end)

        points = MAP_EDGE_POINTS.get((node_a, node_b))
        if points is not None:
            return points

        points = MAP_EDGE_POINTS.get((node_b, node_a))
        if points is not None:
            return list(reversed(points))

        point_a = MAP_NODE_POINTS.get(node_a)
        point_b = MAP_NODE_POINTS.get(node_b)
        if point_a is None or point_b is None:
            return []

        return [point_a, point_b]

    def draw_path(self, painter: QPainter, points: List[Point], color: str, width: int, style=Qt.SolidLine) -> None:
        if len(points) < 2:
            return

        pen = QPen(QColor(color), width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        pen.setStyle(style)
        painter.setPen(pen)
        painter.setBrush(QBrush(Qt.NoBrush))
        painter.drawPath(self.path_from_points(points))

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        self.draw_title(painter)
        self.draw_base_map(painter)
        self.draw_routes(painter)
        self.draw_nodes(painter)
        self.draw_legend(painter)

    def draw_title(self, painter: QPainter) -> None:
        painter.setPen(QColor("#0f172a"))
        painter.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
        painter.drawText(18, 25, "Floor Map")

    def draw_base_map(self, painter: QPainter) -> None:
        for track in MAP_BASE_TRACKS:
            self.draw_path(painter, track, "#e2e8f0", 24)
            self.draw_path(painter, track, "#64748b", 3, Qt.DashLine)

    def robots(self) -> Dict[str, Any]:
        robots = self.fleet_state.get("robots", {})
        return robots if isinstance(robots, dict) else {}

    def route_for_robot(self, robot_data: Dict[str, Any]) -> List[str]:
        raw_path = robot_data.get("full_path", robot_data.get("route", []))
        if not isinstance(raw_path, list):
            return []
        return [canonical_node(node) for node in raw_path if canonical_node(node)]

    def int_value(self, value: Any, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def draw_routes(self, painter: QPainter) -> None:
        all_reserved_edges = self.reserved_edges_from_state()

        for robot_name, robot_data in self.robots().items():
            if not isinstance(robot_data, dict):
                continue

            route = self.route_for_robot(robot_data)
            current_index = self.int_value(robot_data.get("current_index"), 0)
            released_until = self.int_value(robot_data.get("released_until_index"), current_index)

            self.draw_full_route(painter, route, current_index)
            self.draw_released_route(painter, route, current_index, released_until)
            self.draw_reserved_edges(painter, all_reserved_edges)
            self.draw_active_segment(painter, str(robot_name), robot_data, route, current_index)

    def draw_full_route(self, painter: QPainter, route: List[str], current_index: int) -> None:
        if len(route) < 2:
            return

        start_index = max(0, min(current_index, len(route) - 2))
        for index in range(start_index, len(route) - 1):
            self.draw_path(painter, self.edge_points(route[index], route[index + 1]), "#38bdf8", 8)

    def draw_released_route(self, painter: QPainter, route: List[str], current_index: int, released_until: int) -> None:
        if len(route) < 2:
            return

        start_index = max(0, min(current_index, len(route) - 2))
        end_index = max(start_index, min(released_until, len(route) - 1))

        for index in range(start_index, end_index):
            self.draw_path(painter, self.edge_points(route[index], route[index + 1]), "#facc15", 12)

    def reserved_edges_from_state(self) -> Set[Edge]:
        edges: Set[Edge] = set()

        for robot_data in self.robots().values():
            if not isinstance(robot_data, dict):
                continue

            for raw_edge in robot_data.get("reserved_edges", []) or []:
                edge = self.clean_edge(raw_edge)
                if edge:
                    edges.add(edge)

        reservations = self.fleet_state.get("reservations", {})
        if isinstance(reservations, dict):
            for entry in reservations.get("edge_reservations", []) or []:
                if isinstance(entry, dict):
                    edge = self.clean_edge(entry.get("edge"))
                    if edge:
                        edges.add(edge)

        return edges

    def clean_edge(self, raw_edge: Any) -> Optional[Edge]:
        if not isinstance(raw_edge, list) or len(raw_edge) != 2:
            return None

        start = canonical_node(raw_edge[0])
        end = canonical_node(raw_edge[1])
        if not start or not end:
            return None

        return (start, end)

    def draw_reserved_edges(self, painter: QPainter, edges: Set[Edge]) -> None:
        for start, end in sorted(edges):
            self.draw_path(painter, self.edge_points(start, end), "#f59e0b", 14)

    def draw_active_segment(
        self,
        painter: QPainter,
        robot_name: str,
        robot_data: Dict[str, Any],
        route: List[str],
        current_index: int,
    ) -> None:
        current_node = canonical_node(robot_data.get("current_node", ""))
        next_node = canonical_node(robot_data.get("next_node", ""))

        if (not current_node or not next_node) and len(route) >= 2:
            index = max(0, min(current_index, len(route) - 2))
            current_node = route[index]
            next_node = route[index + 1]

        if not current_node or not next_node:
            return

        color = "#dc2626" if bool(robot_data.get("safety_hold", False)) else "#2563eb"
        points = self.edge_points(current_node, next_node)
        self.draw_path(painter, points, color, 18)

        if points:
            marker = self.point(points[len(points) // 2])
            self.draw_robot_marker(painter, marker, robot_name, color)

    def draw_robot_marker(self, painter: QPainter, point: QPointF, robot_name: str, color: str) -> None:
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(QBrush(QColor(color)))
        painter.drawEllipse(point, 9, 9)

        painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold))
        painter.setPen(QColor(color))
        painter.drawText(QRectF(point.x() - 42, point.y() + 12, 84, 18), Qt.AlignCenter, robot_name.upper())

    def draw_nodes(self, painter: QPainter) -> None:
        current_nodes = set()
        reserved_nodes = self.reserved_nodes_from_state()
        occupied_nodes = self.occupied_nodes_from_state()

        for robot_data in self.robots().values():
            if isinstance(robot_data, dict):
                node = canonical_node(robot_data.get("current_node", ""))
                if node:
                    current_nodes.add(node)

        for node, raw_point in MAP_NODE_POINTS.items():
            point = self.point(raw_point)
            is_station = node in {"agv1_home", "stores", "bench_1", "bench_2", "bench_3"}

            fill = QColor("#ffffff")
            border = QColor("#475569")
            ring = None

            if node in reserved_nodes:
                ring = QColor("#f59e0b")
            if node in occupied_nodes:
                ring = QColor("#dc2626")
            if node in current_nodes:
                ring = QColor("#2563eb")
                fill = QColor("#dbeafe")

            if ring is not None:
                painter.setPen(QPen(ring, 4))
                painter.setBrush(QBrush(Qt.NoBrush))
                painter.drawEllipse(point, 15, 15)

            painter.setPen(QPen(border, 2))
            painter.setBrush(QBrush(fill))

            if is_station:
                rect = QRectF(point.x() - 10, point.y() - 10, 20, 20)
                painter.drawRoundedRect(rect, 4, 4)
            else:
                painter.drawEllipse(point, 7, 7)

            painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold if node in current_nodes else QFont.Normal))
            painter.setPen(QColor("#0f172a"))
            painter.drawText(QPointF(point.x() + 13, point.y() - 7), MAP_NODE_LABELS.get(node, node))

    def reserved_nodes_from_state(self) -> Set[str]:
        nodes: Set[str] = set()

        for robot_data in self.robots().values():
            if not isinstance(robot_data, dict):
                continue

            for node in robot_data.get("reserved_nodes", []) or []:
                clean = canonical_node(node)
                if clean:
                    nodes.add(clean)

        reservations = self.fleet_state.get("reservations", {})
        if isinstance(reservations, dict):
            node_reservations = reservations.get("node_reservations", {})
            if isinstance(node_reservations, dict):
                for node in node_reservations:
                    clean = canonical_node(node)
                    if clean:
                        nodes.add(clean)

        return nodes

    def occupied_nodes_from_state(self) -> Set[str]:
        nodes: Set[str] = set()
        reservations = self.fleet_state.get("reservations", {})

        if isinstance(reservations, dict):
            occupied = reservations.get("occupied_nodes", {})
            if isinstance(occupied, dict):
                for node in occupied:
                    clean = canonical_node(node)
                    if clean:
                        nodes.add(clean)

        return nodes

    def draw_legend(self, painter: QPainter) -> None:
        y = self.height() - 18
        items = [
            ("Track", "#64748b"),
            ("Route", "#38bdf8"),
            ("Reserved", "#f59e0b"),
            ("Active", "#2563eb"),
            ("Hold", "#dc2626"),
        ]

        painter.setFont(QFont("DejaVu Sans", 8))
        x = 18

        for label, color in items:
            painter.setPen(QPen(QColor(color), 5, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(x, y - 4, x + 22, y - 4)
            painter.setPen(QColor("#334155"))
            painter.drawText(x + 28, y, label)
            x += 92
