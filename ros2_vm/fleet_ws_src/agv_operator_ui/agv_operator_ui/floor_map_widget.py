from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QBrush
from PyQt5.QtWidgets import QWidget


Point = Tuple[float, float]

MAP_NODE_ALIASES = {
    "home": "agv1_home",
    "workbench_1": "bench_1",
    "workbench_2": "bench_2",
    "workbench_3": "bench_3",
    "junction1_3": "junction2_1",
    "junction1_4": "junction2_2",
}

MAP_NODE_POINTS: Dict[str, Point] = {
    "agv1_home": (4560, 6550),
    "stores": (6550, 5550),
    "junction1_1": (5050, 5000),
    "junction1_center": (4500, 5000),
    "junction1_2": (4680, 5300),
    "junction2_1": (3260, 4920),
    "junction2_2": (3600, 4560),
    "bench_1": (3150, 3000),
    "bench_2": (3150, 1450),
    "bench_3": (520, 3000),
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
    [(3260, 4920), (1850, 4920), (760, 4700), (520, 4050), (520, 3000), (520, 1100), (920, 520), (2180, 520), (2920, 820), (3150, 1450), (3150, 3000), (3150, 4200), (3400, 4520), (3600, 4560), (4500, 5000)],
    [(4500, 5000), (5050, 5000), (6100, 5000), (6550, 5280), (6550, 5550), (6550, 6850), (6160, 7450), (5000, 7450), (4560, 7020), (4560, 6550), (4680, 5300), (4500, 5000)],
    [(4500, 5000), (3900, 4920), (3260, 4920)],
]

MAP_EDGE_POINTS: Dict[Tuple[str, str], List[Point]] = {
    ("agv1_home", "stores"): [(4560, 6550), (4560, 7020), (5000, 7450), (6160, 7450), (6550, 6850), (6550, 5550)],
    ("stores", "junction1_1"): [(6550, 5550), (6550, 5280), (6100, 5000), (5050, 5000)],
    ("junction1_1", "junction1_center"): [(5050, 5000), (4500, 5000)],
    ("junction1_center", "junction1_2"): [(4500, 5000), (4680, 5300)],
    ("junction1_2", "agv1_home"): [(4680, 5300), (4560, 6550)],
    ("junction1_center", "junction2_1"): [(4500, 5000), (3900, 4920), (3260, 4920)],
    ("junction2_2", "junction1_center"): [(3600, 4560), (3900, 4750), (4500, 5000)],
    ("junction2_1", "bench_3"): [(3260, 4920), (1850, 4920), (760, 4700), (520, 4050), (520, 3000)],
    ("junction2_1", "bench_2"): [(3260, 4920), (1850, 4920), (760, 4700), (520, 4050), (520, 3000), (520, 1100), (920, 520), (2180, 520), (2920, 820), (3150, 1450)],
    ("junction2_1", "bench_1"): [(3260, 4920), (1850, 4920), (760, 4700), (520, 4050), (520, 3000), (520, 1100), (920, 520), (2180, 520), (2920, 820), (3150, 1450), (3150, 3000)],
    ("bench_3", "junction2_2"): [(520, 3000), (520, 1100), (920, 520), (2180, 520), (2920, 820), (3150, 1450), (3150, 3000), (3150, 4200), (3400, 4520), (3600, 4560)],
    ("bench_2", "junction2_2"): [(3150, 1450), (3150, 3000), (3150, 4200), (3400, 4520), (3600, 4560)],
    ("bench_1", "junction2_2"): [(3150, 3000), (3150, 4200), (3400, 4520), (3600, 4560)],
}


def canonical_node(name: Any) -> str:
    value = str(name or "").strip()
    return MAP_NODE_ALIASES.get(value, value)


class FloorMapWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumHeight(360)
        self.fleet_state: Dict[str, Any] = {}

    def set_fleet_state(self, fleet_state: Dict[str, Any]):
        self.fleet_state = fleet_state or {}
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        transform = self.build_transform()

        self.draw_title(painter)
        self.draw_base_tracks(painter, transform)
        self.draw_routes(painter, transform)
        self.draw_nodes(painter, transform)
        self.draw_legend(painter)

    def build_transform(self):
        bounds = QRectF(240, 260, 6740, 7440)
        margin_x = 26
        margin_top = 44
        margin_bottom = 40

        usable_w = max(1, self.width() - margin_x * 2)
        usable_h = max(1, self.height() - margin_top - margin_bottom)
        scale = min(usable_w / bounds.width(), usable_h / bounds.height())

        used_w = bounds.width() * scale
        used_h = bounds.height() * scale
        origin_x = (self.width() - used_w) / 2.0
        origin_y = margin_top + (usable_h - used_h) / 2.0

        def map_point(point: Point) -> QPointF:
            x = origin_x + (point[0] - bounds.left()) * scale
            y = origin_y + (bounds.bottom() - point[1]) * scale
            return QPointF(x, y)

        return map_point

    def make_path(self, points: List[Point], transform) -> QPainterPath:
        path = QPainterPath()
        if not points:
            return path
        path.moveTo(transform(points[0]))
        for point in points[1:]:
            path.lineTo(transform(point))
        return path

    def draw_path(self, painter: QPainter, points: List[Point], transform, color: str, width: int):
        if len(points) < 2:
            return
        painter.setPen(QPen(QColor(color), width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.setBrush(QBrush(Qt.NoBrush))
        painter.drawPath(self.make_path(points, transform))

    def draw_title(self, painter: QPainter):
        painter.setPen(QColor("#0f172a"))
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(18, 26, "Floor Map")

    def draw_base_tracks(self, painter: QPainter, transform):
        for track in MAP_BASE_TRACKS:
            self.draw_path(painter, track, transform, "#e2e8f0", 28)
            self.draw_path(painter, track, transform, "#94a3b8", 2)

    def draw_routes(self, painter: QPainter, transform):
        robots = self.fleet_state.get("robots", {})
        if not isinstance(robots, dict):
            return

        for raw_info in robots.values():
            if not isinstance(raw_info, dict):
                continue

            route = [canonical_node(node) for node in raw_info.get("route", []) if canonical_node(node)]

            try:
                current_index = int(raw_info.get("current_index", 0))
            except (TypeError, ValueError):
                current_index = 0

            safety_hold = bool(raw_info.get("safety_hold", False))

            self.draw_route_polyline(painter, route, transform, "#93c5fd", 10)
            self.draw_active_segment(
                painter,
                route,
                current_index,
                transform,
                "#ef4444" if safety_hold else "#2563eb",
            )

    def draw_route_polyline(self, painter: QPainter, route: List[str], transform, color: str, width: int):
        if len(route) < 2:
            return
        for start, end in zip(route, route[1:]):
            points = self.edge_points(start, end)
            if points:
                self.draw_path(painter, points, transform, color, width)

    def draw_active_segment(self, painter: QPainter, route: List[str], current_index: int, transform, color: str):
        if len(route) < 2:
            return

        index = max(0, min(current_index, len(route) - 1))

        if index < len(route) - 1:
            start = route[index]
            end = route[index + 1]
        else:
            start = route[index - 1]
            end = route[index]

        points = self.edge_points(start, end)
        if points:
            self.draw_path(painter, points, transform, color, 12)

    def edge_points(self, start: str, end: str) -> List[Point]:
        start = canonical_node(start)
        end = canonical_node(end)

        direct = MAP_EDGE_POINTS.get((start, end))
        if direct:
            return direct

        reverse = MAP_EDGE_POINTS.get((end, start))
        if reverse:
            return list(reversed(reverse))

        if start in MAP_NODE_POINTS and end in MAP_NODE_POINTS:
            return [MAP_NODE_POINTS[start], MAP_NODE_POINTS[end]]

        return []

    def draw_nodes(self, painter: QPainter, transform):
        current_nodes = set()

        robots = self.fleet_state.get("robots", {})
        if isinstance(robots, dict):
            for raw_info in robots.values():
                if isinstance(raw_info, dict):
                    current = canonical_node(raw_info.get("current_node", ""))
                    if current:
                        current_nodes.add(current)

        for node, point in MAP_NODE_POINTS.items():
            mapped = transform(point)
            is_station = node in {"agv1_home", "stores", "bench_1", "bench_2", "bench_3"}

            fill = QColor("#dbeafe") if node in current_nodes else QColor("#ffffff")
            border = QColor("#2563eb") if node in current_nodes else QColor("#475569")

            painter.setPen(QPen(border, 2))
            painter.setBrush(fill)

            if is_station:
                rect = QRectF(mapped.x() - 10, mapped.y() - 10, 20, 20)
                painter.drawRoundedRect(rect, 4, 4)
            else:
                painter.drawEllipse(mapped, 8, 8)

            painter.setPen(QColor("#0f172a"))
            font = QFont()
            font.setPointSize(8)
            font.setBold(node in current_nodes)
            painter.setFont(font)
            painter.drawText(mapped + QPointF(12, -8), MAP_NODE_LABELS.get(node, node))

    def draw_legend(self, painter: QPainter):
        y = self.height() - 20
        items = [
            ("Track", "#94a3b8"),
            ("Route", "#93c5fd"),
            ("Active", "#2563eb"),
            ("Hold", "#ef4444"),
        ]

        x = 18
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)

        for label, color in items:
            painter.setPen(QPen(QColor(color), 5, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(x, y - 4, x + 22, y - 4)
            painter.setPen(QColor("#334155"))
            painter.drawText(x + 28, y, label)
            x += 92
