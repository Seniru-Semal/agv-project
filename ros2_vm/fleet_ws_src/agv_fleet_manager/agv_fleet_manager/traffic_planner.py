#!/usr/bin/env python3

from __future__ import annotations

import heapq
import time
import uuid

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple


Edge = Tuple[str, str]


def edge_key(
    node_a: str,
    node_b: str,
) -> Edge:
    values = sorted(
        (
            str(node_a),
            str(node_b),
        )
    )

    return (
        values[0],
        values[1],
    )


@dataclass
class MissionRequest:
    robot: str
    destination: str
    priority: int

    mission_id: str = field(
        default_factory=lambda:
            "mission_"
            + uuid.uuid4().hex[:10]
    )

    queued_at: float = field(
        default_factory=time.monotonic
    )


class TrackPlanner:

    def __init__(
        self,
        nodes: Dict[
            str,
            Dict,
        ],
        connections: Iterable[
            Tuple[str, str]
        ],
    ) -> None:
        self.nodes = nodes

        self.connections: Set[
            Edge
        ] = {
            edge_key(
                node_a,
                node_b,
            )
            for (
                node_a,
                node_b,
            ) in connections
        }

        self.adjacency: Dict[
            str,
            List[str],
        ] = {
            str(node): []
            for node in nodes
        }

        for (
            node_a,
            node_b,
        ) in self.connections:
            self.adjacency[
                node_a
            ].append(
                node_b
            )

            self.adjacency[
                node_b
            ].append(
                node_a
            )

        for neighbors in (
            self.adjacency.values()
        ):
            neighbors.sort()

    def distance(
        self,
        node_a: str,
        node_b: str,
    ) -> float:
        pos_a = self.nodes[
            node_a
        ].get(
            "pos",
            [0.0, 0.0],
        )

        pos_b = self.nodes[
            node_b
        ].get(
            "pos",
            [0.0, 0.0],
        )

        delta_x = (
            float(pos_a[0])
            - float(pos_b[0])
        )

        delta_y = (
            float(pos_a[1])
            - float(pos_b[1])
        )

        return (
            delta_x * delta_x
            + delta_y * delta_y
        ) ** 0.5

    def shortest_path(
        self,
        start: str,
        goal: str,
        blocked_nodes: Iterable[
            str
        ] = (),
        blocked_edges: Iterable[
            Edge
        ] = (),
    ) -> Optional[List[str]]:
        start = (
            str(start)
            .strip()
            .lower()
        )

        goal = (
            str(goal)
            .strip()
            .lower()
        )

        if (
            start not in self.nodes
            or goal not in self.nodes
        ):
            return None

        blocked_node_set = {
            str(node)
            .strip()
            .lower()
            for node in blocked_nodes
        }

        blocked_edge_set = {
            edge_key(
                node_a,
                node_b,
            )
            for (
                node_a,
                node_b,
            ) in blocked_edges
        }

        blocked_node_set.discard(
            start
        )

        if goal in blocked_node_set:
            return None

        queue: List[
            Tuple[float, str]
        ] = [
            (
                0.0,
                start,
            )
        ]

        best_cost: Dict[
            str,
            float,
        ] = {
            start: 0.0
        }

        previous: Dict[
            str,
            str,
        ] = {}

        while queue:
            (
                current_cost,
                node,
            ) = heapq.heappop(
                queue
            )

            if (
                current_cost
                != best_cost.get(node)
            ):
                continue

            if node == goal:
                break

            for neighbor in (
                self.adjacency.get(
                    node,
                    [],
                )
            ):
                if (
                    neighbor
                    in blocked_node_set
                ):
                    continue

                edge = edge_key(
                    node,
                    neighbor,
                )

                if edge in blocked_edge_set:
                    continue

                next_cost = (
                    current_cost
                    + self.distance(
                        node,
                        neighbor,
                    )
                )

                if (
                    next_cost
                    < best_cost.get(
                        neighbor,
                        float("inf"),
                    )
                ):
                    best_cost[
                        neighbor
                    ] = next_cost

                    previous[
                        neighbor
                    ] = node

                    heapq.heappush(
                        queue,
                        (
                            next_cost,
                            neighbor,
                        ),
                    )

        if goal not in best_cost:
            return None

        path = [
            goal
        ]

        while path[-1] != start:
            parent = previous.get(
                path[-1]
            )

            if parent is None:
                return None

            path.append(
                parent
            )

        path.reverse()

        return path
