"""Tree data structure used by the RRT family of planners."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Point = Tuple[float, float]


@dataclass
class Node:
    x: float
    y: float
    parent: Optional[int] = None
    cost: float = 0.0  # accumulated cost from the tree root
    children: List[int] = field(default_factory=list)


class Tree:
    def __init__(self):
        self.nodes: List[Node] = []

    # ------------------------------------------------------------------
    def add(self, x: float, y: float, parent: Optional[int] = None,
            cost: float = 0.0) -> int:
        idx = len(self.nodes)
        self.nodes.append(Node(x, y, parent, cost))
        if parent is not None:
            self.nodes[parent].children.append(idx)
        return idx

    def __len__(self) -> int:
        return len(self.nodes)

    # ------------------------------------------------------------------
    def nearest(self, x: float, y: float) -> Tuple[int, float]:
        best_idx, best_dist = 0, math.inf
        for i, node in enumerate(self.nodes):
            d = math.hypot(node.x - x, node.y - y)
            if d < best_dist:
                best_idx, best_dist = i, d
        return best_idx, best_dist

    def near(self, x: float, y: float, radius: float) -> List[Tuple[int, float]]:
        """All nodes within ``radius`` (used by RRT*)."""
        result = []
        for i, node in enumerate(self.nodes):
            d = math.hypot(node.x - x, node.y - y)
            if d <= radius:
                result.append((i, d))
        return result

    # ------------------------------------------------------------------
    def path_to_root(self, idx: int) -> List[Point]:
        path: List[Point] = []
        while idx is not None:
            node = self.nodes[idx]
            path.append((node.x, node.y))
            idx = node.parent
        path.reverse()
        return path

    def edges(self) -> List[Tuple[Point, Point]]:
        out: List[Tuple[Point, Point]] = []
        for i, node in enumerate(self.nodes):
            if node.parent is not None:
                p = self.nodes[node.parent]
                out.append(((p.x, p.y), (node.x, node.y)))
        return out

    # ------------------------------------------------------------------
    def change_parent(self, idx: int, new_parent: int, new_cost: float) -> None:
        """Rewire ``idx`` to a different parent (RRT* rewiring)."""
        node = self.nodes[idx]
        if node.parent is not None:
            self.nodes[node.parent].children.remove(idx)
        node.parent = new_parent
        node.cost = new_cost
        self.nodes[new_parent].children.append(idx)

        # Propagate the new cost to the whole subtree.
        stack = [(child, new_cost) for child in node.children]
        while stack:
            child_idx, parent_cost = stack.pop()
            child = self.nodes[child_idx]
            child.cost = parent_cost + math.hypot(
                child.x - self.nodes[child.parent].x,  # type: ignore[arg-type]
                child.y - self.nodes[child.parent].y,  # type: ignore[arg-type]
            )
            stack.extend((c, child.cost) for c in child.children)
