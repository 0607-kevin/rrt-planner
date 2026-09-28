"""RRT*: asymptotically optimal RRT with neighbour selection and rewiring."""

from __future__ import annotations

import math
from typing import Optional

from .rrt import RRTPlanner
from .tree import Tree


class RRTStarPlanner(RRTPlanner):
    name = "RRT*"

    def __init__(self, *args, gamma: float = 40.0,
                 max_radius: Optional[float] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.gamma = gamma
        self.max_radius = max_radius if max_radius is not None else self.step * 5.0

    def connection_radius(self, n_nodes: int) -> float:
        """Shrinking-ball radius gamma * sqrt(log(n) / n)."""
        shrinking = self.gamma * math.sqrt(
            math.log(n_nodes + 1) / (n_nodes + 1)
        )
        return min(self.max_radius, shrinking)

    # ------------------------------------------------------------------
    def add_node(self, tree: Tree, nearest_idx: int, new) -> int:
        radius = self.connection_radius(len(tree))
        neighbours = tree.near(new[0], new[1], radius)

        # ---- Choose the parent giving the lowest cost-to-come. ----
        nearest = tree.nodes[nearest_idx]
        best_parent = nearest_idx
        best_cost = nearest.cost + math.hypot(
            new[0] - nearest.x, new[1] - nearest.y
        )
        for ni, d in neighbours:
            if ni == nearest_idx:
                continue
            cand = tree.nodes[ni]
            cost = cand.cost + d
            if cost < best_cost and self.env.segment_free(
                (cand.x, cand.y), new
            ):
                best_parent, best_cost = ni, cost

        idx = tree.add(new[0], new[1], best_parent, best_cost)

        # ---- Rewire existing nodes if routing through new is cheaper. ----
        for ni, d in neighbours:
            if ni == best_parent:
                continue
            cand = tree.nodes[ni]
            cost = best_cost + d
            if cost < cand.cost and self.env.segment_free(
                new, (cand.x, cand.y)
            ):
                tree.change_parent(ni, idx, cost)

        return idx
