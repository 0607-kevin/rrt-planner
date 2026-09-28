"""RRT-Connect: two trees greedily grow toward each other.

A single RRT is cheap but slow to converge in cluttered scenes. RRT-Connect
grows one tree toward a random sample and then aggressively extends a second
tree (rooted at the goal) straight toward the newly added node. The two trees
are swapped every iteration.
"""

from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

from .rrt import RRTPlanner
from .tree import Tree
from utils.environment import Environment

Point = Tuple[float, float]

_TRAPPED = 0
_ADVANCED = 1
_REACHED = 2


class RRTConnectPlanner(RRTPlanner):
    name = "RRT-Connect"

    def __init__(self, env: Environment, step_size: float = 2.0,
                 max_iter: int = 2000, seed: Optional[int] = None):
        super().__init__(env, step_size=step_size, max_iter=max_iter,
                         goal_bias=0.0, seed=seed)
        self.start_tree: Optional[Tree] = None
        self.goal_tree: Optional[Tree] = None

    # ------------------------------------------------------------------
    def _extend(self, tree: Tree, target: Point) -> Tuple[int, Point, int]:
        ni, _ = tree.nearest(target[0], target[1])
        n = tree.nodes[ni]
        new = self.steer((n.x, n.y), target)
        if self.env.segment_free((n.x, n.y), new):
            idx = tree.add(new[0], new[1], ni)
            status = _REACHED if new == target else _ADVANCED
            return idx, new, status
        return -1, new, _TRAPPED

    def _connect(self, tree: Tree, target: Point) -> Tuple[int, int]:
        """Greedy extension until the target is reached or progress stops."""
        idx, status = -1, _ADVANCED
        while status == _ADVANCED:
            idx, _, status = self._extend(tree, target)
        return idx, status

    # ------------------------------------------------------------------
    def plan(self, start: Point, goal: Point) -> Optional[List[Point]]:
        if not self.env.collision_free(*start):
            raise ValueError("start is in collision")
        if not self.env.collision_free(*goal):
            raise ValueError("goal is in collision")

        ta = Tree()
        ta.add(start[0], start[1])
        tb = Tree()
        tb.add(goal[0], goal[1])
        self.start_tree, self.goal_tree = ta, tb
        trees = [ta, tb]

        for it in range(self.max_iter):
            self.iterations_used = it + 1
            a_idx = it % 2
            b_idx = 1 - a_idx
            tree_a, tree_b = trees[a_idx], trees[b_idx]

            sample = (
                self.rng.uniform(0, self.env.width),
                self.rng.uniform(0, self.env.height),
            )
            new_idx, new_point, status = self._extend(tree_a, sample)
            if status != _TRAPPED:
                connect_idx, connect_status = self._connect(tree_b, new_point)
                if connect_status == _REACHED:
                    return self._merge(
                        tree_a, new_idx, tree_b, connect_idx,
                        a_is_start=(a_idx == 0),
                    )
        return None

    # ------------------------------------------------------------------
    @staticmethod
    def _merge(tree_a, idx_a, tree_b, idx_b, a_is_start: bool) -> List[Point]:
        path_a = tree_a.path_to_root(idx_a)   # root(a) -> connection
        path_b = tree_b.path_to_root(idx_b)   # root(b) -> connection

        # Reverse the root(b)->connection path into connection->root(b), then
        # drop its first point (the connection, already the last point of the
        # other half).
        tail_b = list(reversed(path_b))[1:]
        tail_a = list(reversed(path_a))[1:]

        if a_is_start:  # a rooted at start, b rooted at goal
            return path_a + tail_b
        # b rooted at start, a rooted at goal
        return path_b + tail_a
