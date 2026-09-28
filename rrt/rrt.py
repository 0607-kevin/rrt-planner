"""Rapidly-exploring Random Tree (RRT)."""

from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

from .tree import Tree
from utils.environment import Environment

Point = Tuple[float, float]


class RRTPlanner:
    name = "RRT"

    def __init__(
        self,
        env: Environment,
        step_size: float = 2.0,
        max_iter: int = 3000,
        goal_bias: float = 0.1,
        goal_tolerance: float = 1.0,
        seed: Optional[int] = None,
    ):
        self.env = env
        self.step = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias
        self.goal_tol = goal_tolerance
        self.rng = random.Random(seed)

        self.tree: Optional[Tree] = None
        self.iterations_used = 0
        self.snapshots: List[List[Tuple[Point, Point]]] = []

    # ------------------------------------------------------------------
    def sample(self, goal: Point) -> Point:
        if self.rng.random() < self.goal_bias:
            return goal
        return (
            self.rng.uniform(0.0, self.env.width),
            self.rng.uniform(0.0, self.env.height),
        )

    def steer(self, p: Point, target: Point) -> Point:
        dx, dy = target[0] - p[0], target[1] - p[1]
        d = math.hypot(dx, dy)
        if d <= self.step or d == 0.0:
            return target
        return (p[0] + self.step * dx / d, p[1] + self.step * dy / d)

    # ------------------------------------------------------------------
    def add_node(self, tree: Tree, parent_idx: int, new: Point) -> int:
        """Hook overridden by RRT* to do neighbour selection and rewiring."""
        return tree.add(new[0], new[1], parent_idx)

    def plan(self, start: Point, goal: Point) -> Optional[List[Point]]:
        if not self.env.collision_free(*start):
            raise ValueError("start is in collision")
        if not self.env.collision_free(*goal):
            raise ValueError("goal is in collision")

        tree = Tree()
        tree.add(start[0], start[1])
        self.tree = tree
        self.snapshots = []

        for it in range(self.max_iter):
            self.iterations_used = it + 1
            s = self.sample(goal)
            ni, _ = tree.nearest(s[0], s[1])
            near_node = tree.nodes[ni]
            new = self.steer((near_node.x, near_node.y), s)

            if not self.env.segment_free((near_node.x, near_node.y), new):
                continue

            idx = self.add_node(tree, ni, new)

            if it % 25 == 0:
                self.snapshots.append(tree.edges())

            if (
                math.hypot(new[0] - goal[0], new[1] - goal[1]) <= self.goal_tol
                and self.env.segment_free(new, goal)
            ):
                self.snapshots.append(tree.edges())
                return tree.path_to_root(idx) + [goal]

        return None
