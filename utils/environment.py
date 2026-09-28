"""Continuous 2D environment with circular and rectangular obstacles.

Unlike a grid, coordinates here are continuous ``(x, y)`` floats. Collision
checks are done both at points and along straight segments (the segment is
densely sampled, which is the standard approach for sampling-based planners).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

Point = Tuple[float, float]


@dataclass
class Circle:
    x: float
    y: float
    r: float

    def contains(self, x: float, y: float) -> bool:
        return math.hypot(x - self.x, y - self.y) <= self.r


@dataclass
class Rectangle:
    x: float
    y: float
    w: float
    h: float

    def contains(self, x: float, y: float) -> bool:
        return self.x <= x <= self.x + self.w and self.y <= y <= self.y + self.h


class Environment:
    """A bounded 2D world holding a list of obstacles."""

    def __init__(self, width: float, height: float):
        self.width = width
        self.height = height
        self.obstacles: List[object] = []

    # ------------------------------------------------------------------
    def add_circle(self, x: float, y: float, r: float) -> None:
        self.obstacles.append(Circle(x, y, r))

    def add_rect(self, x: float, y: float, w: float, h: float) -> None:
        self.obstacles.append(Rectangle(x, y, w, h))

    # ------------------------------------------------------------------
    def in_bounds(self, x: float, y: float) -> bool:
        return 0.0 <= x <= self.width and 0.0 <= y <= self.height

    def collision_free(self, x: float, y: float) -> bool:
        if not self.in_bounds(x, y):
            return False
        return not any(obs.contains(x, y) for obs in self.obstacles)

    def segment_free(self, p1: Point, p2: Point,
                     resolution: float = 0.1) -> bool:
        """Return True when the straight segment p1->p2 is collision-free."""
        x1, y1 = p1
        x2, y2 = p2
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(length / resolution), 1)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.collision_free(x, y):
                return False
        return True

    # ------------------------------------------------------------------
    @classmethod
    def default(cls) -> "Environment":
        """A 50x30 world with scattered obstacles and clear corridors."""
        env = cls(50.0, 30.0)
        env.add_rect(12.0, 0.0, 2.0, 20.0)
        env.add_rect(26.0, 10.0, 2.0, 20.0)
        env.add_rect(38.0, 0.0, 2.0, 18.0)
        env.add_circle(20.0, 22.0, 3.0)
        env.add_circle(33.0, 6.0, 2.5)
        return env
