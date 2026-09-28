"""Matplotlib visualisation of RRT trees, paths and growth animation."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle as MplCircle
from matplotlib.patches import Rectangle as MplRect

from .environment import Circle, Environment, Rectangle

Point = Tuple[float, float]


def path_length(path: Sequence[Point]) -> float:
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    )


def draw_environment(ax, env: Environment) -> None:
    for obs in env.obstacles:
        if isinstance(obs, Circle):
            ax.add_patch(
                MplCircle((obs.x, obs.y), obs.r,
                          facecolor="#bdbdbd", edgecolor="#616161")
            )
        elif isinstance(obs, Rectangle):
            ax.add_patch(
                MplRect((obs.x, obs.y), obs.w, obs.h,
                        facecolor="#bdbdbd", edgecolor="#616161")
            )
    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)


def draw_tree(ax, edges, color: str = "#1976d2") -> None:
    lc = LineCollection(edges, colors=color, alpha=0.55, linewidths=0.8)
    ax.add_collection(lc)


def draw_path(ax, path, color: str = "#d32f2f") -> None:
    ax.plot([p[0] for p in path], [p[1] for p in path],
            color=color, linewidth=2.4, zorder=6)


def plot_result(env: Environment, tree_edge_sets: List[list],
                path: Optional[List[Point]], start: Point, goal: Point,
                title: str = "", ax=None):
    created = ax is None
    if created:
        fig, ax = plt.subplots(figsize=(10, 6))

    draw_environment(ax, env)
    colors = ["#1976d2", "#388e3c", "#7b1fa2"]
    for i, edges in enumerate(tree_edge_sets):
        draw_tree(ax, edges, color=colors[i % len(colors)])
    if path:
        draw_path(ax, path)

    ax.scatter(start[0], start[1], marker="o", color="#2e7d32",
               s=80, zorder=7, label="start")
    ax.scatter(goal[0], goal[1], marker="*", color="#c62828",
               s=160, zorder=7, label="goal")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    return ax


def animate_growth(env: Environment, planner, start: Point, goal: Point,
                   interval: int = 60, title: str = ""):
    """Replay the tree growth using the planner's recorded snapshots."""
    path = planner.plan(start, goal)
    snapshots = planner.snapshots

    fig, ax = plt.subplots(figsize=(10, 6))
    draw_environment(ax, env)
    ax.scatter(start[0], start[1], marker="o", color="#2e7d32", s=80, zorder=7)
    ax.scatter(goal[0], goal[1], marker="*", color="#c62828", s=160, zorder=7)

    lc = LineCollection([], colors="#1976d2", alpha=0.55, linewidths=0.8)
    ax.add_collection(lc)
    (path_line,) = ax.plot([], [], color="#d32f2f", linewidth=2.4, zorder=6)
    ax.set_title(title or planner.name)

    n = len(snapshots)

    def update(frame):
        if frame < n:
            lc.set_segments(snapshots[frame])
            path_line.set_data([], [])
        else:
            if snapshots:
                lc.set_segments(snapshots[-1])
            if path:
                path_line.set_data(
                    [p[0] for p in path], [p[1] for p in path]
                )
        return lc, path_line

    anim = FuncAnimation(fig, update, frames=n + 1, interval=interval,
                         blit=True, repeat=False)
    return fig, anim
