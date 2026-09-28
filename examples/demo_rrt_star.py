"""Compare RRT and RRT*: RRT* rewires toward an asymptotically optimal path."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from rrt import RRTPlanner, RRTStarPlanner
from utils import Environment
from utils.visualize import path_length, plot_result


def main() -> None:
    env = Environment.default()
    start, goal = (3.0, 15.0), (46.0, 15.0)

    rrt = RRTPlanner(env, step_size=2.0, max_iter=6000, seed=7)
    rrt_star = RRTStarPlanner(env, step_size=2.0, max_iter=6000, seed=7)

    p1 = rrt.plan(start, goal)
    p2 = rrt_star.plan(start, goal)
    assert p1 is not None and p2 is not None

    straight = path_length([start, goal])
    print(f"straight-line distance : {straight:.2f}")
    print(f"RRT   path length      : {path_length(p1):.2f}")
    print(f"RRT*  path length      : {path_length(p2):.2f}")
    print(f"RRT*  iterations       : {rrt_star.iterations_used}")

    fig, axes = plt.subplots(1, 2, figsize=(18, 6))
    plot_result(env, [rrt.tree.edges()], p1, start, goal, "RRT", ax=axes[0])
    plot_result(env, [rrt_star.tree.edges()], p2, start, goal, "RRT*", ax=axes[1])
    plt.show()


if __name__ == "__main__":
    main()
