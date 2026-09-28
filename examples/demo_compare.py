"""Compare RRT, RRT* and RRT-Connect side by side."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from rrt import RRTConnectPlanner, RRTPlanner, RRTStarPlanner
from utils import Environment
from utils.visualize import path_length, plot_result


def main() -> None:
    env = Environment.default()
    start, goal = (3.0, 15.0), (46.0, 15.0)

    planners = [
        ("RRT", RRTPlanner(env, step_size=2.0, max_iter=6000, seed=11)),
        ("RRT*", RRTStarPlanner(env, step_size=2.0, max_iter=6000, seed=11)),
        ("RRT-Connect",
         RRTConnectPlanner(env, step_size=2.0, max_iter=4000, seed=11)),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(24, 6))
    print(f"{'planner':<14}{'length':>10}{'iterations':>12}")
    for ax, (name, planner) in zip(axes, planners):
        path = planner.plan(start, goal)
        assert path is not None, f"{name} failed"

        if name == "RRT-Connect":
            edge_sets = [
                planner.start_tree.edges(),
                planner.goal_tree.edges(),
            ]
        else:
            edge_sets = [planner.tree.edges()]
        plot_result(env, edge_sets, path, start, goal,
                    f"{name} (len={path_length(path):.1f})", ax=ax)
        print(f"{name:<14}{path_length(path):>10.2f}"
              f"{planner.iterations_used:>12}")

    plt.show()


if __name__ == "__main__":
    main()
