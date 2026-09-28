"""Basic RRT on the default environment."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from rrt import RRTPlanner
from utils import Environment
from utils.visualize import path_length, plot_result


def main() -> None:
    env = Environment.default()
    start, goal = (3.0, 15.0), (46.0, 15.0)

    planner = RRTPlanner(env, step_size=2.0, max_iter=5000, seed=1)
    path = planner.plan(start, goal)
    assert path is not None, "RRT failed to find a path"

    print(f"path length   : {path_length(path):.2f}")
    print(f"iterations    : {planner.iterations_used}")
    print(f"nodes in tree : {len(planner.tree.nodes)}")

    plot_result(env, [planner.tree.edges()], path, start, goal, "RRT")
    plt.show()


if __name__ == "__main__":
    main()
