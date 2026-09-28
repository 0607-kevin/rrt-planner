"""Tests for the environment, tree and RRT-family planners.

Run from the project root::

    python tests/test_rrt.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rrt import RRTConnectPlanner, RRTPlanner, RRTStarPlanner, Tree
from utils import Environment
from utils.visualize import path_length


# ----------------------------------------------------------------------
# Environment
# ----------------------------------------------------------------------
def test_point_collision():
    env = Environment(10, 10)
    env.add_circle(5, 5, 1)
    env.add_rect(1, 1, 2, 2)
    assert not env.collision_free(5, 5)
    assert not env.collision_free(5.9, 5)
    assert not env.collision_free(2, 2)
    assert env.collision_free(0.5, 8)
    assert not env.collision_free(-1, 5)   # out of bounds
    assert not env.collision_free(11, 5)


def test_segment_collision():
    env = Environment(20, 10)
    env.add_circle(10, 5, 2)
    assert env.segment_free((1, 1), (5, 1))
    assert not env.segment_free((1, 5), (19, 5))  # through the circle
    assert env.segment_free((1, 9), (19, 9))      # above it


# ----------------------------------------------------------------------
# Tree
# ----------------------------------------------------------------------
def test_tree_basic_queries():
    tree = Tree()
    tree.add(0, 0)                 # 0 root
    tree.add(1, 0, parent=0)       # 1
    tree.add(2, 0, parent=1)       # 2
    tree.add(1, 1, parent=0)       # 3

    idx, d = tree.nearest(2.1, 0)
    assert idx == 2 and abs(d - 0.1) < 1e-9

    near = tree.near(0, 0, 1.5)
    assert {i for i, _ in near} == {0, 1, 3}

    path = tree.path_to_root(2)
    assert path == [(0, 0), (1, 0), (2, 0)]


def test_rewire_updates_subtree_costs():
    tree = Tree()
    tree.add(0, 0)
    i1 = tree.add(10, 0, parent=0, cost=10)
    i2 = tree.add(20, 0, parent=1, cost=20)
    i3 = tree.add(3, 0, parent=0, cost=3)

    # Rewire node 1 under the closer node 3: cost 3 + distance(3->10)=10
    tree.change_parent(i1, i3, 13)
    assert tree.nodes[i1].cost == 13
    assert tree.nodes[i2].cost == 23  # propagated: 13 + 10


# ----------------------------------------------------------------------
# Planners
# ----------------------------------------------------------------------
def assert_valid_path(env, path, start, goal):
    assert path is not None
    assert math.hypot(path[0][0] - start[0], path[0][1] - start[1]) < 1e-9
    assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) < 1e-9
    for p in path:
        assert env.collision_free(*p), f"path point in collision: {p}"
    for a, b in zip(path, path[1:]):
        assert env.segment_free(a, b), f"segment in collision: {a}->{b}"


def test_rrt_finds_path():
    env = Environment.default()
    start, goal = (3, 15), (46, 15)
    planner = RRTPlanner(env, step_size=2.0, max_iter=8000, seed=1)
    path = planner.plan(start, goal)
    assert_valid_path(env, path, start, goal)


def test_rrt_connect_finds_path():
    env = Environment.default()
    start, goal = (3, 15), (46, 15)
    planner = RRTConnectPlanner(env, step_size=2.0, max_iter=5000, seed=1)
    path = planner.plan(start, goal)
    assert_valid_path(env, path, start, goal)
    # Dual-tree methods typically need far fewer iterations.
    assert planner.iterations_used < 3000


def test_rrt_star_finds_valid_path():
    env = Environment.default()
    start, goal = (3, 15), (46, 15)
    planner = RRTStarPlanner(env, step_size=2.0, max_iter=8000, seed=1)
    path = planner.plan(start, goal)
    assert_valid_path(env, path, start, goal)

    # Every stored node cost must equal parent cost + edge length.
    for node in planner.tree.nodes[1:]:
        parent = planner.tree.nodes[node.parent]
        expected = parent.cost + math.hypot(
            node.x - parent.x, node.y - parent.y
        )
        assert abs(node.cost - expected) < 1e-9


def test_rrt_star_converges_toward_straight_line_in_the_open():
    env = Environment(50, 30)  # empty world
    start, goal = (5, 15), (45, 15)
    planner = RRTStarPlanner(env, step_size=2.0, max_iter=10000, seed=5)
    path = planner.plan(start, goal)
    assert path is not None
    # With plenty of samples the path should be close to optimal (straight).
    assert path_length(path) <= path_length([start, goal]) * 1.05


def test_endpoints_in_collision_raise():
    env = Environment.default()
    planner = RRTPlanner(env)
    try:
        planner.plan((20, 22), (46, 15))  # start inside the circle
    except ValueError:
        return
    raise AssertionError("expected ValueError for start in collision")


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"\n{len(fns)} tests passed.")


if __name__ == "__main__":
    _run_all()
