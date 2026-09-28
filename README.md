# rrt-planner

![Python](https://img.shields.io/badge/Python-3.9%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Tests](https://img.shields.io/badge/tests-9%20passed-brightgreen)

Sampling-based motion planning in a **continuous 2D environment**: RRT, RRT*
and RRT-Connect, implemented from scratch with collision checking, tree
rewiring, matplotlib plots and growth animations.

Sampling-based planners are the natural choice when the configuration space
is continuous or high-dimensional — they avoid building an explicit graph and
scale far better than grid search in such settings.

## Algorithms

| Planner | Complete | Optimal | Idea |
|---------|----------|---------|------|
| **RRT** | probabilistically | no | Incrementally grows a tree toward random samples |
| **RRT\*** | probabilistically | asymptotically | Adds neighbour selection + rewiring; the path keeps improving |
| **RRT-Connect** | probabilistically | no | Two trees (start & goal) greedily extend toward each other |

Key components:

- `Environment` — circles/rectangles with point and **segment collision
  checking** (segments are densely sampled).
- `Tree` — nearest-neighbour lookup, radius neighbour search, parent rewiring
  with subtree cost propagation.
- RRT* uses the standard shrinking-ball connection radius
  `gamma * sqrt(log(n) / n)`.

## Project layout

```
rrt-planner/
├── rrt/
│   ├── tree.py          # Node / Tree data structure
│   ├── rrt.py           # RRT base
│   ├── rrt_star.py      # RRT* (parent selection + rewiring)
│   └── rrt_connect.py   # dual-tree RRT-Connect
├── utils/
│   ├── environment.py   # obstacles, collision checks
│   └── visualize.py     # plots + growth animation
├── examples/
│   ├── demo_rrt.py
│   ├── demo_rrt_star.py
│   └── demo_compare.py
└── tests/
    └── test_rrt.py
```

## Installation

```bash
git clone https://github.com/0607-kevin/rrt-planner.git
cd rrt-planner
pip install -r requirements.txt
```

## Quick start

```python
from rrt import RRTStarPlanner
from utils import Environment

env = Environment.default()
planner = RRTStarPlanner(env, step_size=2.0, max_iter=8000, seed=1)
path = planner.plan((3, 15), (46, 15))
```

Run the examples:

```bash
python examples/demo_rrt.py        # basic RRT
python examples/demo_rrt_star.py   # RRT vs RRT* path quality
python examples/demo_compare.py    # all three side by side
```

## Sample results

RRT vs RRT* (straight-line distance = 43.00):

```
RRT   path length : 63.77
RRT*  path length : 54.98
```

All three planners on the same environment:

```
planner          length  iterations
RRT               75.11         462
RRT*              56.98         462
RRT-Connect       79.54         235
```

RRT* produces the shortest path thanks to rewiring, while RRT-Connect connects
the start and goal in the fewest iterations (it is optimised for speed, not
path quality). In an empty world RRT* converges to within 5% of the straight
line as samples accumulate.

## Tests

```bash
python tests/test_rrt.py
```

The 9 tests cover collision checks, the tree structure, rewiring cost
propagation, and — for every planner — that the returned path is continuous
and collision-free. RRT* is additionally checked for node-cost consistency and
convergence toward the straight-line optimum.

## References

- LaValle, S. M. (1998). *Rapidly-exploring Random Trees: A New Tool for Path
  Planning.*
- Kuffner, J. J., LaValle, S. M. (2000). *RRT-Connect: An Efficient Approach
  to Single-Query Path Planning.*
- Karaman, S., Frazzoli, E. (2011). *Sampling-based Algorithms for Optimal
  Motion Planning.* (RRT*)

## License

Released under the [MIT License](LICENSE).
