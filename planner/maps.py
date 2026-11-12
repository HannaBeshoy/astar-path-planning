"""The maps used in the comparison."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .grid import Grid, from_ascii, from_scan, load_scan

DATA = Path(__file__).resolve().parents[1] / "data"

WAREHOUSE = """
##############################
#S...........................#
#............................#
#..####..####..####..####....#
#..####..####..####..####....#
#..####..####..####..####....#
#..####..####..####..####....#
#............................#
#............................#
#..####..####..####..####....#
#..####..####..####..####....#
#..####..####..####..####..G.#
#............................#
##############################
"""

# the classic case where greedy best-first is fooled: a wall facing the goal
TRAP = """
######################
#....................#
#.............#......#
#.............#......#
#.............#......#
#....S........#....G.#
#.............#......#
#.............#......#
#.............#......#
#.............#......#
#....................#
######################
"""


def maze(rows=21, cols=41, seed=3):
    """A perfect maze (exactly one route between any two cells), carved by randomized
    depth-first search, then with a few extra openings so there are several routes."""
    rng = np.random.default_rng(seed)
    blocked = np.ones((rows, cols), dtype=bool)
    stack = [(1, 1)]
    blocked[1, 1] = False
    while stack:
        r, c = stack[-1]
        options = [(r + dr, c + dc, dr, dc) for dr, dc in ((-2, 0), (2, 0), (0, -2), (0, 2))
                   if 0 < r + dr < rows - 1 and 0 < c + dc < cols - 1 and blocked[r + dr, c + dc]]
        if not options:
            stack.pop()
            continue
        nr, nc, dr, dc = options[rng.integers(len(options))]
        blocked[r + dr // 2, c + dc // 2] = False
        blocked[nr, nc] = False
        stack.append((nr, nc))
    for _ in range(rows * cols // 15):                     # open some walls: loops
        r, c = rng.integers(1, rows - 1), rng.integers(1, cols - 1)
        if (r % 2) != (c % 2):
            blocked[r, c] = False
    return Grid(blocked), (1, 1), (rows - 2, cols - 2)


def ascii_map(name: str):
    return from_ascii({"warehouse": WAREHOUSE, "trap": TRAP}[name])


def random_map(rows=60, cols=60, density=0.25, seed=0):
    rng = np.random.default_rng(seed)
    blocked = rng.random((rows, cols)) < density
    start, goal = (1, 1), (rows - 2, cols - 2)
    blocked[start] = blocked[goal] = False
    return Grid(blocked), start, goal


def lidar_room(robot_radius_m: float = 0.15):
    """The room seen by the LiDAR in lidar-scan-filtering, filtered with median-5, obstacles grown
    by the robot radius. Start: the sensor; goal: a free cell far across the room."""
    grid = from_scan(load_scan(DATA / "example_scan.csv"))
    inflated = grid.inflate(int(np.ceil(robot_radius_m / grid.resolution)))
    start = tuple(int(x) for x in grid.origin)
    reach, todo = {start}, [start]                         # cells the robot can actually reach
    while todo:
        cell = todo.pop()
        for nxt, _ in inflated.neighbours(cell):
            if nxt not in reach:
                reach.add(nxt)
                todo.append(nxt)
    far = max(reach, key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)
    return inflated, start, far
