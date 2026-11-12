"""Heuristics h(n): estimates of the remaining cost from a cell to the goal.

admissible  h(n) <= true cost   -> A* returns an optimal path
consistent  h(n) <= step(n, n') + h(n')  for every step -> A* never re-expands a node

On a grid with 8 moves (straight 1, diagonal sqrt 2):
    octile     exact cost on an empty grid: admissible and consistent (the best choice)
    euclidean  straight-line distance: admissible and consistent, but less informed
    manhattan  admissible only with 4 moves; with diagonals it overestimates
    zero       h = 0 turns A* into uniform-cost search
"""
from __future__ import annotations

import math


def zero(a, b) -> float:
    return 0.0


def manhattan(a, b) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def octile(a, b) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


HEURISTICS = {"zero": zero, "manhattan": manhattan, "euclidean": euclidean, "octile": octile}
