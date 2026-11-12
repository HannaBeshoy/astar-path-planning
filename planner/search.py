"""Search algorithms from the Foundations of AI course, on one generic best-first loop.

Every algorithm keeps a frontier ordered by f(n) and expands the best node:

    algorithm            f(n)                 optimal?                     complete?
    breadth-first        depth (FIFO queue)   yes, if all steps cost 1     yes
    depth-first          -depth (LIFO stack)  no                           yes on finite graphs (with visited set)
    uniform-cost         g(n)                 yes                          yes
    greedy best-first    h(n)                 no                           yes on finite graphs (with visited set)
    A*                   g(n) + h(n)          yes, if h is admissible      yes
    weighted A*          g(n) + w*h(n)        cost <= w * optimal          yes

g(n) = cost from the start, h(n) = heuristic estimate of the cost to the goal.
"""
from __future__ import annotations

import heapq
import itertools
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from .heuristics import zero


@dataclass
class Result:
    algorithm: str
    path: list | None                   # cells from start to goal, or None
    cost: float                         # path cost (inf if no path)
    expanded: int                       # nodes taken off the frontier and expanded
    generated: int                      # successors created
    max_frontier: int                   # largest frontier size (memory)
    seconds: float
    order: list = field(default_factory=list)   # expansion order, for the figures


def _path(parent: dict, goal):
    out = [goal]
    while parent[out[-1]] is not None:
        out.append(parent[out[-1]])
    return out[::-1]


def _cost(grid, path):
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += 1.0 if (a[0] == b[0] or a[1] == b[1]) else 2 ** 0.5
    return total


def best_first(grid, start, goal, f: Callable, name: str, eight: bool = True, h: Callable = zero) -> Result:
    """Graph search: a node is expanded at most once; a cheaper path to a frontier node replaces
    the old one (with a consistent heuristic the first expansion of a node is optimal)."""
    t0 = time.perf_counter()
    tie = itertools.count()                               # FIFO among equal f values
    g = {start: 0.0}
    parent = {start: None}
    frontier = [(f(0.0, h(start, goal), 0), next(tie), start, 0)]
    closed, order = set(), []
    generated, max_frontier = 0, 1
    while frontier:
        _, _, node, depth = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        order.append(node)
        if node == goal:
            p = _path(parent, goal)
            return Result(name, p, g[goal], len(closed), generated, max_frontier, time.perf_counter() - t0, order)
        for nxt, step in grid.neighbours(node, eight):
            generated += 1
            new_g = g[node] + step
            if nxt not in closed and new_g < g.get(nxt, float("inf")):
                g[nxt] = new_g
                parent[nxt] = node
                heapq.heappush(frontier, (f(new_g, h(nxt, goal), depth + 1), next(tie), nxt, depth + 1))
        max_frontier = max(max_frontier, len(frontier))
    return Result(name, None, float("inf"), len(closed), generated, max_frontier, time.perf_counter() - t0, order)


def bfs(grid, start, goal, eight=True) -> Result:
    """Breadth-first search with a FIFO queue; the goal test is done when a node is generated."""
    t0 = time.perf_counter()
    parent = {start: None}
    queue = deque([start])
    order, generated, max_frontier = [], 0, 1
    if start == goal:
        return Result("BFS", [start], 0.0, 0, 0, 1, 0.0, [start])
    while queue:
        node = queue.popleft()
        order.append(node)
        for nxt, _ in grid.neighbours(node, eight):
            generated += 1
            if nxt not in parent:
                parent[nxt] = node
                if nxt == goal:
                    p = _path(parent, goal)
                    return Result("BFS", p, _cost(grid, p), len(order), generated, max_frontier,
                                  time.perf_counter() - t0, order)
                queue.append(nxt)
        max_frontier = max(max_frontier, len(queue))
    return Result("BFS", None, float("inf"), len(order), generated, max_frontier, time.perf_counter() - t0, order)


def dfs(grid, start, goal, eight=True) -> Result:
    """Depth-first search (stack, visited set): finds a path quickly but usually not a short one."""
    t0 = time.perf_counter()
    parent = {start: None}
    stack, order, generated, max_frontier = [start], [], 0, 1
    seen = {start}
    while stack:
        node = stack.pop()
        order.append(node)
        if node == goal:
            p = _path(parent, goal)
            return Result("DFS", p, _cost(grid, p), len(order), generated, max_frontier, time.perf_counter() - t0, order)
        for nxt, _ in grid.neighbours(node, eight):
            generated += 1
            if nxt not in seen:
                seen.add(nxt)
                parent[nxt] = node
                stack.append(nxt)
        max_frontier = max(max_frontier, len(stack))
    return Result("DFS", None, float("inf"), len(order), generated, max_frontier, time.perf_counter() - t0, order)


def ucs(grid, start, goal, eight=True) -> Result:
    return best_first(grid, start, goal, lambda g, h, d: g, "Uniform-cost", eight)


def greedy(grid, start, goal, h, eight=True) -> Result:
    return best_first(grid, start, goal, lambda g, hv, d: hv, "Greedy", eight, h)


def astar(grid, start, goal, h, eight=True) -> Result:
    return best_first(grid, start, goal, lambda g, hv, d: (g + hv, hv), "A*", eight, h)


def weighted_astar(grid, start, goal, h, w=2.0, eight=True) -> Result:
    return best_first(grid, start, goal, lambda g, hv, d: g + w * hv, f"Weighted A* (w={w:g})", eight, h)


def path_cost(path) -> float:
    return _cost(None, path)
