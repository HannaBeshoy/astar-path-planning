import math

import numpy as np
import pytest

from planner import maps
from planner import search as S
from planner.grid import Grid, from_ascii
from planner.heuristics import HEURISTICS, euclidean, manhattan, octile, zero
from planner.smooth import bezier_corners, curve_is_free, line_cells, shortcut


def valid_path(grid, path, s, t):
    assert path[0] == s and path[-1] == t
    for a, b in zip(path, path[1:]):
        assert grid.free(b)
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1          # one step at a time
    return True


def dijkstra_cost(grid, s, t):
    """Independent reference: plain Dijkstra with a dict, no shared code with best_first."""
    import heapq
    dist, pq = {s: 0.0}, [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == t:
            return d
        if d > dist[u]:
            continue
        for v, w in grid.neighbours(u):
            if d + w < dist.get(v, math.inf):
                dist[v] = d + w
                heapq.heappush(pq, (d + w, v))
    return math.inf


@pytest.mark.parametrize("seed", range(30))
def test_optimal_algorithms_match_dijkstra(seed):
    g, s, t = maps.random_map(25, 25, 0.28, seed)
    ref = dijkstra_cost(g, s, t)
    for r in (S.ucs(g, s, t), S.astar(g, s, t, octile), S.astar(g, s, t, euclidean)):
        assert r.cost == pytest.approx(ref)
        if r.path:
            valid_path(g, r.path, s, t)


@pytest.mark.parametrize("seed", range(30))
def test_suboptimal_algorithms_are_bounded_and_valid(seed):
    g, s, t = maps.random_map(25, 25, 0.28, seed)
    ref = dijkstra_cost(g, s, t)
    wa = S.weighted_astar(g, s, t, octile, 2.0)
    assert wa.cost <= 2.0 * ref + 1e-9                                  # w-admissible
    for r in (S.greedy(g, s, t, octile), S.dfs(g, s, t), S.bfs(g, s, t), wa):
        assert (r.path is None) == math.isinf(ref)                      # complete
        if r.path:
            valid_path(g, r.path, s, t)
            assert r.cost >= ref - 1e-9


def test_bfs_is_optimal_in_steps_with_four_moves():
    g, s, t = maps.random_map(25, 25, 0.25, 4)
    bfs = S.bfs(g, s, t, eight=False)
    ucs = S.ucs(g, s, t, eight=False)
    assert bfs.cost == ucs.cost


def test_astar_expands_fewer_nodes_than_ucs_on_the_lidar_room():
    g, s, t = maps.lidar_room()
    a, u = S.astar(g, s, t, octile), S.ucs(g, s, t)
    assert a.cost == pytest.approx(u.cost)
    assert a.expanded * 10 < u.expanded


def test_greedy_is_fooled_by_the_trap():
    g, s, t = maps.ascii_map("trap")
    assert S.greedy(g, s, t, octile).cost > S.astar(g, s, t, octile).cost


def test_no_path_and_start_is_goal():
    g, s, t = from_ascii("#####\n#S#G#\n#####")
    for r in (S.bfs(g, s, t), S.dfs(g, s, t), S.ucs(g, s, t), S.astar(g, s, t, octile)):
        assert r.path is None and math.isinf(r.cost)
    assert S.astar(g, s, s, octile).cost == 0


def test_no_corner_cutting():
    g, s, t = from_ascii("S#\n#G")
    assert S.astar(g, s, t, octile).path is None


@pytest.mark.parametrize("seed", range(10))
def test_heuristics_admissible_and_consistent_with_eight_moves(seed):
    g, _, t = maps.random_map(20, 20, 0.2, seed)
    exact = {}
    for cell in map(tuple, np.argwhere(~g.blocked)):
        exact[cell] = dijkstra_cost(g, cell, t)
    for name in ("zero", "euclidean", "octile"):
        h = HEURISTICS[name]
        for cell, cost in exact.items():
            assert h(cell, t) <= cost + 1e-9, name                       # admissible
            for nxt, step in g.neighbours(cell):
                assert h(cell, t) <= step + h(nxt, t) + 1e-9, name       # consistent


def test_manhattan_overestimates_with_diagonals():
    assert manhattan((0, 0), (3, 3)) == 6 > octile((0, 0), (3, 3))
    assert zero((0, 0), (9, 9)) == 0


def test_inflate_and_lidar_grid():
    g = Grid(np.zeros((9, 9), dtype=bool))
    g.blocked[4, 4] = True
    assert g.inflate(2).blocked.sum() == 13                              # disc of radius 2
    room, s, t = maps.lidar_room()
    assert room.free(s) and room.free(t) and room.shape == (200, 200)


def test_shortcut_and_bezier_stay_collision_free():
    g, s, t = maps.lidar_room()
    p = S.astar(g, s, t, octile).path
    short = shortcut(g, p)
    assert len(short) < len(p) and short[0] == s and short[-1] == t
    assert curve_is_free(g, bezier_corners(short, radius=4))
    assert line_cells((0, 0), (3, 1)) == [(0, 0), (1, 0), (2, 1), (3, 1)]
