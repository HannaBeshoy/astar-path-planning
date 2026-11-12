"""Turning a grid path into something a car can drive.

1 shortcut: drop every waypoint that the robot can skip in a straight line (line of sight on
  the inflated grid), which removes the grid's zig-zag.
2 round the corners with quadratic Bezier curves (as in the bezier-path-planning project), so the
  steering does not jump.
"""
from __future__ import annotations

import numpy as np


def line_cells(a, b):
    """Cells on the segment a-b (Bresenham)."""
    (r0, c0), (r1, c1) = a, b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dr - dc
    r, c = r0, c0
    out = [(r, c)]
    while (r, c) != (r1, c1):
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
        out.append((r, c))
    return out


def visible(grid, a, b) -> bool:
    return all(grid.free(cell) for cell in line_cells(a, b))


def shortcut(grid, path):
    if not path or len(path) < 3:
        return list(path or [])
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not visible(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def bezier_corners(points, radius: float = 2.0, samples: int = 8) -> np.ndarray:
    """Replace each corner by a quadratic Bezier from `radius` cells before it to `radius` after."""
    pts = [np.array(p, dtype=float) for p in points]
    if len(pts) < 3:
        return np.array(pts)
    out = [pts[0]]
    for prev, corner, nxt in zip(pts, pts[1:], pts[2:]):
        a = corner + (prev - corner) * min(1.0, radius / max(np.linalg.norm(prev - corner), 1e-9))
        b = corner + (nxt - corner) * min(1.0, radius / max(np.linalg.norm(nxt - corner), 1e-9))
        for t in np.linspace(0, 1, samples):
            out.append((1 - t) ** 2 * a + 2 * (1 - t) * t * corner + t ** 2 * b)
    out.append(pts[-1])
    return np.array(out)


def curve_is_free(grid, curve) -> bool:
    for p, q in zip(curve, curve[1:]):
        a = (int(round(p[0])), int(round(p[1])))
        b = (int(round(q[0])), int(round(q[1])))
        if not visible(grid, a, b):
            return False
    return True
