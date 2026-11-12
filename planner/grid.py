"""Occupancy grids: the state space of the search problem.

A cell is (row, col). True = blocked. Grids come from ASCII maps or from a 2D LiDAR scan
(the same CSV format as the lidar-scan-filtering project: angle_deg,range_m[,quality]).
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Grid:
    blocked: np.ndarray                 # bool [rows, cols]
    resolution: float = 0.05            # metres per cell
    origin: tuple = (0, 0)              # cell of the sensor / world (0, 0)

    @property
    def shape(self):
        return self.blocked.shape

    def inside(self, cell) -> bool:
        r, c = cell
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1]

    def free(self, cell) -> bool:
        return self.inside(cell) and not self.blocked[cell]

    def neighbours(self, cell, eight: bool = True):
        """Successor function: (next cell, step cost). Diagonal steps cost sqrt(2) and may not
        cut a blocked corner (the robot cannot squeeze between two diagonal obstacles)."""
        r, c = cell
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            n = (r + dr, c + dc)
            if self.free(n):
                yield n, 1.0
        if eight:
            for dr, dc in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
                n = (r + dr, c + dc)
                if self.free(n) and self.free((r + dr, c)) and self.free((r, c + dc)):
                    yield n, math.sqrt(2)

    def inflate(self, radius_cells: int) -> "Grid":
        """Configuration space: grow every obstacle by the robot's radius, so the robot can be
        planned as a single point."""
        if radius_cells <= 0:
            return Grid(self.blocked.copy(), self.resolution, self.origin)
        out = self.blocked.copy()
        rr, cc = np.nonzero(self.blocked)
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    r2, c2 = rr + dr, cc + dc
                    ok = (r2 >= 0) & (r2 < self.shape[0]) & (c2 >= 0) & (c2 < self.shape[1])
                    out[r2[ok], c2[ok]] = True
        return Grid(out, self.resolution, self.origin)


def from_ascii(text: str) -> tuple:
    """'#' blocked, '.' free, 'S' start, 'G' goal. Returns (grid, start, goal)."""
    rows = [line for line in text.strip("\n").splitlines()]
    width = max(len(r) for r in rows)
    blocked = np.zeros((len(rows), width), dtype=bool)
    start = goal = None
    for r, line in enumerate(rows):
        for c, ch in enumerate(line.ljust(width, "#")):
            blocked[r, c] = ch == "#"
            if ch == "S":
                start = (r, c)
            elif ch == "G":
                goal = (r, c)
    return Grid(blocked), start, goal


def load_scan(path: Path) -> np.ndarray:
    """[N, 2] array of (angle in radians, range in metres); 0 or missing ranges are dropouts."""
    rows = []
    with open(path, newline="") as f:
        for rec in csv.DictReader(f):
            rng = float(rec["range_m"])
            if rng > 30:                                # millimetres
                rng /= 1000
            rows.append((math.radians(float(rec["angle_deg"])), rng))
    return np.array(rows)


def median5(ranges: np.ndarray) -> np.ndarray:
    """The filter chosen in lidar-scan-filtering: circular median of 5, dropouts ignored."""
    n = len(ranges)
    out = ranges.copy()
    for i in range(n):
        win = [ranges[(i + k) % n] for k in range(-2, 3)]
        win = [w for w in win if w > 0]
        out[i] = float(np.median(win)) if win else 0.0
    return out


def from_scan(scan: np.ndarray, resolution: float = 0.05, size_m: float = 10.0, max_range: float = 8.0) -> Grid:
    """Occupancy grid from one 360-degree scan, sensor in the centre.
    Cells along each beam are free; the cell where it ends is blocked; cells never crossed by a
    beam (behind walls, outside the room) are blocked too, so the planner stays where it has seen."""
    n = int(round(size_m / resolution))
    centre = (n // 2, n // 2)
    seen_free = np.zeros((n, n), dtype=bool)
    hit = np.zeros((n, n), dtype=bool)
    ranges = median5(scan[:, 1])
    for (angle, _), rng in zip(scan, ranges, strict=True):
        if rng <= 0 or rng > max_range:
            continue
        steps = int(rng / (resolution / 2))
        for k in range(steps):
            d = k * resolution / 2
            r = centre[0] - int(round(d * math.sin(angle) / resolution))
            c = centre[1] + int(round(d * math.cos(angle) / resolution))
            if 0 <= r < n and 0 <= c < n:
                seen_free[r, c] = True
        r = centre[0] - int(round(rng * math.sin(angle) / resolution))
        c = centre[1] + int(round(rng * math.cos(angle) / resolution))
        if 0 <= r < n and 0 <= c < n:
            hit[r, c] = True
    blocked = ~seen_free | hit
    return Grid(blocked, resolution, centre)
