"""Runs every algorithm on every map and writes results/compare.md and the figures in docs/img.

    python compare.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from planner import maps  # noqa: E402
from planner import search as S  # noqa: E402
from planner.heuristics import euclidean, manhattan, octile  # noqa: E402
from planner.smooth import bezier_corners, curve_is_free, shortcut  # noqa: E402

ROOT = Path(__file__).resolve().parent
MAPS = {
    "maze": maps.maze(),
    "warehouse": maps.ascii_map("warehouse"),
    "trap": maps.ascii_map("trap"),
    "random 25%": maps.random_map(),
    "LiDAR room": maps.lidar_room(),
}


def run_all(grid, s, t):
    return [
        S.bfs(grid, s, t), S.dfs(grid, s, t), S.ucs(grid, s, t),
        S.greedy(grid, s, t, octile),
        S.astar(grid, s, t, euclidean), S.astar(grid, s, t, octile),
        S.astar(grid, s, t, manhattan),
        S.weighted_astar(grid, s, t, octile, 2.0),
    ]


def label(r: S.Result, i: int) -> str:
    return r.algorithm + {4: " (euclidean)", 5: " (octile)", 6: " (manhattan, not admissible)"}.get(i, "")


def table() -> str:
    out = ["# Comparison of search algorithms", "",
           "8-connected grid, straight step 1, diagonal step sqrt(2). Cost = path length in cells; "
           "expanded = nodes taken off the frontier; memory = largest frontier.", ""]
    for name, (g, s, t) in MAPS.items():
        rs = run_all(g, s, t)
        best = min(r.cost for r in rs)
        out += [f"## {name} ({g.shape[0]}x{g.shape[1]}, {100 * (~g.blocked).mean():.0f}% free)", "",
                "| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |", "| --- | --- | --- | --- | --- | --- |"]
        for i, r in enumerate(rs):
            extra = f"+{100 * (r.cost / best - 1):.1f}%" if r.cost > best + 1e-9 else "optimal"
            out.append(f"| {label(r, i)} | {r.cost:.2f} | {extra} | {r.expanded} | {r.max_frontier} | {1000 * r.seconds:.1f} |")
        out.append("")
    return "\n".join(out)


def figure(name: str, filename: str):
    g, s, t = MAPS[name]
    rs = {"Uniform-cost (h = 0)": S.ucs(g, s, t), "Greedy best-first (f = h)": S.greedy(g, s, t, octile),
          "A* (f = g + h, octile)": S.astar(g, s, t, octile)}
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))
    for ax, (title, r) in zip(axes, rs.items(), strict=True):
        img = np.where(g.blocked, 0.15, 1.0)
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        if r.order:
            e = np.array(r.order)
            ax.scatter(e[:, 1], e[:, 0], s=4 if g.shape[0] > 60 else 14, c="#f2a65a", marker="s", linewidths=0)
        if r.path:
            p = np.array(r.path)
            ax.plot(p[:, 1], p[:, 0], color="#1f5fa8", lw=2)
        ax.plot(s[1], s[0], "o", color="#2a9d4b", ms=7)
        ax.plot(t[1], t[0], "*", color="#c23b3b", ms=12)
        ax.set_title(f"{title}\ncost {r.cost:.1f}, expanded {r.expanded}", fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.suptitle(f"{name}: expanded cells (orange) and the path found (blue)")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(ROOT / "docs/img" / filename, dpi=110)
    plt.close(fig)


def smoothing_figure():
    g, s, t = MAPS["LiDAR room"]
    r = S.astar(g, s, t, octile)
    short = shortcut(g, r.path)
    curve = bezier_corners(short, radius=4)
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.imshow(np.where(g.blocked, 0.15, 1.0), cmap="gray", vmin=0, vmax=1)
    p = np.array(r.path)
    ax.plot(p[:, 1], p[:, 0], color="#bbbbbb", lw=3, label=f"A* grid path ({len(r.path)} cells)")
    q = np.array(short)
    ax.plot(q[:, 1], q[:, 0], "o--", color="#1f5fa8", ms=4, label=f"shortcut ({len(short)} waypoints)")
    ax.plot(curve[:, 1], curve[:, 0], color="#c23b3b", lw=2,
            label="Bezier corners" + (" (collision-free)" if curve_is_free(g, curve) else ""))
    ax.legend(loc="lower left", fontsize=8)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("From grid path to drivable curve (LiDAR room, robot radius 15 cm)", fontsize=9)
    fig.tight_layout()
    fig.savefig(ROOT / "docs/img/smoothing.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    (ROOT / "results").mkdir(exist_ok=True)
    (ROOT / "docs/img").mkdir(parents=True, exist_ok=True)
    md = table()
    (ROOT / "results/compare.md").write_text(md + "\n")
    figure("LiDAR room", "lidar_room.png")
    figure("trap", "trap.png")
    smoothing_figure()
    print(md)
