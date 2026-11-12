# Project guide

## 1. The idea in one paragraph

Path finding is a search problem: states are free cells, actions are the 8 moves, and the cost is the distance driven. Every algorithm of the course keeps a **frontier** of cells it has reached but not explored, and repeatedly expands the "best" one. They differ only in what "best" means, and `planner/search.py` shows that by running all of them through one loop with a different priority f(n).

## 2. Walk through A* on a small example

```text
S . . #
. # . #
. # . G
```

Start S = (0,0), goal G = (2,3). With 4 moves and h = Manhattan:

1. Expand S: f = 0 + 5. Successors (0,1) f = 1 + 4 = 5 and (1,0) f = 1 + 4 = 5.
2. Ties are broken first-in-first-out: expand (0,1), then (0,2) with f = 2 + 3 = 5, ...
3. Every cell on the shortest path has f = 5 = the true cost. The dead end (1,0), (2,0) also has f = 5 (1 + 4 and 2 + 3), so whether A* looks into it depends only on tie-breaking. No cell with f > 5 is ever expanded.

That is the whole point of A*: with a good h, only cells that might lie on an optimal path are expanded (here: every cell with f ≤ 5).

## 3. Admissible vs consistent

- **Admissible:** h never overestimates. Then A* is optimal: the goal cannot be expanded while a cheaper path is still waiting on the frontier.
- **Consistent:** h(n) ≤ cost(n, n′) + h(n′). Then f never decreases along a path, so the first time a cell is expanded its cost is final (no re-expansion).
- `tests/test_search.py::test_heuristics_admissible_and_consistent_with_eight_moves` computes the exact cost from every free cell and checks both conditions for zero, Euclidean and octile.
- Manhattan counts a diagonal step as 2 instead of √2, so it overestimates. With it, A* returned a 1.8 % longer path on the warehouse map: a concrete example of why admissibility matters.

## 4. Reading the comparison

| Observation | Why |
| --- | --- |
| BFS cost = optimal on most maps | all straight steps cost 1; diagonals make it slightly wrong in general |
| DFS finds long paths | it follows one branch as deep as possible |
| UCS expands almost the whole room | it has no idea where the goal is: a circle growing from the start |
| Greedy expands very few cells but takes a longer route | it ignores the cost already driven; the trap map shows it running into the wall |
| A* octile expands 51× fewer cells than UCS, same path | the heuristic focuses the search towards the goal |
| Weighted A* is in between | trusting h more is faster but no longer optimal |

## 5. Likely questions

| Question | Answer |
| --- | --- |
| Why inflate the obstacles? | The car is not a point; growing obstacles by its radius lets us plan for a point (configuration space). |
| Why are unknown cells blocked? | The car should only drive where the LiDAR has seen free space. |
| When is greedy best-first a good choice? | When any path is fine and speed matters, such as a first guess or re-planning. |
| What would you use on the real car? | A* with octile on the inflated grid, then smoothing; re-plan when a new scan arrives. |
| Time complexity of A*? | Exponential in the worst case, like UCS; the heuristic reduces the effective branching factor. |

## 6. Next steps

- [ ] Re-plan on every new scan as the car moves (D* Lite or simply A* again)
- [ ] Use the real scans from the thesis LiDAR
- [ ] Add the car's turning radius (hybrid A* on (x, y, heading) states)
- [ ] Run on the Raspberry Pi and measure time per plan
