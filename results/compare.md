# Comparison of search algorithms

8-connected grid, straight step 1, diagonal step sqrt(2). Cost = path length in cells; expanded = nodes taken off the frontier; memory = largest frontier.

## maze (21x41, 48% free)

| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |
| --- | --- | --- | --- | --- | --- |
| BFS | 92.00 | optimal | 395 | 13 | 1.6 |
| DFS | 100.00 | +8.7% | 181 | 21 | 0.7 |
| Uniform-cost | 92.00 | optimal | 402 | 13 | 1.9 |
| Greedy | 96.00 | +4.3% | 103 | 16 | 0.5 |
| A* (euclidean) | 92.00 | optimal | 284 | 13 | 1.4 |
| A* (octile) | 92.00 | optimal | 282 | 13 | 1.5 |
| A* (manhattan, not admissible) | 92.00 | optimal | 252 | 13 | 1.2 |
| Weighted A* (w=2) | 92.00 | optimal | 225 | 14 | 1.1 |

## warehouse (14x30, 53% free)

| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |
| --- | --- | --- | --- | --- | --- |
| BFS | 33.07 | optimal | 212 | 12 | 1.2 |
| DFS | 62.91 | +90.2% | 156 | 100 | 0.8 |
| Uniform-cost | 33.07 | optimal | 219 | 14 | 1.5 |
| Greedy | 35.07 | +6.0% | 35 | 39 | 0.3 |
| A* (euclidean) | 33.07 | optimal | 138 | 39 | 1.1 |
| A* (octile) | 33.07 | optimal | 112 | 52 | 0.9 |
| A* (manhattan, not admissible) | 33.66 | +1.8% | 51 | 61 | 0.4 |
| Weighted A* (w=2) | 33.07 | optimal | 38 | 57 | 0.3 |

## trap (12x22, 73% free)

| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |
| --- | --- | --- | --- | --- | --- |
| BFS | 17.31 | optimal | 157 | 32 | 1.0 |
| DFS | 29.46 | +70.1% | 96 | 83 | 0.6 |
| Uniform-cost | 17.31 | optimal | 182 | 34 | 1.5 |
| Greedy | 19.07 | +10.2% | 24 | 43 | 0.2 |
| A* (euclidean) | 17.31 | optimal | 82 | 49 | 0.7 |
| A* (octile) | 17.31 | optimal | 78 | 102 | 0.9 |
| A* (manhattan, not admissible) | 17.31 | optimal | 77 | 97 | 0.7 |
| Weighted A* (w=2) | 17.31 | optimal | 34 | 59 | 0.4 |

## random 25% (60x60, 75% free)

| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |
| --- | --- | --- | --- | --- | --- |
| BFS | 97.84 | optimal | 2666 | 81 | 15.7 |
| DFS | 210.45 | +115.1% | 2367 | 691 | 13.6 |
| Uniform-cost | 97.84 | optimal | 2677 | 83 | 17.5 |
| Greedy | 104.43 | +6.7% | 106 | 141 | 0.8 |
| A* (euclidean) | 97.84 | optimal | 1569 | 175 | 11.8 |
| A* (octile) | 97.84 | optimal | 1247 | 188 | 10.8 |
| A* (manhattan, not admissible) | 98.43 | +0.6% | 184 | 170 | 1.6 |
| Weighted A* (w=2) | 101.01 | +3.2% | 116 | 141 | 0.9 |

## LiDAR room (200x200, 16% free)

| Algorithm | Cost | vs. optimal | Expanded | Memory | Time (ms) |
| --- | --- | --- | --- | --- | --- |
| BFS | 78.46 | optimal | 6404 | 198 | 45.7 |
| DFS | 489.49 | +523.9% | 388 | 793 | 3.0 |
| Uniform-cost | 78.46 | optimal | 6411 | 219 | 60.8 |
| Greedy | 88.50 | +12.8% | 84 | 179 | 1.1 |
| A* (euclidean) | 78.46 | optimal | 1118 | 267 | 10.7 |
| A* (octile) | 78.46 | optimal | 125 | 201 | 1.3 |
| A* (manhattan, not admissible) | 78.46 | optimal | 231 | 240 | 2.3 |
| Weighted A* (w=2) | 86.74 | +10.6% | 98 | 194 | 1.4 |

