# Phase Notes – Traffic Simulation Project

## Phase 1: Graph + Data Model
**Date:** Day 1–3

### Data Structures Used
- `Node` struct – stores id, name, x/y coordinates
- `Edge` struct – stores from/to, length, capacity, free_flow_time, flow, closed
- `Graph` class – adjacency list via `unordered_map<int, vector<Edge>>`

### BPR Function
```
t(x) = t0 * (1 + 0.15 * (x/c)^4)
```
- t0 = free-flow time
- x  = current flow (vph)
- c  = capacity (vph)
- At capacity: t = 1.15 * t0  (only 15% slower)
- At 2x capacity: t = 1.15 * 16 * t0  (much slower — congestion)

### JSON Parser (custom, no external libs)
1. `split_json_array(json, key)` – returns all `{...}` blocks inside a named array
2. `get_json_value(obj, key)` – extracts a value (string or number) by key

### Time Complexities
| Operation | Complexity |
|-----------|-----------|
| add_node  | O(1) amortised |
| add_edge  | O(1) amortised |
| load_from_json | O(V + E) |
| get_bpr_time | O(deg(v)) |
| euclidean_dist | O(1) |
| edge_count | O(V) |

### City Layout (15 nodes, 25 undirected = 50 directed edges)
See `data/sample_city.json`

---
*Next: Phase 2 – Custom Min-Heap, Dijkstra, A\**
