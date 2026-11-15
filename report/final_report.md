# Traffic Flow Analysis & Simulation System
## Academic Capstone Project Report Outline
**Course:** Data Structures & Algorithms (B.Tech 2nd Year, Computer Science & Technology)  
**Institution:** Maharaja Agrasen Institute of Technology (MAIT), GGSIPU  
**Project Repository:** [github.com/Ayush-Codes-11/Traffic_Flow_Analysis](https://github.com/Ayush-Codes-11/Traffic_Flow_Analysis)  
**Live Demonstration:** [ayush-codes-11.github.io/Traffic_Flow_Analysis](https://ayush-codes-11.github.io/Traffic_Flow_Analysis/)  

---

## 1. Problem Statement & Motivation

Urban vehicular traffic congestion leads to substantial economic losses, fuel wastage, and increased emissions. Traditional static routing fails to account for dynamic network states such as non-linear link slowdown under congestion, cyclic traffic signal delays, and sudden incident-induced road closures.

The objective of this project is to architect and implement a comprehensive, zero-dependency software system in C++17 and modern web standards that bridges:
1. **Macroscopic Traffic Assignment**: Formulating network equilibrium models (Wardrop’s Principles) to predict equilibrium traffic flows under non-linear Bureau of Public Roads (BPR) link delay functions.
2. **Microscopic Traffic Simulation**: Simulating individual vehicle trajectories, intersection physical queueing, headway separation, and cyclic signal phases using a discrete-event priority queue.
3. **Dynamic Incident Response & Telemetry Ingestion**: Detecting anomalies through sliding-window sensors, handling dropouts, smoothing readings via exponential moving averages, and dynamically rerouting vehicles.
4. **High-Performance Multi-Threading**: Parallelizing batch shortest-path queries across multi-core processors.
5. **Interactive Data Visualization**: Providing traffic engineers with real-time visual dashboards, bottleneck heatmaps, time scrubbers, and exportable engineering reports.

---

## 2. Core Data Structures & Design Justifications

Rather than relying on generic standard library containers, custom data structures were designed and implemented to maximize spatial locality, eliminate memory allocation overhead, and guarantee algorithmic bounds.

### 2.1 Adjacency List Network Graph (`src/graph.h`)
- **Structure:** Array/Vector of directed adjacency lists (`std::unordered_map<int, std::vector<Edge>>`).
- **Justification:** Road networks are planar and sparse (average node degree $d \approx 3\text{--}4$). An adjacency list guarantees $O(V + E)$ spatial complexity compared to an $O(V^2)$ adjacency matrix, enabling efficient neighbor iteration during Dijkstra relaxation.
- **BPR Cost Model:** Link travel time is evaluated in $O(1)$ time:
  $$t(x) = t_0 \left(1 + 0.15 \left(\frac{x}{c}\right)^4\right)$$

### 2.2 Binary Min-Heap with Lazy Deletion (`src/minheap.h`)
- **Structure:** Dynamic array-backed binary tree with contiguous heap storage.
- **Justification:** In graph shortest-path algorithms, relaxing an edge requires updating a node's distance. Maintaining explicit back-pointers for index-based decrease-key operations incurs substantial bookkeeping overhead. Instead, **lazy deletion** pushes updated `(node_id, priority)` pairs directly into the heap and skips stale entries upon extraction via a `visited` boolean vector.
- **Benefits:** Maximizes cache spatial locality and avoids pointer indirection, outperforming pointer-based Fibonacci or pairing heaps on graphs with fewer than 50,000 nodes.

### 2.3 Circular Ring-Buffer Vehicle Queue (`src/vehicle_queue.h`)
- **Structure:** Fixed-capacity or dynamically doubling circular array with `head` and `tail` index pointers.
- **Justification:** Used for per-edge physical queueing at intersections. Standard `std::queue<T>` (typically backed by `std::deque`) performs non-contiguous block allocations. The circular array provides guaranteed $O(1)$ amortized `enqueue`, `dequeue`, and `front` operations with zero dynamic allocations in steady-state.

### 2.4 Discrete-Event Priority Queue (`src/simulation.h`)
- **Structure:** Event-ordered min-heap prioritizing simulation events by timestamp (`double event_time`).
- **Justification:** Replaces continuous-time fixed-delta polling ($O(N \cdot \text{steps})$) with discrete-event scheduling ($O(\log K)$ per event). Events include `VEHICLE_ARRIVES_AT_EDGE`, `SIGNAL_CHECK`, `VEHICLE_EXITS_EDGE`, `INCIDENT_START`, and `INCIDENT_END`. Signal cycles advance without CPU busy waiting.

### 2.5 Thread-Safe Telemetry Queue (`src/sensor_feed.h`)
- **Structure:** Ring buffer guarded by `std::mutex` and signaled via `std::condition_variable`.
- **Justification:** Decouples sensor packet ingestion threads from the main simulation worker thread, preventing I/O stalls during high-frequency telemetry processing.

---

## 3. Algorithm Complexity & Performance Analysis

| Algorithm | Implementation File | Primary Operations | Worst-Case Time | Space Complexity | Practical Purpose |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **Custom Binary Min-Heap** | `src/minheap.h` | `push()`, `pop()`, `peek()` | $O(\log N)$ | $O(N)$ | Priority queue for Dijkstra, A*, and simulation events |
| **Dijkstra Shortest Path** | `src/dijkstra.cpp` | Edge relaxation, heap extraction | $O((V + E) \log V)$ | $O(V + E)$ | Shortest travel-time route under BPR congestion |
| **A\* Search Algorithm** | `src/astar.cpp` | Heuristic evaluation, relaxation | $O((V + E) \log V)$ | $O(V + E)$ | Fast directed routing with Euclidean lower-bound |
| **Circular Vehicle Queue** | `src/vehicle_queue.h`| `enqueue()`, `dequeue()`, `front()`| $O(1)$ | $O(C)$ | Edge-level physical queueing and signal holding |
| **All-or-Nothing (AON)** | `src/assignment.cpp` | All-pairs shortest path + flow load | $O(K \cdot (V + E) \log V)$| $O(E)$ | Baseline assignment under free-flow conditions |
| **Method of Successive Averages (MSA)** | `src/assignment.cpp` | Iterative AON + convex update | $O(M \cdot K \cdot (V + E) \log V)$| $O(E)$ | Solves User Equilibrium ($1/n$ step size) |
| **Frank-Wolfe Convex Optimization** | `src/assignment.cpp` | Direction finding + Bisection line search | $O(M \cdot K \cdot (V + E) \log V)$| $O(E)$ | Computes exact optimal step size $\lambda$; accelerates convergence |
| **Parallel Batch Router** | `src/parallel.cpp` | Multi-threaded query partitioning | $O\left(\frac{N}{T} (V + E) \log V\right)$ | $O(N + V + E)$ | High-throughput concurrent route generation |

---

## 4. Experimental Results Grounded in Continuous Integration

All reported metrics are verified directly from automated GitHub Actions CI execution logs running on headless Ubuntu Linux:

### 4.1 Macroscopic Traffic Assignment Comparison
Evaluated on the 15-node city network at standard scale factor $0.4$ (Total Network Demand = $5,520$ vehicles/hour):

| Assignment Method | Iterations | Final Relative Gap | Total Travel Time (veh-min/h) | Total Travel Time (veh-hrs) | Peak Link V/C | Over-Capacity Edges |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All-or-Nothing (AON)** | 1 | $0.000000$ | $79,554.5$ | $1,325.91$ | $1.200$ | 1 |
| **User Equilibrium (MSA)** | 7 | $0.000482$ | $77,354.1$ | $1,289.23$ | $1.020$ | 1 |
| **User Equilibrium (Frank-Wolfe)** | 3 | $0.000338$ | $77,512.4$ | $1,291.87$ | $1.041$ | 1 |
| **System Optimal (Frank-Wolfe)** | 28 | $0.000871$ | $76,667.9$ | $1,277.80$ | $0.785$ | 0 |

#### City Network Stress Test (Scale = 1.0, Extreme Peak Demand = 13,800 vph):

| Assignment Method | Iterations | Final Relative Gap | Total Travel Time (veh-min/h) | Total Travel Time (veh-hrs) | Peak Link V/C | Over-Capacity Edges |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All-or-Nothing (AON)** | 1 | $0.000000$ | $719,816.4$ | $11,996.94$ | $3.000$ | 15 |
| **User Equilibrium (Frank-Wolfe)** | 228 | $0.000928$ | $283,080.8$ | $4,718.01$ | $1.680$ | 18 |
| **System Optimal (Frank-Wolfe)** | 792 | $0.000998$ | $278,848.6$ | $4,647.48$ | $1.595$ | 18 |

#### Engineering Findings:
1. **Wardrop’s Principle Verified:** System Optimal achieves the minimum Total System Travel Time ($1,277.80$ veh-hrs at scale 0.4), saving **$14.07$ vehicle-hours per hour** compared to User Equilibrium ($1,291.87$ veh-hrs).
2. **Bottleneck Elimination:** Under UE (Frank-Wolfe, scale 0.4), the single over-capacity link is **Edge 0→5 (Central_Station → Business_District)** with an assigned flow of $2,082$ vph against a design capacity of $2,000$ vph (Peak V/C = $1.041$). Under System Optimal, marginal social cost pricing shifts traffic onto alternative routes, reducing flow on Edge 0→5 to $1,570$ vph (V/C = $0.785$) and eliminating all over-capacity bottlenecks network-wide.
3. **Frank-Wolfe Efficiency:** Frank-Wolfe UE achieves convergence ($\text{RelGap} < 10^{-3}$) in only **3 iterations**, whereas MSA required **7 iterations**, confirming the mathematical advantage of line search over harmonic step sizing. Under stress testing (13,800 vph), Frank-Wolfe UE converged in 228 iterations while cutting total system delay from $11,996.94$ veh-hrs (AON) down to $4,718.01$ veh-hrs ($-60.7\%$).

---

### 4.2 Dynamic Incident & Rerouting Experiment
Evaluated under deterministic random seeds on the 15-node, 50-directed-edge city network:

#### Scenario 1: Moderate Load (50 Vehicles, Edge 0→13 Closed $t = 10 \dots 25$ min)
- **Closure Window:** 15 minutes.
- **Results:**
  - Completed Vehicles: **50 / 50** in both modes.
  - Average Travel Time: **$15.09$ min** (Without Rerouting) vs **$15.09$ min** (With Rerouting).
  - Maximum Individual Travel Time: Reduced from **$35.64$ min** down to **$32.97$ min** ($-7.5\%$).
  - Total Queue Wait Time: Reduced from **$272.67$ min** down to **$258.47$ min** ($-5.2\%$).

#### Scenario 2: Heavy Load & Sustained Closure (100 Vehicles, Edge 0→13 Closed $t = 5 \dots 40$ min)
- **Closure Window:** 35 minutes across the primary arterial corridor (Central Station $\to$ Old Market).
- **Results:**
  - Completed Vehicles: **100 / 100** in both modes.
  - Average Travel Time: Reduced from **$14.83$ min** down to **$14.35$ min** ($-3.2\%$).
  - Maximum Individual Travel Time: Reduced from **$48.97$ min** down to **$34.79$ min** (**$-28.9\%$**).
  - Total Queue Wait Time: Reduced from **$542.89$ min** down to **$486.29$ min** (**$-10.4\%$**).

#### Engineering Analysis & Secondary Congestion Transfer:
- In Scenario 2, rerouting vehicles around the closed arterial (Edge 0→13, capacity $2,000$ vph) diverted flow onto secondary corridors (e.g., via Nodes 4, 10, 2, and 7).
- Inspection of `data/sample_city.json` confirms that these secondary detour roads have strictly lower design capacities ($1,200\text{--}1,800$ vph vs $2,000$ vph on arterial 0→13):
  - Edge 0→4 (Shopping Mall): capacity **$1,800$ vph**
  - Edge 4→2 (University): capacity **$1,600$ vph**
  - Edge 2→13 (Old Market): capacity **$1,600$ vph**
  - Edge 2→7 (North Suburb): capacity **$1,200$ vph**
  - Edge 7→13 (Old Market): capacity **$1,200$ vph**
- Under heavy demand (100 vehicles), this diverted volume causes **secondary congestion transfer** onto these narrower side roads. 
- However, dynamic A* rerouting successfully prevented localized gridlock, capped the maximum individual delay from $48.97$ min to $34.79$ min, and eliminated stranded queues.

#### Worked Example: Real Dynamic Rerouting Trace (Scenario 2)
The discrete-event simulation engine logs real vehicle trajectory traces during execution. The following trace represents the first completed rerouted vehicle in Scenario 2 as recorded directly by the C++ engine:

```text
====================================================================
  REAL REROUTING TRACE: Scenario 2 (Vehicle ID 16)
====================================================================
  Vehicle ID        : 16
  Trip OD           : Node 12 (Stadium) -> Node 3 (Hospital)
  Spawn Time        : 0.33 min
  Incident Active   : Edge 0 -> 13 closed between t=5.00 min and t=40.00 min
  Reroute Event     : Triggered at t=5.00 min at Node 6 (City_Park)
  Original Route    : 12 (Stadium) -> 6 (City_Park) -> 0 (Central_Station) -> 13 (Old_Market) -> 3 (Hospital)
  Detour Route      : 12 (Stadium) -> 6 (City_Park) -> 0 (Central_Station) -> 5 (Business_District) -> 3 (Hospital)
  Finish Time       : 26.00 min
  Total Travel Time : 25.67 min
  Total Wait Time   : 9.67 min
====================================================================
```

*Trace Commentary:* Vehicle 16 spawned at $t=0.33$ min with an original route heading through City Park (6) and Central Station (0) to Old Market (13) and Hospital (3). At $t=5.00$ min, while Vehicle 16 was traversing towards Central Station, Edge $0 \to 13$ was closed by Incident 202. The simulation detected the impending closure at the node level and triggered a dynamic A* search from Node 6. The route was updated to divert via Node 5 (Business District), traversing $0 \to 5 \to 3$ instead of waiting for the arterial to reopen at $t=40.00$ min. The vehicle arrived safely at $t=26.00$ min with total travel time $25.67$ min, avoiding over $30$ minutes of road-closure gridlock.

---

### 4.3 Multi-Threaded Batch Routing Scalability
Evaluated across 5,000 random origin-destination queries using `std::thread` data-parallel partitioning:

#### Part A: 15-Node City Network
- **1 Thread:** $9.57$ ms ($522,613$ queries/sec)
- **2 Threads:** $10.21$ ms ($0.98\times$ speedup)
- **4 Threads:** $6.93$ ms ($1.44\times$ speedup, $721,267$ queries/sec)
- **Analysis:** Individual Dijkstra queries on a 15-node graph execute in $\sim 1\text{--}3$ microseconds. Thread invocation overhead, cache migration, and thread synchronization account for a noticeable fraction of total runtime on small networks. All parallel query path costs matched sequential results with $100\%$ precision.

#### Part B: Synthetic 40x40 Grid Network (1,600 Nodes, 6,240 Edges)
- **1 Thread:** $1,367.59$ ms ($3,656$ queries/sec)
- **2 Threads:** $686.13$ ms ($2.00\times$ speedup)
- **4 Threads:** $591.21$ ms ($2.32\times$ speedup, $8,457$ queries/sec)
- **Analysis:** With 1,600 nodes, each path search explores hundreds of heap nodes ($\sim 100\text{--}300\ \mu\text{s}$ per query). Computation heavily dominates thread dispatch, achieving a measured speedup of **$2.32\times$ on 4 threads** (parallel efficiency: **$57.9\% = 2.32 / 4 \times 100\%$**) on shared virtualized CI runners. All 5,000 path costs produced by parallel thread pools matched the sequential baseline exactly, verifying algorithmic equivalence.

---

## 5. Web Architecture & Data Flow

The Phase 6 visualization dashboard operates without server-side compute or external JavaScript dependencies:
1. **Compilation Export:** During test execution, `TrafficAssignment::save_all_scenarios_to_json()` and `export_unified_web_data()` serialize the graph topology, assignment results, alert logs, and time-series telemetry into `data/` and `web/data.js`.
2. **CORS-Free Execution:** By attaching the data payload to `window.TRAFFIC_DATA`, the user can launch `web/index.html` via the `file://` protocol simply by double-clicking the file in any browser without triggering local origin security errors.
3. **Interactive UI Features:**
   - Standalone SVG renderer dynamically projects node cartesian coordinates and separates bidirectional opposing road links with a 4-pixel parallel offset.
   - HTML5 Canvas 2D engine plots network-average V/C ratios over time with an interactive scrubber synchronized to the road network map.
   - Client-side Dijkstra in `web/logic.js` allows interactive point-to-point route planning reflecting active closures.
   - `@media print` CSS formats the dashboard into an academic report layout.

---

## 6. Limitations & Future Extensions

1. **Static Signal Timing:** Current signals operate on fixed-duration cycle splits. Future work can implement adaptive actuation (e.g., Webster’s formula or GreenWave synchronization).
2. **Lane-Level Micro-Sim:** Current edge queues model aggregate link physical storage. Future iterations can model multi-lane overtaking and lane-changing behavior (e.g., MOBIL/IDM models).
3. **GPU-Accelerated Assignment:** For continental networks ($> 100,000$ links), all-pairs shortest paths could be ported to CUDA/OpenCL tensor kernels.

---

## 7. Conclusion

This capstone project successfully integrates rigorous theoretical transportation modeling with high-performance C++ software engineering. By implementing custom data structures (MinHeap, Circular Queue, Discrete Event Engine), convex optimization assignment models (Frank-Wolfe), dynamic sensor alerting, and an interactive zero-dependency web dashboard, the system provides a robust platform for urban traffic analysis and incident mitigation.
