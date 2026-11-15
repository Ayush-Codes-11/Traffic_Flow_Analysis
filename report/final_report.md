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
2. **Bottleneck Elimination:** Under UE (Frank-Wolfe, scale 0.4), the single over-capacity link is **Edge 0→5 (Central_Station → Business_District)** with an assigned flow of $2,081.6$ vph against a design capacity of $2,000.0$ vph (Peak V/C = $1.041$). Under System Optimal, marginal social cost pricing shifts traffic onto alternative routes, reducing flow on Edge 0→5 down to $1,570.4$ vph (V/C = $0.785$) and eliminating all over-capacity bottlenecks network-wide:

##### Edge 0→5 (Central_Station → Business_District, Capacity = 2,000.0 vph) Link Telemetry:
| Method | Flow (vph) | Capacity (vph) | V/C Ratio | Bottleneck Status |
| :--- | :---: | :---: | :---: | :--- |
| **All-or-Nothing (AON)** | $2,400.0$ | $2,000.0$ | $1.200$ | OVER CAPACITY (Bottleneck) |
| **User Equilibrium (MSA)** | $2,040.0$ | $2,000.0$ | $1.020$ | OVER CAPACITY (Bottleneck) |
| **User Equilibrium (Frank-Wolfe)** | $2,081.6$ | $2,000.0$ | $1.041$ | OVER CAPACITY (Bottleneck) |
| **System Optimal (Frank-Wolfe)** | $1,570.4$ | $2,000.0$ | $0.785$ | Under Capacity (Optimal) |

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

##### Top 10 Busiest Edges during Closure Window ($t = 10.0 \dots 25.0$ min):
| Rank | Edge | Edge Description | Entries (Rerouting ON) | Entries (Rerouting OFF) | Telemetry Observation |
| :---: | :---: | :--- | :---: | :---: | :--- |
| 1 | 6→12 | City_Park → Stadium | 5 | 3 | Alternative eastern transit path |
| 2 | 0→4 | Central_Station → Shopping_Mall | 4 | 0 | Traffic diverted away from closed arterial 0→13 |
| 3 | 0→6 | Central_Station → City_Park | 4 | 4 | Stable primary egress corridor |
| 4 | 12→4 | Stadium → Shopping_Mall | 4 | 0 | Diversion path connecting stadium to shopping mall |
| 5 | 5→0 | Business_District → Central_Station | 3 | 3 | Inbound commute unaffected |
| 6 | 9→6 | South_Suburb → City_Park | 3 | 3 | South-to-center transit unaffected |
| 7 | 14→6 | Industrial_Zone → City_Park | 3 | 3 | Industrial freight transit unaffected |
| 8 | 2→13 | University → Old_Market | 2 | 2 | Northern market access unaffected |
| 9 | 4→2 | Shopping_Mall → University | 2 | 2 | Secondary cross-link unaffected |
| 10 | 4→12 | Shopping_Mall → Stadium | 2 | 2 | Cross-town route unaffected |

#### Scenario 2: Heavy Load & Sustained Closure (100 Vehicles, Edge 0→13 Closed $t = 5 \dots 40$ min)
- **Closure Window:** 35 minutes across the primary arterial corridor (Central Station $\to$ Old Market).
- **Results:**
  - Completed Vehicles: **100 / 100** in both modes.
  - Average Travel Time: Reduced from **$14.83$ min** down to **$14.35$ min** ($-3.2\%$).
  - Maximum Individual Travel Time: Reduced from **$48.97$ min** down to **$34.79$ min** (**$-28.9\%$**).
  - Total Queue Wait Time: Reduced from **$542.89$ min** down to **$486.29$ min** (**$-10.4\%$**).

##### Top 10 Busiest Edges during Closure Window ($t = 5.0 \dots 40.0$ min):
| Rank | Edge | Edge Description | Entries (Rerouting ON) | Entries (Rerouting OFF) | Telemetry Observation |
| :---: | :---: | :--- | :---: | :---: | :--- |
| 1 | 2→13 | University → Old_Market | 8 | 8 | Northern market ingress corridor |
| 2 | 4→2 | Shopping_Mall → University | 7 | 6 | Secondary link serving detoured trips |
| 3 | 0→4 | Central_Station → Shopping_Mall | 6 | 0 | Primary detour away from closed arterial 0→13 |
| 4 | 5→0 | Business_District → Central_Station | 6 | 7 | Inbound business traffic |
| 5 | 6→12 | City_Park → Stadium | 6 | 5 | Alternative bypass corridor |
| 6 | 12→6 | Stadium → City_Park | 6 | 6 | Return bypass corridor |
| 7 | 13→0 | Old_Market → Central_Station | 6 | 6 | Reverse arterial unaffected |
| 8 | 6→9 | City_Park → South_Suburb | 5 | 5 | Outbound suburban transit |
| 9 | 10→4 | West_Suburb → Shopping_Mall | 5 | 5 | Suburban feeder corridor |
| 10 | 11→5 | Tech_Park → Business_District | 5 | 5 | Tech park commuter flow |

#### Engineering Analysis of Rerouting Dynamics (Grounded in Empirical Telemetry):
- **Grounded Empirical Finding:** Rerouting reduced worst-case delay and total wait; average travel time improved in Scenario 2 ($14.83 \to 14.35$ min) and stayed the same in Scenario 1 ($15.09 \to 15.09$ min).
- **Why Average Travel Time Improves Little in Scenario 1:** 
  The telemetry decomposition in `results_summary.txt` explains this phenomenon:
  - In Scenario 1, only **11 out of 50 vehicles** planned trips using Edge 0→13. 
  - The **39 unaffected vehicles** had nearly identical travel times (**$12.78$ min** OFF vs **$12.79$ min** ON).
  - The **11 affected vehicles** averaged **$23.31$ min** OFF vs **$23.24$ min** ON.
  - Averaged across all 50 vehicles, the benefit is diluted across the majority unaffected population, resulting in a rounded network average of **$15.09$ min** in both modes.
- **Why Scenario 2 Shows Measurable Average Improvement ($14.83 \to 14.35$ min):**
  - In Scenario 2, closure was active for 35 minutes ($t=5 \dots 40$ min).
  - The **8 affected vehicles** had their average travel time slashed from **$30.20$ min** down to **$24.20$ min** (**$-6.00$ min / $-19.9\%$** reduction per affected vehicle).
  - The **92 unaffected vehicles** remained constant (**$13.49$ min** OFF vs **$13.50$ min** ON).
  - This 6-minute concentrated benefit across 8 vehicles lowers the overall network-wide average from $14.83$ min to $14.35$ min.
- **Why Maximum Individual Delay Improves Significantly:** Without dynamic rerouting, vehicles arriving at the closed edge must wait idly in the physical edge queue for the entire remainder of the closure window (up to 15 minutes in Scenario 1 and up to 35 minutes in Scenario 2) before the road reopens. Dynamic A* rerouting immediately detects the closure and dispatches vehicles onto open alternative corridors, capping the worst-case individual delay from $35.64$ min down to $32.97$ min in Scenario 1 ($-7.5\%$) and from $48.97$ min down to $34.79$ min in Scenario 2 ($-28.9\%$).

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
- **1 Thread:** $7.34$ ms ($681,431$ queries/sec)
- **2 Threads:** $4.64$ ms ($1.68\times$ speedup)
- **4 Threads:** $3.46$ ms ($2.25\times$ speedup, $1,446,386$ queries/sec)
- **Analysis:** Individual Dijkstra queries on a 15-node graph execute in $\sim 1\text{--}3$ microseconds. Thread invocation overhead, cache migration, and thread synchronization account for a noticeable fraction of total runtime on small networks. All parallel query path costs matched sequential results with $100\%$ precision.

#### Part B: Synthetic 40x40 Grid Network (1,600 Nodes, 6,240 Edges)
- **1 Thread:** $1,203.36$ ms ($4,155$ queries/sec)
- **2 Threads:** $622.26$ ms ($1.96\times$ speedup)
- **4 Threads:** $429.76$ ms ($2.84\times$ speedup, $11,634$ queries/sec)
- **Analysis:** With 1,600 nodes, each path search explores hundreds of heap nodes ($\sim 100\text{--}300\ \mu\text{s}$ per query). Computation heavily dominates thread dispatch, achieving a measured speedup of **$2.84\times$ on 4 threads** (parallel efficiency: **$70.9\% = 2.84 / 4 \times 100\%$**) on Ubuntu Linux runners. All 5,000 path costs produced by parallel thread pools matched the sequential baseline exactly, verifying algorithmic equivalence.

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
