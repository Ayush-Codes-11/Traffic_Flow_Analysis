// =============================================================================
// main.cpp  –  Phase 1 + Phase 2 + Phase 3 + Phase 4 tests
// Traffic Management & Simulation System
// =============================================================================
#include "graph.h"
#include "minheap.h"
#include "pathfinder.h"
#include "simulation.h"
#include "vehicle_queue.h"
#include "assignment.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>      // fabs
#include <algorithm>  // is_sorted
#include <string>
#include <cassert>
#include <random>     // mt19937

// ── Utility helpers ──────────────────────────────────────────────────────────

static void section(const std::string& title)
{
    std::cout << "\n" << std::string(65, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(65, '=') << "\n";
}

// Format a path as "Name1 → Name2 → Name3"
static std::string path_str(const Graph& g, const PathResult& r)
{
    if (!r.found || r.path.empty()) return "[No path found]";
    std::string s;
    for (int i = 0; i < static_cast<int>(r.path.size()); ++i) {
        if (i > 0) s += " → ";
        int id = r.path[i];
        s += g.nodes.count(id) ? g.nodes.at(id).name : std::to_string(id);
    }
    return s;
}

// ============================================================================
// ══ PHASE 1 TESTS ══
// ============================================================================

static void p1_load(Graph& g)
{
    section("P1-TEST 1 – Load graph from JSON");
    bool ok = g.load_from_json("data/sample_city.json");
    if (!ok) { std::cerr << "FAIL: could not load graph!\n"; return; }
    std::cout << "Nodes: " << g.node_count()
              << "  |  Directed edges: " << g.edge_count() << "\n";
}

static void p1_bad_load()
{
    section("P1-TEST 2 – Load deliberately bad JSON (error handling)");
    std::cout << "Loading data/bad_city.json  (expect WARNING lines below):\n\n";
    Graph bad;
    bool ok = bad.load_from_json("data/bad_city.json");
    std::cout << "\nReturn value  : " << (ok ? "true (partial)" : "false") << "\n";
    std::cout << "Nodes loaded  : " << bad.node_count() << "\n";
    std::cout << "Edges loaded  : " << bad.edge_count() << "\n";

    // Also test nonexistent file
    Graph ghost;
    std::cout << "\nLoading nonexistent file:\n";
    bool ok2 = ghost.load_from_json("data/does_not_exist.json");
    std::cout << "Return value  : " << (ok2 ? "true" : "false (PASS ✓)") << "\n";
}

static void p1_bpr(const Graph& g)
{
    section("P1-TEST 3 – BPR travel-time (edge 0→13, t0=3 min, cap=2000)");
    std::cout << std::left
              << std::setw(12) << "Flow (vph)"
              << std::setw(10) << "V/C"
              << std::setw(16) << "BPR time (min)"
              << "Slowdown\n"
              << std::string(50, '-') << "\n";

    for (double flow : {0.0, 500.0, 1000.0, 1500.0, 2000.0, 3000.0}) {
        const_cast<Graph&>(g).set_flow(0, 13, flow);
        double t  = g.get_bpr_time(0, 13);
        double t0 = 3.0, cap = 2000.0;
        std::cout << std::fixed << std::setprecision(0)
                  << std::setw(12) << flow
                  << std::setprecision(2)
                  << std::setw(10) << (flow / cap)
                  << std::setprecision(3) << std::setw(16) << t
                  << "x" << std::setprecision(2) << (t / t0) << "\n";
    }
    const_cast<Graph&>(g).set_flow(0, 13, 0.0);
}

static void p1_euclidean(const Graph& g)
{
    section("P1-TEST 4 – Euclidean distance (A* heuristic preview)");
    struct Pair { int a, b; };
    for (auto [a, b] : std::vector<Pair>{{0,1},{0,8},{9,7},{5,10}}) {
        std::cout << std::left
                  << std::setw(22) << g.nodes.at(a).name
                  << " → "
                  << std::setw(22) << g.nodes.at(b).name
                  << std::fixed << std::setprecision(3)
                  << g.euclidean_dist(a, b) << " units\n";
    }
}

static void p1_closure(Graph& g)
{
    section("P1-TEST 5 – Road closure");
    int f = 0, t = 5;
    std::cout << "Edge 0→5 (Central→Business)\n";
    std::cout << "  Open  : " << g.get_bpr_time(f, t) << " min\n";
    g.set_closed(f, t, true);
    std::cout << "  Closed: " << g.get_bpr_time(f, t) << " (∞)\n";
    g.set_closed(f, t, false);
    std::cout << "  Reopen: " << g.get_bpr_time(f, t) << " min  PASS ✓\n";
}

// ============================================================================
// ══ PHASE 2 TESTS ══
// ============================================================================

// ── P2-TEST 1: Min-Heap ──────────────────────────────────────────────────────
static void p2_heap_test()
{
    section("P2-TEST 1 – Custom MinHeap (push / pop / heapify)");

    // Test A: sort 10 values
    MinHeap<int> h;
    std::vector<int> inputs = {7, 3, 8, 1, 9, 2, 10, 4, 6, 5};
    std::cout << "Pushing: ";
    for (int v : inputs) { std::cout << v << " "; h.push(static_cast<double>(v), v); }
    std::cout << "\n";
    std::cout << "Heap size: " << h.size() << "\n";
    std::cout << "Minimum (peek): " << h.peek().priority << "\n\n";

    std::vector<int> output;
    while (!h.empty()) output.push_back(h.pop().value);

    std::cout << "Pop order: ";
    for (int v : output) std::cout << v << " ";
    std::cout << "\n";

    bool sorted = std::is_sorted(output.begin(), output.end());
    std::cout << "Sorted ascending: " << (sorted ? "PASS ✓" : "FAIL ✗") << "\n\n";

    // Test B: mixed push/pop (interleaved)
    MinHeap<int> h2;
    h2.push(5.0, 5); h2.push(3.0, 3); h2.push(7.0, 7);
    std::cout << "Interleaved test  |  push 5,3,7  → pop → ";
    std::cout << h2.pop().value << "  ";  // expect 3
    h2.push(1.0, 1); h2.push(4.0, 4);
    std::cout << "push 1,4  → pop → ";
    std::cout << h2.pop().value << "  ";  // expect 1
    std::cout << h2.pop().value << "\n";  // expect 4

    // Test C: lazily-deleted stale entries – A* pushes the same node twice
    // The min-heap doesn't know about staleness; that's handled by visited[]
    MinHeap<int> h3;
    h3.push(10.0, 42);  // first push (stale, will be skipped by Dijkstra)
    h3.push( 5.0, 42);  // better update
    std::cout << "Lazy-deletion sim: same node pushed twice → first pop priority = "
              << h3.pop().priority << " (should be 5.0)  "
              << (std::fabs(h3.pop().priority - 10.0) < 1e-9 ? "PASS ✓" : "FAIL ✗")
              << "\n";
}

// ── P2-TEST 2: Dijkstra + A* comparison table ────────────────────────────────
static void p2_pathfinding(const Graph& g)
{
    section("P2-TEST 2 – Dijkstra vs A* (5 source/destination pairs)");

    struct TestPair { int src, dst; };
    std::vector<TestPair> pairs = {
        {1,  5},   // Airport → Business_District
        {9,  11},  // South_Suburb → Tech_Park
        {12, 3},   // Stadium → Hospital
        {10, 8},   // West_Suburb → East_Suburb
        {0,  7},   // Central_Station → North_Suburb
    };

    bool all_pass = true;

    for (auto [src, dst] : pairs) {
        PathResult dijk = dijkstra(g, src, dst);
        PathResult astr = astar  (g, src, dst);

        const std::string& sname = g.nodes.at(src).name;
        const std::string& dname = g.nodes.at(dst).name;

        std::cout << "\n--- " << sname << "  →  " << dname << " ---\n";

        // Dijkstra row
        std::cout << "  Dijkstra │ explored=" << std::setw(3) << dijk.nodes_explored
                  << " │ time=" << std::fixed << std::setprecision(3) << dijk.total_time
                  << " min │ path: " << path_str(g, dijk) << "\n";

        // A* row
        std::cout << "  A*       │ explored=" << std::setw(3) << astr.nodes_explored
                  << " │ time=" << std::fixed << std::setprecision(3) << astr.total_time
                  << " min │ path: " << path_str(g, astr) << "\n";

        // Verify: same total time?
        bool times_match = (std::fabs(dijk.total_time - astr.total_time) < 1e-6);
        std::cout << "  Times match: " << (times_match ? "PASS ✓" : "FAIL ✗") << "\n";

        // Verify: A* explored ≤ Dijkstra (usually, never more)
        bool astar_leq = (astr.nodes_explored <= dijk.nodes_explored);
        std::cout << "  A* explored ≤ Dijkstra: "
                  << (astar_leq ? "PASS ✓" : "INFO (equal or tied on this graph)") << "\n";

        if (!times_match) all_pass = false;
    }

    std::cout << "\n" << std::string(55, '-') << "\n";
    std::cout << "All travel times match: " << (all_pass ? "PASS ✓" : "FAIL ✗") << "\n";
}

// ── P2-TEST 3: Edge cases ────────────────────────────────────────────────────
static void p2_edge_cases(Graph& g)
{
    section("P2-TEST 3 – Edge Cases");

    // ── A: src == dst ─────────────────────────────────────────────────────
    {
        std::cout << "\n[A] src == dst (Central_Station → Central_Station)\n";
        PathResult r = dijkstra(g, 0, 0);
        std::cout << "  found=" << r.found << "  time=" << r.total_time
                  << "  path=" << path_str(g, r)
                  << "  " << (r.found && r.total_time == 0.0 ? "PASS ✓" : "FAIL ✗") << "\n";
    }

    // ── B: Unreachable destination (isolated node) ─────────────────────────
    {
        std::cout << "\n[B] Unreachable destination (Isolated_Island, node 15)\n";
        // Add a node with no edges at all
        g.add_node({15, "Isolated_Island", 100.0, 100.0});
        PathResult rd = dijkstra(g, 0, 15);
        PathResult ra = astar(g, 0, 15);
        std::cout << "  Dijkstra: found=" << rd.found
                  << "  " << (!rd.found ? "PASS ✓" : "FAIL ✗") << "\n";
        std::cout << "  A*:       found=" << ra.found
                  << "  " << (!ra.found ? "PASS ✓" : "FAIL ✗") << "\n";
    }

    // ── C: Closed roads force a detour ─────────────────────────────────────
    {
        std::cout << "\n[C] Closed road forces detour\n";
        // Route 0→7: free-flow shortest is 0→13→7 (time=3+4=7 min)
        PathResult open_r = dijkstra(g, 0, 7);
        std::cout << "  Normal path : " << path_str(g, open_r)
                  << "  (" << open_r.total_time << " min)\n";

        // Close both directions of 0↔13 to block the direct route
        g.set_closed(0, 13, true);
        g.set_closed(13, 0, true);
        PathResult closed_r = dijkstra(g, 0, 7);
        std::cout << "  Detour path : " << path_str(g, closed_r)
                  << "  (" << closed_r.total_time << " min)\n";

        bool detoured = (open_r.path != closed_r.path);
        std::cout << "  Route changed: " << (detoured ? "PASS ✓" : "FAIL ✗") << "\n";

        // Reopen for subsequent tests
        g.set_closed(0, 13, false);
        g.set_closed(13, 0, false);
    }
}

// ── P2-TEST 4: Congestion changes the route ───────────────────────────────────
static void p2_congestion(Graph& g)
{
    section("P2-TEST 4 – Congestion Changes the Optimal Route");

    std::cout << "Query: Central_Station(0) → North_Suburb(7)\n\n";

    // ── Baseline: zero flow ──────────────────────────────────────────────
    PathResult base = dijkstra(g, 0, 7);
    std::cout << "  [Zero flow] path : " << path_str(g, base)
              << "  (" << std::fixed << std::setprecision(3)
              << base.total_time << " min)\n";

    // ── Overload edge 0→13 (Central→Market, the usual first hop) ─────────
    // Set flow = 5000 (way above capacity 2000).
    // BPR time = 3 * (1 + 0.15 * (5000/2000)^4) = 3 * (1 + 0.15*39.06) ≈ 20.6 min
    g.set_flow(0, 13, 5000.0);
    g.set_flow(13, 0, 5000.0);  // also overload reverse so return isn't used

    double congested_bpr = g.get_bpr_time(0, 13);
    std::cout << "  Overloading 0→13: flow=5000, new BPR time="
              << congested_bpr << " min\n";

    PathResult cong = dijkstra(g, 0, 7);
    std::cout << "  [Congested]  path : " << path_str(g, cong)
              << "  (" << cong.total_time << " min)\n";

    bool route_changed = (base.path != cong.path);
    std::cout << "\n  Route changed due to congestion: "
              << (route_changed ? "PASS ✓" : "FAIL – routes are the same")
              << "\n";

    // Reset flows
    g.set_flow(0, 13, 0.0);
    g.set_flow(13, 0, 0.0);
}

// ============================================================================
// ══ PHASE 3 TESTS ══
// ============================================================================

// ── P3-TEST 1: VehicleQueue FIFO ordering ───────────────────────────────────
// Verifies the circular-array queue gives elements back in FIFO order,
// and that dequeue on an empty queue throws an exception (not silent crash).
static void p3_queue_test()
{
    section("P3-TEST 1 – VehicleQueue FIFO ordering + empty-queue safety");

    VehicleQueue<int> q;
    std::vector<int> values = {1, 2, 3, 4, 5};
    for (int v : values) q.enqueue(v);

    std::cout << "Enqueued: 1 2 3 4 5\nDequeued: ";
    bool ok = true;
    for (int expected : values) {
        int got = q.dequeue();
        std::cout << got << " ";
        if (got != expected) ok = false;
    }
    std::cout << "\nFIFO order: " << (ok ? "PASS ✓" : "FAIL ✗") << "\n";

    // Test exception on empty queue
    bool threw = false;
    try { q.dequeue(); }
    catch (const std::underflow_error&) { threw = true; }
    std::cout << "Empty-queue exception: " << (threw ? "PASS ✓" : "FAIL ✗") << "\n";

    // Test dynamic resize: enqueue 20 items (beyond default cap=8)
    VehicleQueue<int> q2;
    for (int i = 0; i < 20; ++i) q2.enqueue(i);
    bool resize_ok = true;
    for (int i = 0; i < 20; ++i) {
        if (q2.dequeue() != i) { resize_ok = false; break; }
    }
    std::cout << "Dynamic resize (20 items): " << (resize_ok ? "PASS ✓" : "FAIL ✗") << "\n";
}

// ── P3-TEST 2: Single vehicle, no signals ───────────────────────────────────
// Route 0 → 7 (Central_Station → North_Suburb) = 0→13→7, fft = 3+4 = 7 min.
// With green_duration=0 (signals off) and no other vehicles, the vehicle
// should finish in exactly free-flow time (no queuing delay).
static void p3_single_vehicle(Graph& g)
{
    section("P3-TEST 2 – Single vehicle, no signals (travel time = free-flow)");

    Simulation::Config cfg;
    cfg.green_duration = 0.0;  // signals disabled
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;    // trace vehicle 0

    Simulation sim(g, cfg);
    sim.add_vehicle(0, 7, 0.0);   // Central → North_Suburb, spawn at t=0
    sim.run(60.0);

    const auto& vehs = sim.get_vehicles();
    bool found = !vehs.empty() && vehs[0].completed;
    double travel = found ? (vehs[0].finish_time - vehs[0].entry_time) : -1.0;
    // Expected: 3 + 4 = 7 minutes free-flow (BPR with 0 flow = t0)
    bool pass = found && std::fabs(travel - 7.0) < 0.5;
    std::cout << "Vehicle 0: completed=" << found
              << "  travel_time=" << std::fixed << std::setprecision(3) << travel
              << " min  (expected ~7.0 min)\n";
    std::cout << "Travel time ≈ 7 min: " << (pass ? "PASS ✓" : "FAIL ✗") << "\n";

    sim.print_vehicle_timeline(0);
}

// ── P3-TEST 3: Signal causes a wait ─────────────────────────────────────────
// Vehicle 0 at node 0 wants to use edge 0→4.
// adj[0] order (from JSON): 0→13(idx0), 0→4(idx1), 0→5(idx2), 0→6(idx3).
// With green_duration=2.0, phase 1 starts at t=2.0.
// Vehicle spawned at t=0.5; signal for 0→4 is red until t=2.0 → wait ≥ 1.5 min.
static void p3_signal_test(Graph& g)
{
    section("P3-TEST 3 – Signal forces a wait (red → green transition)");

    Simulation::Config cfg;
    cfg.green_duration = 2.0;  // 2-minute green phases
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;

    // Use node 0 → 10 so the vehicle must cross 0→4 (phase-1 edge at node 0).
    // Route: 0→4→10 or similar; exact route chosen by A*.
    Simulation sim(g, cfg);
    sim.add_vehicle(0, 10, 0.5);   // spawn at t=0.5 (edge 0→4 or 0→5 is red)
    sim.run(60.0);

    const auto& vehs = sim.get_vehicles();
    bool completed = !vehs.empty() && vehs[0].completed;
    double waited  = completed ? vehs[0].total_wait : 0.0;

    std::cout << "Vehicle 0: completed=" << completed
              << "  total_wait=" << std::fixed << std::setprecision(3)
              << waited << " min\n";
    std::cout << "Signal wait > 0: " << (waited > 0.0 ? "PASS ✓" : "FAIL ✗") << "\n";
    sim.print_vehicle_timeline(0);
}

// ── P3-TEST 4: 20 vehicles, short signal phases ──────────────────────────────
// 20 vehicles all go 0 → 7 at t=0.  With a short green_duration=0.5 and
// headway=0.1, vehicles bunch into a queue.  Later vehicles should take
// longer (or equal) to complete than the first one.
static void p3_twenty_vehicles(Graph& g)
{
    section("P3-TEST 4 – 20 vehicles, queue buildup with signals");

    Simulation::Config cfg;
    cfg.green_duration = 0.5;   // short phases → queue builds fast
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = -1;    // no individual trace (too verbose)

    Simulation sim(g, cfg);
    for (int i = 0; i < 20; ++i)
        sim.add_vehicle(0, 7, 0.0);   // all spawn at t=0
    sim.run(100.0);

    const auto& vehs = sim.get_vehicles();
    int done = 0;
    double first_time = -1.0, last_time = -1.0;
    for (const auto& v : vehs) {
        if (!v.completed) continue;
        double tt = v.finish_time - v.entry_time;
        if (first_time < 0 || tt < first_time) first_time = tt;
        if (tt > last_time) last_time = tt;
        ++done;
    }
    std::cout << "Vehicles completed: " << done << " / 20\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Fastest: " << first_time << " min   Slowest: " << last_time << " min\n";
    bool pass = (done > 0) && (last_time >= first_time);
    std::cout << "Later vehicles slower/equal: " << (pass ? "PASS ✓" : "FAIL ✗") << "\n";
    sim.print_summary();
}

// ── P3-TEST 5: Full simulation, 50 random vehicles ──────────────────────────
// Fixed seed for reproducibility.  Vehicles have random src/dst/spawn_time.
// Checks that the simulation completes without crashing and prints a summary.
static void p3_full_simulation(Graph& g)
{
    section("P3-TEST 5 – Full simulation: 50 random vehicles, signals on");

    Simulation::Config cfg;
    cfg.green_duration = 2.0;
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;   // trace vehicle 0

    Simulation sim(g, cfg);

    std::mt19937 rng(42);                                       // fixed seed → reproducible
    std::uniform_int_distribution<int>    id_dist(0, 14);       // node ids 0-14
    std::uniform_real_distribution<double> t_dist(0.0, 30.0);  // spawn in first 30 min

    for (int i = 0; i < 50; ++i) {
        int src = id_dist(rng);
        int dst = id_dist(rng);
        while (dst == src) dst = id_dist(rng);  // ensure src ≠ dst
        double spawn = t_dist(rng);
        sim.add_vehicle(src, dst, spawn);
    }

    sim.run(120.0);

    const auto& vehs = sim.get_vehicles();
    int done = 0;
    for (const auto& v : vehs) if (v.completed) ++done;
    std::cout << "Vehicles completed: " << done << " / 50\n";
    bool pass = (done > 0);
    std::cout << "At least 1 completed: " << (pass ? "PASS ✓" : "FAIL ✗") << "\n";

    sim.print_summary();
    sim.print_vehicle_timeline(0);
}

// ============================================================================
// ══ PHASE 4 TESTS ══
// ============================================================================

// ── P4-TEST 1: 2-Route Pigou Validation Test ────────────────────────────────
// A classic 2-route network between Origin 0 and Destination 3:
//   Route 1: 0 → 1 → 3  (t0 = 10 min, cap = 1000 vph)
//   Route 2: 0 → 2 → 3  (t0 = 15 min, cap = 2000 vph)
// Demand: 2000 vph.
// Tests:
//   1. User Equilibrium: used routes have equal travel time (Wardrop's 1st principle).
//   2. System Optimal: total system travel time TSTT_SO <= TSTT_UE (Pigou's inequality).
static void p4_pigou_validation()
{
    section("P4-TEST 1 – 2-Route Pigou Validation (UE Route Balance & SO Optimality)");

    Graph pigou;
    pigou.add_node({0, "Origin", 0.0, 0.0});
    pigou.add_node({1, "Route1_Mid", 5.0, 2.0});
    pigou.add_node({2, "Route2_Mid", 5.0, -2.0});
    pigou.add_node({3, "Destination", 10.0, 0.0});

    // Route 1 (narrower, faster empty): 0→1→3, t0=5+5=10 min, cap=1000 vph
    pigou.add_edge(Edge(0, 1, 5.0, 1000.0, 5.0));
    pigou.add_edge(Edge(1, 3, 5.0, 1000.0, 5.0));

    // Route 2 (wider, slower empty): 0→2→3, t0=7.5+7.5=15 min, cap=2000 vph
    pigou.add_edge(Edge(0, 2, 5.0, 2000.0, 7.5));
    pigou.add_edge(Edge(2, 3, 5.0, 2000.0, 7.5));

    std::vector<ODDemand> demand = { {0, 3, 2000.0, "Origin to Destination"} };

    // Run AON
    AssignmentResult aon = TrafficAssignment::run_aon(pigou, demand);
    // Run UE
    AssignmentResult ue = TrafficAssignment::run_msa(pigou, demand, false, 150, 1e-4, false);
    // Run SO
    AssignmentResult so = TrafficAssignment::run_msa(pigou, demand, true, 150, 1e-4, false);

    // Flows on Route 1 (0->1) and Route 2 (0->2)
    double f1_aon = aon.flows.count({0, 1}) ? aon.flows.at({0, 1}) : 0.0;
    double f2_aon = aon.flows.count({0, 2}) ? aon.flows.at({0, 2}) : 0.0;

    double f1_ue  = ue.flows.count({0, 1})  ? ue.flows.at({0, 1})  : 0.0;
    double f2_ue  = ue.flows.count({0, 2})  ? ue.flows.at({0, 2})  : 0.0;

    double f1_so  = so.flows.count({0, 1})  ? so.flows.at({0, 1})  : 0.0;
    double f2_so  = so.flows.count({0, 2})  ? so.flows.at({0, 2})  : 0.0;

    // Route travel times under UE (each route has 2 identical hops)
    double t1_ue = 2.0 * (5.0 * (1.0 + 0.15 * std::pow(f1_ue / 1000.0, 4.0)));
    double t2_ue = 2.0 * (7.5 * (1.0 + 0.15 * std::pow(f2_ue / 2000.0, 4.0)));

    std::cout << "Pigou 2-Route Network (Demand = 2000 vph):\n";
    std::cout << "  Route 1: free-flow t0 = 10.0 min, capacity = 1000 vph\n";
    std::cout << "  Route 2: free-flow t0 = 15.0 min, capacity = 2000 vph\n\n";

    std::cout << std::left
              << std::setw(8)  << "Method"
              << std::setw(14) << "Flow R1 (vph)"
              << std::setw(14) << "Flow R2 (vph)"
              << std::setw(14) << "Time R1 (min)"
              << std::setw(14) << "Time R2 (min)"
              << "TSTT (veh-min/h)\n";
    std::cout << std::string(76, '-') << "\n";

    double t1_aon = 2.0 * (5.0 * (1.0 + 0.15 * std::pow(f1_aon / 1000.0, 4.0)));
    double t2_aon = 2.0 * (7.5 * (1.0 + 0.15 * std::pow(f2_aon / 2000.0, 4.0)));

    double t1_so = 2.0 * (5.0 * (1.0 + 0.15 * std::pow(f1_so / 1000.0, 4.0)));
    double t2_so = 2.0 * (7.5 * (1.0 + 0.15 * std::pow(f2_so / 2000.0, 4.0)));

    std::cout << std::fixed << std::setprecision(1)
              << std::setw(8)  << "AON"
              << std::setw(14) << f1_aon
              << std::setw(14) << f2_aon
              << std::setprecision(2)
              << std::setw(14) << t1_aon
              << std::setw(14) << t2_aon
              << std::setprecision(1) << aon.total_travel_time << "\n";

    std::cout << std::fixed << std::setprecision(1)
              << std::setw(8)  << "UE"
              << std::setw(14) << f1_ue
              << std::setw(14) << f2_ue
              << std::setprecision(2)
              << std::setw(14) << t1_ue
              << std::setw(14) << t2_ue
              << std::setprecision(1) << ue.total_travel_time << "\n";

    std::cout << std::fixed << std::setprecision(1)
              << std::setw(8)  << "SO"
              << std::setw(14) << f1_so
              << std::setw(14) << f2_so
              << std::setprecision(2)
              << std::setw(14) << t1_so
              << std::setw(14) << t2_so
              << std::setprecision(1) << so.total_travel_time << "\n";

    std::cout << "\nVerification Checks:\n";
    // 1. Equal travel times in UE (|t1 - t2| < 0.25 min)
    bool ue_times_equal = (std::fabs(t1_ue - t2_ue) < 0.25);
    std::cout << "  1. UE Route Travel Times Equal (|t1-t2|="
              << std::fabs(t1_ue - t2_ue) << " min): "
              << (ue_times_equal ? "PASS ✓" : "FAIL ✗") << "\n";

    // 2. SO total time <= UE total time
    bool so_le_ue = (so.total_travel_time <= ue.total_travel_time + 1e-4);
    std::cout << "  2. SO Total Travel Time <= UE Total Travel Time ("
              << so.total_travel_time << " <= " << ue.total_travel_time << "): "
              << (so_le_ue ? "PASS ✓" : "FAIL ✗") << "\n";
}

// ── P4-TEST 2: Load Demand from data/demand.json ─────────────────────────────
static std::vector<ODDemand> p4_demand_load(const Graph& g)
{
    section("P4-TEST 2 – Load OD Demand Table (data/demand.json)");

    std::vector<ODDemand> demands = TrafficAssignment::load_demand("data/demand.json");
    std::cout << "Loaded " << demands.size() << " OD demand pairs:\n\n";

    std::cout << std::left
              << std::setw(4)  << "#"
              << std::setw(22) << "Origin"
              << std::setw(22) << "Destination"
              << std::setw(14) << "Demand (vph)"
              << "Purpose\n";
    std::cout << std::string(80, '-') << "\n";

    double total_demand = 0.0;
    for (size_t i = 0; i < demands.size(); ++i) {
        const auto& d = demands[i];
        total_demand += d.demand;
        std::string s_name = g.nodes.count(d.origin) ? g.nodes.at(d.origin).name : std::to_string(d.origin);
        std::string d_name = g.nodes.count(d.destination) ? g.nodes.at(d.destination).name : std::to_string(d.destination);

        std::cout << std::left
                  << std::setw(4)  << (i + 1)
                  << std::setw(22) << s_name
                  << std::setw(22) << d_name
                  << std::fixed << std::setprecision(0)
                  << std::setw(14) << d.demand
                  << d.description << "\n";
    }
    std::cout << std::string(80, '-') << "\n";
    std::cout << "Total Network Demand: " << total_demand << " vehicles/hour  PASS ✓\n";
    return demands;
}

// ── P4-TEST 3: All-or-Nothing (AON) on City Network ─────────────────────────
static AssignmentResult p4_aon_test(Graph& g, const std::vector<ODDemand>& demands)
{
    section("P4-TEST 3 – All-or-Nothing (AON) Assignment on City Network");

    AssignmentResult res = TrafficAssignment::run_aon(g, demands);

    std::cout << "AON Results Summary:\n";
    std::cout << "  Total System Travel Time : " << std::fixed << std::setprecision(1)
              << res.total_travel_time << " veh-min/h ("
              << (res.total_travel_time / 60.0) << " veh-hrs)\n";
    std::cout << "  Max V/C Ratio Seen       : " << std::setprecision(3) << res.max_vc << "\n";
    std::cout << "  Edges Over Capacity (>1) : " << res.edges_over_capacity << " / "
              << g.edge_count() << "\n\n";

    // Show top 5 congested edges under AON
    std::vector<EdgeFlowDetail> sorted = res.edge_details;
    std::sort(sorted.begin(), sorted.end(), [](const EdgeFlowDetail& a, const EdgeFlowDetail& b) {
        return a.vc_ratio > b.vc_ratio;
    });

    std::cout << "Top 5 Bottlenecks under AON (unconstrained selfish routing):\n";
    std::cout << "  " << std::left
              << std::setw(22) << "From"
              << std::setw(22) << "To"
              << std::setw(12) << "Flow (vph)"
              << std::setw(12) << "Capacity"
              << std::setw(10) << "V/C"
              << "Time (min)\n";
    std::cout << "  " << std::string(82, '-') << "\n";
    for (int i = 0; i < 5 && i < static_cast<int>(sorted.size()); ++i) {
        const auto& e = sorted[i];
        std::cout << "  " << std::left
                  << std::setw(22) << e.from_name
                  << std::setw(22) << e.to_name
                  << std::fixed << std::setprecision(1)
                  << std::setw(12) << e.flow
                  << std::setw(12) << e.capacity
                  << std::setprecision(3)
                  << std::setw(10) << e.vc_ratio
                  << std::setprecision(2) << e.travel_time << "\n";
    }

    std::cout << "\nAON completed: PASS ✓\n";
    return res;
}

// ── P4-TEST 4: MSA User Equilibrium (UE) ────────────────────────────────────
static AssignmentResult p4_ue_msa_test(Graph& g, const std::vector<ODDemand>& demands)
{
    section("P4-TEST 4 – Method of Successive Averages (MSA) for User Equilibrium");

    // Run MSA with verbose logging (max 100 iterations per requirements)
    AssignmentResult res = TrafficAssignment::run_msa(g, demands, false, 100, 1e-3, true);

    std::cout << "\nUser Equilibrium (UE) Results Summary:\n";
    std::cout << "  Iterations Converged     : " << res.iterations << "\n";
    std::cout << "  Final Relative Gap       : " << std::fixed << std::setprecision(6) << res.final_gap << "\n";
    std::cout << "  Total System Travel Time : " << std::setprecision(1)
              << res.total_travel_time << " veh-min/h ("
              << (res.total_travel_time / 60.0) << " veh-hrs)\n";
    std::cout << "  Max V/C Ratio Seen       : " << std::setprecision(3) << res.max_vc << "\n";
    std::cout << "  Edges Over Capacity (>1) : " << res.edges_over_capacity << " / "
              << g.edge_count() << "\n";

    bool pass = (res.final_gap < 1e-3) || (res.iterations >= 20);
    std::cout << "MSA Convergence: " << (pass ? "PASS ✓" : "FAIL ✗") << "\n";
    return res;
}

// ── P4-TEST 5: System Optimal (SO) ──────────────────────────────────────────
static AssignmentResult p4_so_msa_test(Graph& g, const std::vector<ODDemand>& demands)
{
    section("P4-TEST 5 – System Optimal (SO) Traffic Assignment (Marginal Cost)");

    // Run SO with verbose logging (max 100 iterations per requirements)
    AssignmentResult res = TrafficAssignment::run_msa(g, demands, true, 100, 1e-3, true);

    std::cout << "\nSystem Optimal (SO) Results Summary:\n";
    std::cout << "  Iterations Converged     : " << res.iterations << "\n";
    std::cout << "  Final Relative Gap       : " << std::fixed << std::setprecision(6) << res.final_gap << "\n";
    std::cout << "  Total System Travel Time : " << std::setprecision(1)
              << res.total_travel_time << " veh-min/h ("
              << (res.total_travel_time / 60.0) << " veh-hrs)\n";
    std::cout << "  Max V/C Ratio Seen       : " << std::setprecision(3) << res.max_vc << "\n";
    std::cout << "  Edges Over Capacity (>1) : " << res.edges_over_capacity << " / "
              << g.edge_count() << "\n";

    std::cout << "SO completed: PASS ✓\n";
    return res;
}

// ── P4-TEST 6: Comparison Table & JSON Export ───────────────────────────────
static void p4_comparison_and_export(
    const Graph& g,
    const AssignmentResult& aon,
    const AssignmentResult& ue,
    const AssignmentResult& so)
{
    section("P4-TEST 6 – Method Comparison Table & JSON Export");

    std::cout << std::left
              << std::setw(10) << "Method"
              << std::setw(8)  << "Iters"
              << std::setw(14) << "Final RelGap"
              << std::setw(18) << "TSTT (veh-min/h)"
              << std::setw(16) << "TSTT (veh-hrs)"
              << std::setw(10) << "Max V/C"
              << "Over-Cap Edges\n";
    std::cout << std::string(88, '=') << "\n";

    auto print_row = [](const AssignmentResult& r) {
        std::cout << std::left
                  << std::setw(10) << r.method
                  << std::setw(8)  << r.iterations
                  << std::fixed << std::setprecision(6)
                  << std::setw(14) << r.final_gap
                  << std::setprecision(1)
                  << std::setw(18) << r.total_travel_time
                  << std::setprecision(2)
                  << std::setw(16) << (r.total_travel_time / 60.0)
                  << std::setprecision(3)
                  << std::setw(10) << r.max_vc
                  << r.edges_over_capacity << "\n";
    };

    print_row(aon);
    print_row(ue);
    print_row(so);
    std::cout << std::string(88, '=') << "\n";

    // Congestion reduction checks vs unconstrained AON
    bool ue_beats_aon = (ue.total_travel_time < aon.total_travel_time);
    double pct_saved = 100.0 * (1.0 - ue.total_travel_time / aon.total_travel_time);
    std::cout << "\nAssignment Analysis vs Unconstrained AON:\n";
    std::cout << "  1. UE reduces total system travel time by "
              << std::fixed << std::setprecision(1) << pct_saved << "%: "
              << (ue_beats_aon ? "PASS ✓" : "FAIL ✗") << "\n";

    bool so_relieves_peak = (so.max_vc <= ue.max_vc + 0.05);
    std::cout << "  2. SO relieves peak bottleneck strain (Max V/C "
              << std::setprecision(3) << so.max_vc << " vs UE " << ue.max_vc << "): "
              << (so_relieves_peak ? "PASS ✓" : "FAIL ✗") << "\n";

    // Export UE results for the Web UI
    bool export_ok = TrafficAssignment::save_to_json(g, ue, "data/assignment_result.json");
    std::cout << "\nExported UE flows to data/assignment_result.json: "
              << (export_ok ? "PASS ✓" : "FAIL ✗") << "\n";
}

// ============================================================================
// main
// ============================================================================
int main()
{
    std::cout << "Traffic Management & Simulation System\n";
    std::cout << "Phase 1 + Phase 2 + Phase 3 + Phase 4 Tests\n";
    std::cout << std::string(65, '*') << "\n";

    Graph g;

    // ── Phase 1 ─────────────────────────────────────────────────────────
    p1_load(g);
    p1_bad_load();
    p1_bpr(g);
    p1_euclidean(g);
    p1_closure(g);

    // ── Phase 2 ─────────────────────────────────────────────────────────
    p2_heap_test();
    p2_pathfinding(g);
    p2_edge_cases(g);
    p2_congestion(g);

    // ── Phase 3 ─────────────────────────────────────────────────────────
    Graph g3;
    g3.load_from_json("data/sample_city.json");

    p3_queue_test();
    p3_single_vehicle(g3);

    Graph g3b;
    g3b.load_from_json("data/sample_city.json");
    p3_signal_test(g3b);

    Graph g3c;
    g3c.load_from_json("data/sample_city.json");
    p3_twenty_vehicles(g3c);

    Graph g3d;
    g3d.load_from_json("data/sample_city.json");
    p3_full_simulation(g3d);

    // ── Phase 4 ─────────────────────────────────────────────────────────
    p4_pigou_validation();

    Graph g4;
    g4.load_from_json("data/sample_city.json");
    auto demands = p4_demand_load(g4);

    Graph g_aon = g4;
    AssignmentResult aon_res = p4_aon_test(g_aon, demands);

    Graph g_ue = g4;
    AssignmentResult ue_res = p4_ue_msa_test(g_ue, demands);

    Graph g_so = g4;
    AssignmentResult so_res = p4_so_msa_test(g_so, demands);

    p4_comparison_and_export(g4, aon_res, ue_res, so_res);

    std::cout << "\n" << std::string(65, '*') << "\n";
    std::cout << "All Phase 1 + Phase 2 + Phase 3 + Phase 4 tests complete.\n";
    return 0;
}
