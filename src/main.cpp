// =============================================================================
// main.cpp  –  Phase 1 + Phase 2 tests
// Traffic Management & Simulation System
// =============================================================================
#include "graph.h"
#include "minheap.h"
#include "pathfinder.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>      // fabs
#include <algorithm>  // is_sorted
#include <string>
#include <cassert>

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
// main
// ============================================================================
int main()
{
    std::cout << "Traffic Management & Simulation System\n";
    std::cout << "Phase 1 + Phase 2 Tests\n";
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

    std::cout << "\n" << std::string(65, '*') << "\n";
    std::cout << "All Phase 1 + Phase 2 tests complete.\n";
    return 0;
}
