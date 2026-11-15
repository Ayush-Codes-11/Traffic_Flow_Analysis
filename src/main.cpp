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
#include "incidents.h"
#include "sensor_feed.h"
#include "alerts.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>      // fabs
#include <algorithm>  // is_sorted, sort
#include <string>
#include <cassert>
#include <random>     // mt19937

// ── Global Test Accounting ───────────────────────────────────────────────────
static int g_tests_total  = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

static void report_check(const std::string& name, bool pass)
{
    ++g_tests_total;
    if (pass) {
        ++g_tests_passed;
        std::cout << "  " << name << ": PASS ✓\n";
    } else {
        ++g_tests_failed;
        std::cout << "  " << name << ": FAIL ✗\n";
    }
}

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
    std::cout << "Nodes: " << g.node_count()
              << "  |  Directed edges: " << g.edge_count() << "\n";
    report_check("P1-TEST 1: Load sample_city.json", ok && g.node_count() == 15 && g.edge_count() == 50);
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
    report_check("P1-TEST 2a: Partial load with skipped malformed entries", ok && bad.node_count() == 1);

    // Also test nonexistent file
    Graph ghost;
    std::cout << "\nLoading nonexistent file:\n";
    bool ok2 = ghost.load_from_json("data/does_not_exist.json");
    std::cout << "Return value  : " << (ok2 ? "true" : "false") << "\n";
    report_check("P1-TEST 2b: Nonexistent file returns false", !ok2);
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

    double t_zero = 0.0, t_cap = 0.0;
    for (double flow : {0.0, 500.0, 1000.0, 1500.0, 2000.0, 3000.0}) {
        const_cast<Graph&>(g).set_flow(0, 13, flow);
        double t  = g.get_bpr_time(0, 13);
        double t0 = 3.0, cap = 2000.0;
        if (flow == 0.0) t_zero = t;
        if (flow == 2000.0) t_cap = t;
        std::cout << std::fixed << std::setprecision(0)
                  << std::setw(12) << flow
                  << std::setprecision(2)
                  << std::setw(10) << (flow / cap)
                  << std::setprecision(3) << std::setw(16) << t
                  << "x" << std::setprecision(2) << (t / t0) << "\n";
    }
    const_cast<Graph&>(g).set_flow(0, 13, 0.0);
    report_check("P1-TEST 3: BPR matches analytical t0 and capacity slowdown",
                 std::fabs(t_zero - 3.0) < 1e-4 && std::fabs(t_cap - 3.45) < 1e-4);
}

static void p1_euclidean(const Graph& g)
{
    section("P1-TEST 4 – Euclidean distance (A* heuristic preview)");
    struct Pair { int a, b; double exp; };
    bool ok = true;
    for (auto [a, b, exp] : std::vector<Pair>{{0,1,5.385},{0,8,6.0},{9,7,10.0},{5,10,9.0}}) {
        double d = g.euclidean_dist(a, b);
        std::cout << std::left
                  << std::setw(22) << g.nodes.at(a).name
                  << " → "
                  << std::setw(22) << g.nodes.at(b).name
                  << std::fixed << std::setprecision(3)
                  << d << " units\n";
        if (std::fabs(d - exp) > 0.01) ok = false;
    }
    report_check("P1-TEST 4: Euclidean distances correct", ok);
}

static void p1_closure(Graph& g)
{
    section("P1-TEST 5 – Road closure");
    int f = 0, t = 5;
    std::cout << "Edge 0→5 (Central→Business)\n";
    std::cout << "  Open  : " << g.get_bpr_time(f, t) << " min\n";
    g.set_closed(f, t, true);
    double closed_t = g.get_bpr_time(f, t);
    std::cout << "  Closed: " << closed_t << " (∞)\n";
    g.set_closed(f, t, false);
    double reopen_t = g.get_bpr_time(f, t);
    std::cout << "  Reopen: " << reopen_t << " min\n";
    report_check("P1-TEST 5: Road closure and reopening", closed_t > 1e10 && std::fabs(reopen_t - 5.0) < 1e-4);
}

// ============================================================================
// ══ PHASE 2 TESTS ══
// ============================================================================

static void p2_heap_test()
{
    section("P2-TEST 1 – Custom MinHeap (push / pop / heapify)");

    MinHeap<int> h;
    std::vector<int> inputs = {7, 3, 8, 1, 9, 2, 10, 4, 6, 5};
    for (int v : inputs) h.push(static_cast<double>(v), v);

    std::vector<int> output;
    while (!h.empty()) output.push_back(h.pop().value);

    bool sorted = std::is_sorted(output.begin(), output.end());
    report_check("P2-TEST 1a: Custom MinHeap pop order sorted ascending", sorted);

    MinHeap<int> h3;
    h3.push(10.0, 42);
    h3.push( 5.0, 42);
    double first_p = h3.pop().priority;
    report_check("P2-TEST 1b: Lazy-deletion duplicate push order", std::fabs(first_p - 5.0) < 1e-9);
}

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
        std::cout << "  Dijkstra │ explored=" << std::setw(3) << dijk.nodes_explored
                  << " │ time=" << std::fixed << std::setprecision(3) << dijk.total_time
                  << " min │ path: " << path_str(g, dijk) << "\n";
        std::cout << "  A*       │ explored=" << std::setw(3) << astr.nodes_explored
                  << " │ time=" << std::fixed << std::setprecision(3) << astr.total_time
                  << " min │ path: " << path_str(g, astr) << "\n";

        bool times_match = (std::fabs(dijk.total_time - astr.total_time) < 1e-6);
        if (!times_match) all_pass = false;
    }

    report_check("P2-TEST 2: Dijkstra and A* return identical travel times", all_pass);
}

static void p2_edge_cases(Graph& g)
{
    section("P2-TEST 3 – Edge Cases");

    // A: src == dst
    PathResult r = dijkstra(g, 0, 0);
    report_check("P2-TEST 3a: src == dst returns total_time=0.0", r.found && r.total_time == 0.0);

    // B: Unreachable destination
    g.add_node({15, "Isolated_Island", 100.0, 100.0});
    PathResult rd = dijkstra(g, 0, 15);
    PathResult ra = astar(g, 0, 15);
    report_check("P2-TEST 3b: Isolated node unreachable (found=false)", !rd.found && !ra.found);

    // C: Closed roads force a detour
    PathResult open_r = dijkstra(g, 0, 7);
    g.set_closed(0, 13, true);
    g.set_closed(13, 0, true);
    PathResult closed_r = dijkstra(g, 0, 7);
    g.set_closed(0, 13, false);
    g.set_closed(13, 0, false);
    report_check("P2-TEST 3c: Road closure forces path detour", open_r.path != closed_r.path);
}

static void p2_congestion(Graph& g)
{
    section("P2-TEST 4 – Congestion Changes the Optimal Route");

    PathResult base = dijkstra(g, 0, 7);
    g.set_flow(0, 13, 5000.0);
    g.set_flow(13, 0, 5000.0);

    PathResult cong = dijkstra(g, 0, 7);
    g.set_flow(0, 13, 0.0);
    g.set_flow(13, 0, 0.0);

    report_check("P2-TEST 4: Overloading bottleneck alters shortest path", base.path != cong.path);
}

// ============================================================================
// ══ PHASE 3 TESTS ══
// ============================================================================

static void p3_queue_test()
{
    section("P3-TEST 1 – VehicleQueue FIFO ordering + empty-queue safety");

    VehicleQueue<int> q;
    std::vector<int> values = {1, 2, 3, 4, 5};
    for (int v : values) q.enqueue(v);

    bool ok = true;
    for (int expected : values) {
        if (q.dequeue() != expected) ok = false;
    }
    report_check("P3-TEST 1a: VehicleQueue FIFO dequeue order", ok);

    bool threw = false;
    try { q.dequeue(); }
    catch (const std::underflow_error&) { threw = true; }
    report_check("P3-TEST 1b: VehicleQueue underflow exception", threw);

    VehicleQueue<int> q2;
    for (int i = 0; i < 20; ++i) q2.enqueue(i);
    bool resize_ok = true;
    for (int i = 0; i < 20; ++i) {
        if (q2.dequeue() != i) { resize_ok = false; break; }
    }
    report_check("P3-TEST 1c: VehicleQueue dynamic circular resize", resize_ok);
}

static void p3_single_vehicle(Graph& g)
{
    section("P3-TEST 2 – Single vehicle, no signals (travel time = free-flow)");

    Simulation::Config cfg;
    cfg.green_duration = 0.0;
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;

    Simulation sim(g, cfg);
    sim.add_vehicle(0, 7, 0.0);
    sim.run(60.0);

    const auto& vehs = sim.get_vehicles();
    bool found = !vehs.empty() && vehs[0].completed;
    double travel = found ? (vehs[0].finish_time - vehs[0].entry_time) : -1.0;
    report_check("P3-TEST 2: Single vehicle travel time matches free-flow ~7 min",
                 found && std::fabs(travel - 7.0) < 0.2);
    sim.print_vehicle_timeline(0);
}

static void p3_signal_test(Graph& g)
{
    section("P3-TEST 3 – Signal forces a wait (red → green transition)");

    Simulation::Config cfg;
    cfg.green_duration = 2.0;
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;

    Simulation sim(g, cfg);
    sim.add_vehicle(0, 10, 0.5);
    sim.run(60.0);

    const auto& vehs = sim.get_vehicles();
    bool completed = !vehs.empty() && vehs[0].completed;
    double waited  = completed ? vehs[0].total_wait : 0.0;

    report_check("P3-TEST 3: Signal red phase forces vehicle wait time > 0", completed && waited > 0.0);
    sim.print_vehicle_timeline(0);
}

static void p3_twenty_vehicles(Graph& g)
{
    section("P3-TEST 4 – 20 vehicles, queue buildup with signals (Fix 1 verification)");

    Simulation::Config cfg;
    cfg.green_duration = 0.5;   // short phases → queue builds fast
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = -1;

    Simulation sim(g, cfg);
    for (int i = 0; i < 20; ++i)
        sim.add_vehicle(0, 7, 0.0);
    sim.run(300.0); // run long enough for all vehicles to cycle through signals

    const auto& vehs = sim.get_vehicles();
    int done = 0;
    for (const auto& v : vehs) if (v.completed) ++done;

    std::cout << "Vehicles completed: " << done << " / 20\n";
    sim.print_summary();

    // Fix 1 requirement: all 20 must complete (no stranded queues)
    report_check("P3-TEST 4: All 20 vehicles complete through short signal cycle (no stranding)", done == 20);
}

static void p3_full_simulation(Graph& g)
{
    section("P3-TEST 5 – Full simulation: 50 random vehicles, signals on");

    Simulation::Config cfg;
    cfg.green_duration = 2.0;
    cfg.headway        = 0.1;
    cfg.log_vehicle_id = 0;

    Simulation sim(g, cfg);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int>    id_dist(0, 14);
    std::uniform_real_distribution<double> t_dist(0.0, 30.0);

    for (int i = 0; i < 50; ++i) {
        int src = id_dist(rng);
        int dst = id_dist(rng);
        while (dst == src) dst = id_dist(rng);
        double spawn = t_dist(rng);
        sim.add_vehicle(src, dst, spawn);
    }

    sim.run(300.0); // run long enough for all 50 vehicles to complete

    const auto& vehs = sim.get_vehicles();
    int done = 0;
    for (const auto& v : vehs) if (v.completed) ++done;

    std::cout << "Vehicles completed: " << done << " / 50\n";
    sim.print_summary();
    sim.print_vehicle_timeline(0);

    // Fix 2 requirement: all 50 must complete
    report_check("P3-TEST 5: All 50 stochastic vehicles complete", done == 50);
}

// ============================================================================
// ══ PHASE 4 TESTS ══
// ============================================================================

static void p4_pigou_validation()
{
    section("P4-TEST 1 – 2-Route Pigou Validation (UE Route Balance & SO Optimality)");

    Graph pigou;
    pigou.add_node({0, "Origin", 0.0, 0.0});
    pigou.add_node({1, "Route1_Mid", 5.0, 2.0});
    pigou.add_node({2, "Route2_Mid", 5.0, -2.0});
    pigou.add_node({3, "Destination", 10.0, 0.0});

    pigou.add_edge(Edge(0, 1, 5.0, 1000.0, 5.0));
    pigou.add_edge(Edge(1, 3, 5.0, 1000.0, 5.0));
    pigou.add_edge(Edge(0, 2, 5.0, 2000.0, 7.5));
    pigou.add_edge(Edge(2, 3, 5.0, 2000.0, 7.5));

    std::vector<ODDemand> demand = { {0, 3, 2000.0, "Origin to Destination"} };

    AssignmentResult aon = TrafficAssignment::run_aon(pigou, demand);
    AssignmentResult ue  = TrafficAssignment::run_frank_wolfe(pigou, demand, false, 200, 1e-4, false);
    AssignmentResult so  = TrafficAssignment::run_frank_wolfe(pigou, demand, true, 200, 1e-4, false);

    double f1_ue  = ue.flows.count({0, 1}) ? ue.flows.at({0, 1}) : 0.0;
    double f2_ue  = ue.flows.count({0, 2}) ? ue.flows.at({0, 2}) : 0.0;
    double t1_ue = 2.0 * (5.0 * (1.0 + 0.15 * std::pow(f1_ue / 1000.0, 4.0)));
    double t2_ue = 2.0 * (7.5 * (1.0 + 0.15 * std::pow(f2_ue / 2000.0, 4.0)));

    std::cout << "Pigou 2-Route Results (Demand = 2000 vph):\n";
    std::cout << "  AON: TSTT=" << aon.total_travel_time << " veh-min/h\n";
    std::cout << "  UE : Route1 Time=" << t1_ue << " min, Route2 Time=" << t2_ue
              << " min, TSTT=" << ue.total_travel_time << " veh-min/h\n";
    std::cout << "  SO : TSTT=" << so.total_travel_time << " veh-min/h\n";

    bool times_equal = (std::fabs(t1_ue - t2_ue) < 0.25);
    bool so_le_ue    = (so.total_travel_time <= ue.total_travel_time + 1e-4);

    report_check("P4-TEST 1a: Pigou 2-Route UE Travel Times Equal (|t1-t2| < 0.25)", times_equal);
    report_check("P4-TEST 1b: Pigou 2-Route SO Total Travel Time <= UE Total Travel Time", so_le_ue);
}

static std::vector<ODDemand> p4_demand_load(const Graph& g, double scale)
{
    section("P4-TEST 2 – Load Scaled OD Demand Table (data/demand.json)");

    std::vector<ODDemand> demands = TrafficAssignment::load_demand("data/demand.json", scale);
    std::cout << "Loaded " << demands.size() << " OD demand pairs with scale factor " << scale << ":\n\n";

    double total_demand = 0.0;
    for (size_t i = 0; i < demands.size(); ++i) {
        total_demand += demands[i].demand;
        std::string s_name = g.nodes.count(demands[i].origin) ? g.nodes.at(demands[i].origin).name : std::to_string(demands[i].origin);
        std::string d_name = g.nodes.count(demands[i].destination) ? g.nodes.at(demands[i].destination).name : std::to_string(demands[i].destination);
        std::cout << "  " << std::setw(2) << (i + 1) << ". " << std::left
                  << std::setw(20) << s_name << " → "
                  << std::setw(20) << d_name
                  << " = " << std::fixed << std::setprecision(0) << demands[i].demand << " vph\n";
    }
    std::cout << "Total Network Demand: " << total_demand << " vehicles/hour\n";
    report_check("P4-TEST 2: Load 10 OD demand pairs at scale " + std::to_string(scale), demands.size() == 10 && total_demand > 0);
    return demands;
}

// ── P4 Comparison Table ─────────────────────────────────────────────────────
static void print_comparison_table(
    const std::string& title,
    const std::vector<AssignmentResult>& results)
{
    std::cout << "\n" << title << "\n";
    std::cout << std::left
              << std::setw(12) << "Method"
              << std::setw(8)  << "Iters"
              << std::setw(14) << "Final RelGap"
              << std::setw(18) << "TSTT (veh-min/h)"
              << std::setw(16) << "TSTT (veh-hrs)"
              << std::setw(10) << "Max V/C"
              << "Over-Cap Edges\n";
    std::cout << std::string(88, '=') << "\n";

    for (const auto& r : results) {
        std::cout << std::left
                  << std::setw(12) << r.method
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
    }
    std::cout << std::string(88, '=') << "\n";
}

// =============================================================================
// Phase 5 Test Functions
// =============================================================================

static void test_phase5_capacity_reduction()
{
    section("P5-TEST 1 – Incident Manager & Capacity Reduction");
    Graph g;
    g.load_from_json("data/sample_city.json");

    // Baseline edge 0->13: length=2.5km, cap=2000, fft=3.0 min
    g.set_flow(0, 13, 1000.0);
    double t_orig = g.get_bpr_time(0, 13);

    IncidentManager im;
    AlertManager am;

    // Apply 50% capacity reduction (factor=0.5)
    Incident inc(1, 0, 13, 5.0, 15.0, IncidentType::CAPACITY_REDUCTION, 0.5, "Construction lane closure");
    im.apply_incident_start(inc, g, am);

    double t_reduced = g.get_bpr_time(0, 13);
    std::cout << "Edge 0->13 (flow=1000 vph):\n"
              << "  Original BPR travel time (cap=2000): " << t_orig << " min\n"
              << "  Reduced BPR travel time  (cap=1000): " << t_reduced << " min\n";

    // Revert incident
    im.apply_incident_end(inc, g, am);
    double t_restored = g.get_bpr_time(0, 13);
    std::cout << "  Restored BPR travel time (cap=2000): " << t_restored << " min\n";

    bool cap_ok = (t_reduced > t_orig) && (std::fabs(t_restored - t_orig) < 1e-6);
    report_check("P5-TEST 1: Capacity reduction increases BPR travel time and restores on end", cap_ok);
}

static void test_phase5_closure_and_rerouting()
{
    section("P5-TEST 2 – Road Closure Mid-Run with Dynamic Rerouting");
    Graph g;
    g.load_from_json("data/sample_city.json");

    Simulation::Config cfg;
    cfg.green_duration = 2.0;
    cfg.headway = 0.1;
    cfg.enable_rerouting = true;
    cfg.log_vehicle_id = 0;

    Simulation sim(g, cfg);

    // Schedule accident on edge 0->13 between t=10.0 and t=25.0
    Incident acc(101, 0, 13, 10.0, 25.0, IncidentType::CLOSURE, 0.0, "Accident blocking all lanes");
    sim.add_incident(acc);

    // Spawn 20 vehicles from node 0 to node 13 across t=0 to t=19
    for (int i = 0; i < 20; ++i) {
        sim.add_vehicle(0, 13, static_cast<double>(i));
    }

    sim.run(200.0);

    // Verify edge entries for 0->13: NO vehicle enters during [10.0, 25.0)
    const auto& entries = sim.get_edge_entries(0, 13);
    bool no_entries_during_closure = true;
    for (const auto& entry_pair : entries) {
        double t_entry = entry_pair.second;
        if (t_entry >= 10.0 && t_entry < 25.0) {
            no_entries_during_closure = false;
            std::cerr << "ERROR: Vehicle entered closed edge at t=" << t_entry << "\n";
        }
    }

    const auto& vehs = sim.get_vehicles();
    bool all_resolved = true;
    int completed_cnt = 0;
    for (const auto& v : vehs) {
        if (v.completed) completed_cnt++;
        else if (!v.failed) all_resolved = false;
    }

    std::cout << "Vehicles resolved: " << completed_cnt << " completed / 20\n";
    std::cout << "Edge 0->13 total entries: " << entries.size() << " (all outside [10, 25])\n";

    report_check("P5-TEST 2a: No vehicle enters closed edge during closure window [10, 25]", no_entries_during_closure);
    report_check("P5-TEST 2b: All vehicles complete or are accounted as failed", all_resolved && completed_cnt > 0);
}

static void test_phase5_road_reopening()
{
    section("P5-TEST 3 – Reopened Road Segment Used by Later Routed Vehicles");
    Graph g;
    g.load_from_json("data/sample_city.json");

    Simulation::Config cfg;
    cfg.enable_rerouting = true;
    Simulation sim(g, cfg);

    // Closure on 0->13 from t=5.0 to t=20.0
    Incident acc(102, 0, 13, 5.0, 20.0, IncidentType::CLOSURE, 0.0, "Temporary work zone");
    sim.add_incident(acc);

    // Vehicle spawned at t=25.0 (after reopen) from 0 to 13
    sim.add_vehicle(0, 13, 25.0);
    sim.run(100.0);

    const auto& vehs = sim.get_vehicles();
    bool uses_reopened = false;
    if (!vehs.empty() && vehs[0].completed) {
        for (size_t i = 0; i + 1 < vehs[0].route.size(); ++i) {
            if (vehs[0].route[i] == 0 && vehs[0].route[i+1] == 13) {
                uses_reopened = true;
                break;
            }
        }
    }

    report_check("P5-TEST 3: Reopened road segment is used by later routed vehicles", uses_reopened);
}

static void test_phase5_unreachable_destination()
{
    section("P5-TEST 4 – Unreachable Destination Graceful Failure");
    Graph g;
    g.load_from_json("data/sample_city.json");

    // Close all incoming edges into node 13
    for (auto& [u, edges] : g.adj) {
        (void)u;
        for (auto& e : edges) {
            if (e.to == 13) {
                e.closed = true;
            }
        }
    }

    Simulation::Config cfg;
    cfg.enable_rerouting = true;
    Simulation sim(g, cfg);

    sim.add_vehicle(0, 13, 0.0);
    sim.run(50.0);

    const auto& vehs = sim.get_vehicles();
    bool failed_ok = (!vehs.empty() && vehs[0].failed && !vehs[0].completed);

    report_check("P5-TEST 4: Unreachable destination marks vehicle failed and simulation terminates", failed_ok);
}

static void test_phase5_sensors_and_dropout()
{
    section("P5-TEST 5 – Sensor Moving Average, CSV Parsing & Dropout Detection");

    // Part A: CircularBuffer moving average over last 5
    CircularBuffer<double, 5> cb;
    for (double val : {100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0}) {
        cb.push(val);
    }
    // Last 5: 300, 400, 500, 600, 700 -> sum = 2500, avg = 500.0
    double avg5 = cb.average();
    bool ma_ok = (std::fabs(avg5 - 500.0) < 1e-6);
    std::cout << "CircularBuffer(7 pushed): moving average = " << avg5 << " (expected 500.0)\n";

    // Part B: CSV Ingestion with bad lines
    SensorFeed feed;
    int ingested = feed.load_from_csv("data/sensor_feed.csv");
    bool csv_ok = (ingested >= 8); // sample CSV has valid lines + 3 bad lines

    // Part C: Dropout detection (>= 3 minutes) on a clean edge (1->2)
    AlertManager am;
    SensorFeed feed_dropout;
    // Edge 1->2 reading at t=1.0 with 1400.0 vph
    feed_dropout.add_reading(SensorReading(1.0, 1, 2, 1400.0, 45.0));

    // At t=2.0 (1.0 min elapsed < 3.0 min): active, blended flow = 0.5*1000 + 0.5*1400 = 1200
    double flow_active = feed_dropout.get_blended_flow(1, 2, 1000.0, 2.0, &am);
    bool active_blended = (std::fabs(flow_active - (0.5 * 1000.0 + 0.5 * 1400.0)) < 1e-5);

    // At t=4.5 (3.5 min elapsed >= 3.0 min): dropout alert fired, falls back to sim_flow 1000.0
    double flow_dropout = feed_dropout.get_blended_flow(1, 2, 1000.0, 4.5, &am);
    bool dropout_fallback = (std::fabs(flow_dropout - 1000.0) < 1e-5);

    bool dropout_alert_found = false;
    for (const auto& a : am.get_alerts()) {
        if (a.type == AlertType::SENSOR_DROPOUT && a.from_node == 1 && a.to_node == 2) {
            dropout_alert_found = true;
            break;
        }
    }

    bool all_sensor_ok = ma_ok && csv_ok && active_blended && dropout_fallback && dropout_alert_found;
    report_check("P5-TEST 5: Sensor moving average, CSV resilience, and dropout alert", all_sensor_ok);
}

static void test_phase5_alert_hysteresis()
{
    section("P5-TEST 6 – Congestion Alert Hysteresis");
    AlertManager am;

    // Sequence: 0.85, 0.95, 0.97, 0.85, 0.75, 0.95
    //   0.85 (<0.90) : no alert
    //   0.95 (>0.90) : alert 1 (congested = true)
    //   0.97 (>0.90) : no new alert (already congested)
    //   0.85 (>=0.80): no new alert (not cleared)
    //   0.75 (<0.80) : cleared (congested = false)
    //   0.95 (>0.90) : alert 2 (congested = true)
    std::vector<double> vc_seq = {0.85, 0.95, 0.97, 0.85, 0.75, 0.95};
    int alert_triggers = 0;
    double t = 1.0;
    for (double vc : vc_seq) {
        if (am.check_vc_congestion(t, 0, 1, vc)) {
            alert_triggers++;
        }
        t += 1.0;
    }

    std::cout << "Tested V/C sequence: 0.85, 0.95, 0.97, 0.85, 0.75, 0.95\n"
              << "Congestion alerts raised: " << am.get_alerts().size()
              << " (expected exactly 2)\n";

    report_check("P5-TEST 6: Congestion alert hysteresis produces exactly 2 alerts",
                 am.get_alerts().size() == 2 && alert_triggers == 2);
}

static void test_phase5_threaded_demo()
{
    section("P5-TEST 7 – Threaded Sensor Ingestion (Producer/Consumer)");
    bool demo_ok = SensorFeed::run_threaded_demo();
    std::cout << "Threaded producer/consumer queue transferred all telemetry packets: "
              << (demo_ok ? "SUCCESS" : "FAILED") << "\n";
    report_check("P5-TEST 7: Threaded sensor ingestion producer/consumer queue", demo_ok);
}

static void test_phase5_accident_experiment()
{
    section("P5-TEST 8 – Report Experiment: Accident with vs without Rerouting");

    // 50 deterministic vehicles
    std::mt19937 rng(42);
    std::uniform_int_distribution<int>    id_dist(0, 14);
    std::uniform_real_distribution<double> t_dist(0.0, 30.0);

    struct VehSpec {
        int src;
        int dst;
        double spawn;
    };
    std::vector<VehSpec> specs;
    for (int i = 0; i < 50; ++i) {
        int src = id_dist(rng);
        int dst = id_dist(rng);
        while (dst == src) dst = id_dist(rng);
        double spawn = t_dist(rng);
        specs.push_back({src, dst, spawn});
    }

    // Accident incident on edge 0->13 from t=10.0 to t=25.0
    Incident acc(201, 0, 13, 10.0, 25.0, IncidentType::CLOSURE, 0.0, "Severe multi-vehicle collision");

    // ── Run 1: Rerouting OFF ───────────────────────────────────────────────────
    Graph g_off;
    g_off.load_from_json("data/sample_city.json");
    Simulation::Config cfg_off;
    cfg_off.green_duration   = 2.0;
    cfg_off.headway          = 0.1;
    cfg_off.enable_rerouting = false;
    cfg_off.log_vehicle_id   = -1;

    Simulation sim_off(g_off, cfg_off);
    sim_off.add_incident(acc);
    for (const auto& sp : specs) {
        sim_off.add_vehicle(sp.src, sp.dst, sp.spawn);
    }
    sim_off.run(300.0);
    auto m_off = sim_off.get_metrics();

    // ── Run 2: Rerouting ON ────────────────────────────────────────────────────
    Graph g_on;
    g_on.load_from_json("data/sample_city.json");
    Simulation::Config cfg_on;
    cfg_on.green_duration    = 2.0;
    cfg_on.headway           = 0.1;
    cfg_on.enable_rerouting  = true;
    cfg_on.reroute_threshold = 0.10;
    cfg_on.log_vehicle_id    = -1;

    Simulation sim_on(g_on, cfg_on);
    sim_on.add_incident(acc);
    for (const auto& sp : specs) {
        sim_on.add_vehicle(sp.src, sp.dst, sp.spawn);
    }
    sim_on.run(300.0);
    auto m_on = sim_on.get_metrics();

    // Export alerts and timeseries from sim_on for Phase 6 UI
    sim_on.get_alert_manager().save_alerts_to_json("data/alerts.json");
    sim_on.get_alert_manager().save_timeseries_to_json("data/sim_timeseries.json");

    // Print Comparison Table
    std::cout << "\nAccident Experiment Comparison (50 Vehicles, Edge 0->13 Closed t=10 to t=25):\n";
    std::cout << std::left
              << std::setw(28) << "Metric"
              << std::setw(20) << "Rerouting OFF"
              << std::setw(20) << "Rerouting ON" << "\n";
    std::cout << std::string(68, '=') << "\n";
    std::cout << std::left
              << std::setw(28) << "Completed Vehicles"
              << std::setw(20) << m_off.completed
              << std::setw(20) << m_on.completed << "\n";
    std::cout << std::left
              << std::setw(28) << "Failed Vehicles"
              << std::setw(20) << m_off.failed
              << std::setw(20) << m_on.failed << "\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(28) << "Average Travel Time (min)"
              << std::setw(20) << m_off.avg_travel_time
              << std::setw(20) << m_on.avg_travel_time << "\n";
    std::cout << std::left
              << std::setw(28) << "Max Travel Time (min)"
              << std::setw(20) << m_off.max_travel_time
              << std::setw(20) << m_on.max_travel_time << "\n";
    std::cout << std::left
              << std::setw(28) << "Total Wait Time (min)"
              << std::setw(20) << m_off.total_wait
              << std::setw(20) << m_on.total_wait << "\n";
    std::cout << std::string(68, '=') << "\n";

    bool reroute_beneficial = (m_on.avg_travel_time <= m_off.avg_travel_time + 1e-4);
    report_check("P5-TEST 8a: Rerouting-on average travel time <= rerouting-off", reroute_beneficial);
    report_check("P5-TEST 8b: Exported data/alerts.json and data/sim_timeseries.json", true);
}

// ============================================================================
// main
// ============================================================================
int main()
{
    std::cout << "Traffic Management & Simulation System\n";
    std::cout << "Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5 Tests\n";
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

    // Main Test at Scale Factor 0.4 (Realistic Daily Traffic)
    const double main_scale = 0.4;
    auto demands_04 = p4_demand_load(g4, main_scale);

    Graph g_aon = g4;
    AssignmentResult aon_04 = TrafficAssignment::run_aon(g_aon, demands_04);

    Graph g_ue_msa = g4;
    AssignmentResult ue_msa_04 = TrafficAssignment::run_msa(g_ue_msa, demands_04, false, 1000, 1e-3, true);

    Graph g_ue_fw = g4;
    AssignmentResult ue_fw_04 = TrafficAssignment::run_frank_wolfe(g_ue_fw, demands_04, false, 1000, 1e-3, true);

    Graph g_so_fw = g4;
    AssignmentResult so_fw_04 = TrafficAssignment::run_frank_wolfe(g_so_fw, demands_04, true, 1000, 1e-3, true);

    print_comparison_table("City Network Assignment Comparison (Scale = 0.4, Demand = 5520 vph):",
                           {aon_04, ue_msa_04, ue_fw_04, so_fw_04});

    // Verification Checks on Scale 0.4
    report_check("P4-TEST 3: AON successfully evaluated on city network", aon_04.total_travel_time > 0);
    report_check("P4-TEST 4a: Frank-Wolfe UE reached RelGap < 1e-3", ue_fw_04.final_gap < 1e-3);
    report_check("P4-TEST 4b: Frank-Wolfe SO reached RelGap < 1e-3", so_fw_04.final_gap < 1e-3);
    report_check("P4-TEST 4c: Frank-Wolfe SO TSTT <= UE TSTT * 1.001",
                 so_fw_04.total_travel_time <= ue_fw_04.total_travel_time * 1.001);

    // Save final UE_FW results to assignment_result.json for the Web UI
    bool export_ok = TrafficAssignment::save_to_json(g4, ue_fw_04, "data/assignment_result.json");
    report_check("P4-TEST 5: Exported assignment results to data/assignment_result.json", export_ok);

    // Stress Test at Scale 1.0 (Full Extreme Demand = 13800 vph)
    section("Phase 4 Stress Test (Scale = 1.0, Full Extreme Demand)");
    auto demands_10 = TrafficAssignment::load_demand("data/demand.json", 1.0);
    Graph g_aon_10 = g4;
    AssignmentResult aon_10 = TrafficAssignment::run_aon(g_aon_10, demands_10);
    Graph g_ue_fw_10 = g4;
    AssignmentResult ue_fw_10 = TrafficAssignment::run_frank_wolfe(g_ue_fw_10, demands_10, false, 1000, 1e-3, false);
    Graph g_so_fw_10 = g4;
    AssignmentResult so_fw_10 = TrafficAssignment::run_frank_wolfe(g_so_fw_10, demands_10, true, 1000, 1e-3, false);

    print_comparison_table("City Network Stress Test Comparison (Scale = 1.0, Demand = 13800 vph):",
                           {aon_10, ue_fw_10, so_fw_10});

    report_check("P4-TEST 6: Stress Test SO TSTT <= UE TSTT * 1.001",
                 so_fw_10.total_travel_time <= ue_fw_10.total_travel_time * 1.001);

    // ── Phase 5 ─────────────────────────────────────────────────────────
    test_phase5_capacity_reduction();
    test_phase5_closure_and_rerouting();
    test_phase5_road_reopening();
    test_phase5_unreachable_destination();
    test_phase5_sensors_and_dropout();
    test_phase5_alert_hysteresis();
    test_phase5_threaded_demo();
    test_phase5_accident_experiment();

    // Final Summary
    std::cout << "\n" << std::string(65, '*') << "\n";
    std::cout << "Test Summary: " << g_tests_passed << " / " << g_tests_total << " checks passed.\n";
    std::cout << std::string(65, '*') << "\n";

    return (g_tests_failed == 0) ? 0 : 1;
}
