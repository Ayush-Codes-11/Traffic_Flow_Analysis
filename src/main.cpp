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

    // Final Summary
    std::cout << "\n" << std::string(65, '*') << "\n";
    std::cout << "Test Summary: " << g_tests_passed << " / " << g_tests_total << " checks passed.\n";
    std::cout << std::string(65, '*') << "\n";

    return (g_tests_failed == 0) ? 0 : 1;
}

// Exportable C entry point for whitelisted runners / Python ctypes
extern "C" __declspec(dllexport) int run_all_tests()
{
    return main();
}
