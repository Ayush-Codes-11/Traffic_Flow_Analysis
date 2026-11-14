// =============================================================================
// assignment.h  –  Traffic Assignment Models (Phase 4)
// All-or-Nothing (AON), User Equilibrium (UE via MSA), and System Optimal (SO)
//
// ALGORITHMIC OVERVIEW (viva summary):
// ─────────────────────────────────────────────────────────────────────────────
// 1. All-or-Nothing (AON):
//    Assigns 100% of each OD pair's demand to its shortest path under initial
//    (free-flow) conditions. Assumes drivers ignore congestion.
//    Time complexity per AON: O(|OD| * (V + E) log V).
//
// 2. Method of Successive Averages (MSA) for User Equilibrium (UE):
//    Finds Wardrop's User Equilibrium where no traveler can unilaterally
//    switch routes to reduce their own travel time.
//    Iterative algorithm:
//      - Step size at iteration n: lambda_n = 1 / n
//      - Update: x^(n) = x^(n-1) + (1/n) * (y^(n) - x^(n-1))
//      - Here y^(n) is the AON auxiliary flow based on current BPR travel times.
//    Convergence: Stops when relative gap < target_gap (e.g. 1e-3) or max iterations.
//
// 3. System Optimal (SO):
//    Minimizes the total system travel time across all vehicles.
//    Uses the exact same MSA algorithm, but replaces user travel time t(x)
//    with the marginal cost:
//      m(x) = d/dx [x * t(x)] = t(x) + x * t'(x)
//    For BPR: t'(x) = t0 * 0.15 * 4 * x^3 / c^4
//      m(x) = t0 * (1 + 0.75 * (x / c)^4)
// =============================================================================
#pragma once

#include "graph.h"
#include "pathfinder.h"
#include "minheap.h"

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <functional>

// =============================================================================
// ODDemand
// Origin-Destination pair with traffic demand in vehicles per hour (vph).
// =============================================================================
struct ODDemand {
    int         origin;
    int         destination;
    double      demand;       // vph
    std::string description;

    ODDemand() : origin(-1), destination(-1), demand(0.0), description("") {}
    ODDemand(int orig, int dest, double dem, const std::string& desc = "")
        : origin(orig), destination(dest), demand(dem), description(desc) {}
};

// =============================================================================
// EdgeFlowDetail
// Complete operational state of a directed edge after traffic assignment.
// =============================================================================
struct EdgeFlowDetail {
    int         from;
    int         to;
    std::string from_name;
    std::string to_name;
    double      flow;            // assigned vph
    double      capacity;        // vph
    double      free_flow_time;  // minutes
    double      travel_time;     // BPR time at assigned flow (minutes)
    double      vc_ratio;        // flow / capacity
};

// =============================================================================
// AssignmentResult
// Encapsulates the complete output and convergence metrics of an assignment run.
// =============================================================================
struct AssignmentResult {
    std::string method;              // "AON", "UE", "SO"
    int         iterations;          // iterations performed
    double      final_gap;           // final relative gap (0 for single-shot AON)
    double      total_travel_time;   // Total System Travel Time (TSTT) in veh-min/h
    double      max_vc;              // maximum V/C ratio across all edges
    int         edges_over_capacity; // number of directed edges with V/C > 1.0

    // Flows on directed edges: key is std::pair<from, to>
    std::map<std::pair<int, int>, double> flows;

    // Detailed per-edge statistics
    std::vector<EdgeFlowDetail> edge_details;

    // Convergence history: relative gap at each iteration
    std::vector<double> gap_history;

    AssignmentResult()
        : method(""), iterations(0), final_gap(0.0), total_travel_time(0.0),
          max_vc(0.0), edges_over_capacity(0) {}
};

// =============================================================================
// TrafficAssignment
// Static methods for running AON, MSA User Equilibrium, and System Optimal.
// =============================================================================
class TrafficAssignment {
public:
    // Load OD demand table from JSON file. Returns fallback default if missing/invalid.
    static std::vector<ODDemand> load_demand(const std::string& filepath);

    // Hardcoded default demand table (10 OD pairs on sample_city.json)
    static std::vector<ODDemand> get_default_demand();

    // Single-shot All-or-Nothing assignment starting from zero flow.
    static AssignmentResult run_aon(Graph& g, const std::vector<ODDemand>& demands);

    // Method of Successive Averages (MSA) assignment:
    // - If system_optimal is false: finds User Equilibrium (Wardrop UE).
    // - If system_optimal is true : finds System Optimal (SO, minimizes TSTT).
    static AssignmentResult run_msa(Graph& g,
                                    const std::vector<ODDemand>& demands,
                                    bool system_optimal = false,
                                    int max_iter = 100,
                                    double target_gap = 1e-3,
                                    bool verbose = false);

    // Export per-edge flows, capacities, and V/C ratios to JSON for the Web UI.
    static bool save_to_json(const Graph& g,
                             const AssignmentResult& res,
                             const std::string& filepath);

    // Calculate Total System Travel Time: sum over all edges of (flow * bpr_time(flow))
    static double compute_tstt(const Graph& g,
                               const std::map<std::pair<int, int>, double>& flows);

    // Apply computed flows directly to Graph::Edge::flow
    static void apply_flows_to_graph(Graph& g,
                                     const std::map<std::pair<int, int>, double>& flows);

    // Custom Dijkstra supporting arbitrary non-negative edge weight functions.
    // Uses the student MinHeap implementation (NO std::priority_queue).
    static PathResult dijkstra_custom(
        const Graph& g,
        int src,
        int dst,
        const std::function<double(int from, int to, const Edge& e)>& weight_fn);

private:
    // Run AON using a given edge weight function to produce auxiliary flow vector y.
    static std::map<std::pair<int, int>, double>
    auxiliary_aon(const Graph& g,
                  const std::vector<ODDemand>& demands,
                  const std::function<double(int from, int to, const Edge& e)>& weight_fn);

    // Calculate relative gap between current flow x and auxiliary flow y:
    // RGAP = (TSTT(x) - SPTT(y|x)) / TSTT(x)
    static double compute_relative_gap(
        const Graph& g,
        const std::map<std::pair<int, int>, double>& current_x,
        const std::map<std::pair<int, int>, double>& aux_y,
        const std::function<double(int from, int to, const Edge& e)>& cost_fn);
};
