// =============================================================================
// assignment.cpp  –  Traffic Assignment Implementations (Phase 4)
// All-or-Nothing (AON), Method of Successive Averages (MSA), System Optimal (SO)
// =============================================================================
#include "assignment.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <limits>

// ── Reconstruct path helper ──────────────────────────────────────────────────
static std::vector<int>
reconstruct_path(const std::unordered_map<int, int>& parent, int src, int dst)
{
    std::vector<int> path;
    int cur = dst;
    while (cur != -1) {
        path.push_back(cur);
        auto it = parent.find(cur);
        cur = (it != parent.end()) ? it->second : -1;
    }
    std::reverse(path.begin(), path.end());
    if (!path.empty() && path.front() != src) path.clear();
    return path;
}

// ── Custom Dijkstra with arbitrary non-negative edge weight function ─────────
// Time complexity: O((V + E) log V) using student MinHeap with lazy deletion.
PathResult TrafficAssignment::dijkstra_custom(
    const Graph& g,
    int src,
    int dst,
    const std::function<double(int from, int to, const Edge& e)>& weight_fn)
{
    const double INF = std::numeric_limits<double>::infinity();

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int>    parent;
    std::unordered_set<int>         visited;

    for (const auto& kv : g.nodes) {
        dist[kv.first]   = INF;
        parent[kv.first] = -1;
    }
    dist[src] = 0.0;

    MinHeap<int> heap;
    heap.push(0.0, src);

    int nodes_explored = 0;

    while (!heap.empty()) {
        auto item = heap.pop();
        int    u = item.value;
        double d = item.priority;

        if (visited.count(u)) continue;  // lazy deletion
        visited.insert(u);
        ++nodes_explored;

        if (u == dst) break;

        auto adj_it = g.adj.find(u);
        if (adj_it == g.adj.end()) continue;

        for (const Edge& e : adj_it->second) {
            if (e.closed) continue;

            double weight = weight_fn(u, e.to, e);
            if (weight < 0.0) weight = 0.0; // guard non-negative

            double new_dist = d + weight;
            if (new_dist < dist[e.to]) {
                dist[e.to]   = new_dist;
                parent[e.to] = u;
                heap.push(new_dist, e.to);
            }
        }
    }

    PathResult res;
    res.nodes_explored = nodes_explored;
    double best = dist.count(dst) ? dist[dst] : INF;
    res.found      = (best < INF / 2.0);
    res.total_time = res.found ? best : -1.0;

    if (res.found)
        res.path = reconstruct_path(parent, src, dst);

    return res;
}

// ── auxiliary_aon ─────────────────────────────────────────────────────────────
// Finds the shortest path for each OD demand using weight_fn and assigns
// 100% of that demand onto the path links.
// Time complexity: O(|OD| * (V + E) log V)
std::map<std::pair<int, int>, double>
TrafficAssignment::auxiliary_aon(
    const Graph& g,
    const std::vector<ODDemand>& demands,
    const std::function<double(int from, int to, const Edge& e)>& weight_fn)
{
    std::map<std::pair<int, int>, double> y;

    // Initialize all directed graph edges with 0 flow
    for (const auto& kv : g.adj) {
        for (const Edge& e : kv.second) {
            y[{e.from, e.to}] = 0.0;
        }
    }

    for (const auto& dem : demands) {
        if (dem.demand <= 0.0 || dem.origin == dem.destination) continue;

        PathResult pr = dijkstra_custom(g, dem.origin, dem.destination, weight_fn);
        if (!pr.found || pr.path.size() < 2) continue;

        for (size_t i = 0; i + 1 < pr.path.size(); ++i) {
            int u = pr.path[i];
            int v = pr.path[i + 1];
            y[{u, v}] += dem.demand;
        }
    }

    return y;
}

// ── compute_relative_gap ──────────────────────────────────────────────────────
// Relative Gap (RGAP) measures proximity to equilibrium:
//   RGAP = ( sum_a x_a * c_a(x_a) - sum_a y_a * c_a(x_a) ) / ( sum_a x_a * c_a(x_a) )
// In User Equilibrium, c_a(x_a) = t_a(x_a).
// In System Optimal,    c_a(x_a) = m_a(x_a) = marginal cost.
// At equilibrium, travelers are on the shortest routes, so y achieves the same
// total cost as x, driving RGAP to 0.
double TrafficAssignment::compute_relative_gap(
    const Graph& g,
    const std::map<std::pair<int, int>, double>& current_x,
    const std::map<std::pair<int, int>, double>& aux_y,
    const std::function<double(int from, int to, const Edge& e)>& cost_fn)
{
    double total_current_cost = 0.0;
    double min_possible_cost  = 0.0;

    for (const auto& kv : g.adj) {
        int u = kv.first;
        for (const Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            double x_a = current_x.count(key) ? current_x.at(key) : 0.0;
            double y_a = aux_y.count(key)     ? aux_y.at(key)     : 0.0;
            double c_a = cost_fn(u, e.to, e);

            total_current_cost += x_a * c_a;
            min_possible_cost  += y_a * c_a;
        }
    }

    if (total_current_cost <= 1e-9) return 0.0;

    double gap = (total_current_cost - min_possible_cost) / total_current_cost;
    return (gap < 0.0) ? 0.0 : gap; // protect against floating precision noise
}

// ── compute_tstt ──────────────────────────────────────────────────────────────
// Total System Travel Time: sum over all edges of (flow * actual BPR travel time).
// Flow is in veh/hour, travel time is in minutes.
// Result unit: vehicle-minutes per hour.
double TrafficAssignment::compute_tstt(
    const Graph& g,
    const std::map<std::pair<int, int>, double>& flows)
{
    double tstt = 0.0;
    for (const auto& kv : g.adj) {
        int u = kv.first;
        for (const Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            double x = flows.count(key) ? flows.at(key) : 0.0;
            if (x <= 0.0) continue;

            double ratio = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;
            double t = e.free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0));
            tstt += x * t;
        }
    }
    return tstt;
}

// ── apply_flows_to_graph ──────────────────────────────────────────────────────
void TrafficAssignment::apply_flows_to_graph(
    Graph& g,
    const std::map<std::pair<int, int>, double>& flows)
{
    for (auto& kv : g.adj) {
        int u = kv.first;
        for (Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            e.flow = flows.count(key) ? flows.at(key) : 0.0;
        }
    }
}

// ── run_aon ───────────────────────────────────────────────────────────────────
AssignmentResult TrafficAssignment::run_aon(
    Graph& g,
    const std::vector<ODDemand>& demands)
{
    AssignmentResult res;
    res.method     = "AON";
    res.iterations = 1;
    res.final_gap  = 0.0;

    // Pure free-flow AON shortest path weight
    auto free_flow_fn = [](int /*u*/, int /*v*/, const Edge& e) {
        return e.free_flow_time;
    };

    res.flows = auxiliary_aon(g, demands, free_flow_fn);
    res.total_travel_time = compute_tstt(g, res.flows);

    // Compute detailed stats
    double max_vc = 0.0;
    int over_cap = 0;

    for (const auto& kv : g.adj) {
        int u = kv.first;
        for (const Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            double x = res.flows.count(key) ? res.flows.at(key) : 0.0;
            double vc = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;
            double t  = e.free_flow_time * (1.0 + 0.15 * std::pow(vc, 4.0));

            if (vc > max_vc) max_vc = vc;
            if (vc > 1.0) ++over_cap;

            EdgeFlowDetail d;
            d.from = u;
            d.to   = e.to;
            d.from_name = g.nodes.count(u) ? g.nodes.at(u).name : std::to_string(u);
            d.to_name   = g.nodes.count(e.to) ? g.nodes.at(e.to).name : std::to_string(e.to);
            d.flow = x;
            d.capacity = e.capacity;
            d.free_flow_time = e.free_flow_time;
            d.travel_time = t;
            d.vc_ratio = vc;
            res.edge_details.push_back(d);
        }
    }

    res.max_vc = max_vc;
    res.edges_over_capacity = over_cap;
    apply_flows_to_graph(g, res.flows);
    return res;
}

// ── run_msa ───────────────────────────────────────────────────────────────────
// Method of Successive Averages:
// Step 0: x^(1) = AON on free-flow times.
// Step n (n = 2, 3, ...):
//   1. Update link costs using x^(n-1).
//   2. Run AON with updated costs to get auxiliary flow y^(n).
//   3. Compute relative gap. If gap < target_gap, terminate!
//   4. Update: x^(n) = x^(n-1) + (1/n) * (y^(n) - x^(n-1)).
//
// Convergence rate: O(1/n) step-size guarantees convergence for convex objectives.
AssignmentResult TrafficAssignment::run_msa(
    Graph& g,
    const std::vector<ODDemand>& demands,
    bool system_optimal,
    int max_iter,
    double target_gap,
    bool verbose)
{
    AssignmentResult res;
    res.method = system_optimal ? "SO_MSA" : "UE_MSA";

    // Step 0: Initial AON at free-flow times
    auto ff_weight_fn = [](int /*u*/, int /*v*/, const Edge& e) {
        return e.free_flow_time;
    };
    std::map<std::pair<int, int>, double> current_x = auxiliary_aon(g, demands, ff_weight_fn);

    int iter = 1;
    double last_gap = 1.0;

    if (verbose) {
        std::cout << "\nStarting MSA (" << res.method << ") for up to " << max_iter
                  << " iterations (target gap=" << target_gap << ")...\n";
    }

    for (iter = 2; iter <= max_iter; ++iter) {
        auto cost_fn = [&](int u, int v, const Edge& e) {
            auto key = std::make_pair(u, v);
            double x = current_x.count(key) ? current_x.at(key) : 0.0;
            double ratio = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;

            if (!system_optimal) {
                // User Equilibrium: BPR travel time
                return e.free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0));
            } else {
                // System Optimal: Marginal cost = t(x) + x * t'(x)
                return e.free_flow_time * (1.0 + 0.75 * std::pow(ratio, 4.0));
            }
        };

        // Auxiliary flow y^(iter)
        std::map<std::pair<int, int>, double> y = auxiliary_aon(g, demands, cost_fn);

        // Compute relative gap
        double gap = compute_relative_gap(g, current_x, y, cost_fn);
        res.gap_history.push_back(gap);
        last_gap = gap;

        if (verbose && (iter == 2 || iter % 50 == 0 || gap < target_gap || iter == max_iter)) {
            double cur_tstt = compute_tstt(g, current_x);
            std::cout << "  Iter " << std::setw(4) << iter
                      << " │ Step λ=" << std::fixed << std::setprecision(4) << (1.0 / iter)
                      << " │ RelGap=" << std::setprecision(6) << gap
                      << " │ TSTT=" << std::setprecision(1) << cur_tstt << " veh-min/h\n";
        }

        if (gap < target_gap) {
            break;
        }

        // MSA update step: x^(n) = x^(n-1) + (1/n) * (y^(n) - x^(n-1))
        double step = 1.0 / static_cast<double>(iter);
        for (auto& kv : current_x) {
            double y_val = y.count(kv.first) ? y.at(kv.first) : 0.0;
            kv.second += step * (y_val - kv.second);
        }
    }

    res.iterations = std::min(iter, max_iter);
    res.final_gap  = last_gap;
    res.flows      = current_x;
    res.total_travel_time = compute_tstt(g, current_x);

    // Compute detailed stats
    double max_vc = 0.0;
    int over_cap = 0;

    for (const auto& kv : g.adj) {
        int u = kv.first;
        for (const Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            double x = res.flows.count(key) ? res.flows.at(key) : 0.0;
            double vc = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;
            double t  = e.free_flow_time * (1.0 + 0.15 * std::pow(vc, 4.0));

            if (vc > max_vc) max_vc = vc;
            if (vc > 1.0) ++over_cap;

            EdgeFlowDetail d;
            d.from = u;
            d.to   = e.to;
            d.from_name = g.nodes.count(u) ? g.nodes.at(u).name : std::to_string(u);
            d.to_name   = g.nodes.count(e.to) ? g.nodes.at(e.to).name : std::to_string(e.to);
            d.flow = x;
            d.capacity = e.capacity;
            d.free_flow_time = e.free_flow_time;
            d.travel_time = t;
            d.vc_ratio = vc;
            res.edge_details.push_back(d);
        }
    }

    res.max_vc = max_vc;
    res.edges_over_capacity = over_cap;
    apply_flows_to_graph(g, res.flows);
    return res;
}

// ── run_frank_wolfe ───────────────────────────────────────────────────────────
// Frank-Wolfe Algorithm with Bisection Line Search:
// At each iteration, finds the exact optimal step size lambda in [0, 1] that
// minimizes the convex objective along the descent direction d = y - x.
// Guarantees monotonic decrease in objective and much faster convergence than MSA.
AssignmentResult TrafficAssignment::run_frank_wolfe(
    Graph& g,
    const std::vector<ODDemand>& demands,
    bool system_optimal,
    int max_iter,
    double target_gap,
    bool verbose)
{
    AssignmentResult res;
    res.method = system_optimal ? "SO_FW" : "UE_FW";

    // Step 0: Initial AON at free-flow times
    auto ff_weight_fn = [](int /*u*/, int /*v*/, const Edge& e) {
        return e.free_flow_time;
    };
    std::map<std::pair<int, int>, double> current_x = auxiliary_aon(g, demands, ff_weight_fn);

    int iter = 1;
    double last_gap = 1.0;

    if (verbose) {
        std::cout << "\nStarting Frank-Wolfe (" << res.method << ") for up to " << max_iter
                  << " iterations (target gap=" << target_gap << ")...\n";
    }

    for (iter = 1; iter <= max_iter; ++iter) {
        auto cost_fn = [&](int u, int v, const Edge& e) {
            auto key = std::make_pair(u, v);
            double x = current_x.count(key) ? current_x.at(key) : 0.0;
            double ratio = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;

            if (!system_optimal) {
                // UE: BPR travel time
                return e.free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0));
            } else {
                // SO: Marginal cost
                return e.free_flow_time * (1.0 + 0.75 * std::pow(ratio, 4.0));
            }
        };

        // Auxiliary flow y^(iter)
        std::map<std::pair<int, int>, double> y = auxiliary_aon(g, demands, cost_fn);

        // Compute relative gap
        double gap = compute_relative_gap(g, current_x, y, cost_fn);
        res.gap_history.push_back(gap);
        last_gap = gap;

        if (verbose && (iter == 1 || iter % 10 == 0 || gap < target_gap || iter == max_iter)) {
            double cur_tstt = compute_tstt(g, current_x);
            std::cout << "  Iter " << std::setw(4) << iter
                      << " │ RelGap=" << std::fixed << std::setprecision(6) << gap
                      << " │ TSTT=" << std::setprecision(1) << cur_tstt << " veh-min/h\n";
        }

        if (gap < target_gap) {
            break;
        }

        // Bisection line search on lambda in [0, 1]
        // Derivative wrt lambda:
        // g(lambda) = sum_a (y_a - x_a) * cost_a(x_a + lambda * (y_a - x_a))
        auto dZ_dlambda = [&](double lambda) -> double {
            double sum = 0.0;
            for (const auto& kv : g.adj) {
                int u = kv.first;
                for (const Edge& e : kv.second) {
                    auto key = std::make_pair(u, e.to);
                    double x_a = current_x.count(key) ? current_x.at(key) : 0.0;
                    double y_a = y.count(key) ? y.at(key) : 0.0;
                    double w_a = x_a + lambda * (y_a - x_a);
                    if (w_a < 0.0) w_a = 0.0;

                    double ratio = (e.capacity > 0.0) ? (w_a / e.capacity) : 0.0;
                    double cost = (!system_optimal)
                        ? e.free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0))
                        : e.free_flow_time * (1.0 + 0.75 * std::pow(ratio, 4.0));

                    sum += (y_a - x_a) * cost;
                }
            }
            return sum;
        };

        double lo = 0.0, hi = 1.0;
        double opt_lambda = 0.0;
        if (dZ_dlambda(0.0) >= 0.0) {
            opt_lambda = 0.0;
        } else if (dZ_dlambda(1.0) <= 0.0) {
            opt_lambda = 1.0;
        } else {
            for (int b = 0; b < 25; ++b) {
                double mid = 0.5 * (lo + hi);
                double val = dZ_dlambda(mid);
                if (val < 0.0) lo = mid;
                else hi = mid;
            }
            opt_lambda = 0.5 * (lo + hi);
        }

        // Update flows
        for (auto& kv : current_x) {
            double y_val = y.count(kv.first) ? y.at(kv.first) : 0.0;
            kv.second += opt_lambda * (y_val - kv.second);
        }
    }

    res.iterations = std::min(iter, max_iter);
    res.final_gap  = last_gap;
    res.flows      = current_x;
    res.total_travel_time = compute_tstt(g, current_x);

    // Compute detailed stats
    double max_vc = 0.0;
    int over_cap = 0;

    for (const auto& kv : g.adj) {
        int u = kv.first;
        for (const Edge& e : kv.second) {
            auto key = std::make_pair(u, e.to);
            double x = res.flows.count(key) ? res.flows.at(key) : 0.0;
            double vc = (e.capacity > 0.0) ? (x / e.capacity) : 0.0;
            double t  = e.free_flow_time * (1.0 + 0.15 * std::pow(vc, 4.0));

            if (vc > max_vc) max_vc = vc;
            if (vc > 1.0) ++over_cap;

            EdgeFlowDetail d;
            d.from = u;
            d.to   = e.to;
            d.from_name = g.nodes.count(u) ? g.nodes.at(u).name : std::to_string(u);
            d.to_name   = g.nodes.count(e.to) ? g.nodes.at(e.to).name : std::to_string(e.to);
            d.flow = x;
            d.capacity = e.capacity;
            d.free_flow_time = e.free_flow_time;
            d.travel_time = t;
            d.vc_ratio = vc;
            res.edge_details.push_back(d);
        }
    }

    res.max_vc = max_vc;
    res.edges_over_capacity = over_cap;
    apply_flows_to_graph(g, res.flows);
    return res;
}

// ── Default Demand Table ──────────────────────────────────────────────────────
std::vector<ODDemand> TrafficAssignment::get_default_demand(double demand_scale)
{
    std::vector<ODDemand> raw = {
        {10, 8,  1800.0, "West_Suburb to East_Suburb (cross-city corridor)"},
        {1,  5,  1400.0, "Airport to Business_District (business commute)"},
        {7,  9,  1600.0, "North_Suburb to South_Suburb (north-south arterial)"},
        {0,  11, 1500.0, "Central_Station to Tech_Park (tech commute)"},
        {9,  7,  1300.0, "South_Suburb to North_Suburb (reverse commute)"},
        {12, 3,  1200.0, "Stadium to Hospital (crosstown)"},
        {1,  11, 1400.0, "Airport to Tech_Park (visitor travel)"},
        {10, 14, 1100.0, "West_Suburb to Industrial_Zone (worker shift)"},
        {2,  8,  1300.0, "University to East_Suburb (student/staff traffic)"},
        {6,  13, 1200.0, "City_Park to Old_Market (shopping & leisure)"}
    };
    for (auto& d : raw) d.demand *= demand_scale;
    return raw;
}

// ── load_demand ───────────────────────────────────────────────────────────────
// Robust JSON parser for demand.json with configurable scale factor
std::vector<ODDemand> TrafficAssignment::load_demand(const std::string& filepath, double demand_scale)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[Assignment] Cannot open " << filepath << " – using default demand.\n";
        return get_default_demand(demand_scale);
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    std::string json = ss.str();

    std::vector<ODDemand> list;
    size_t pos = 0;

    while ((pos = json.find('{', pos)) != std::string::npos) {
        size_t end = json.find('}', pos);
        if (end == std::string::npos) break;

        std::string block = json.substr(pos, end - pos + 1);
        pos = end + 1;

        auto get_val = [&](const std::string& key) -> std::string {
            size_t kpos = block.find("\"" + key + "\"");
            if (kpos == std::string::npos) return "";
            size_t colon = block.find(':', kpos);
            if (colon == std::string::npos) return "";
            size_t vstart = colon + 1;
            while (vstart < block.size() && (block[vstart] == ' ' || block[vstart] == '\t' || block[vstart] == '"'))
                ++vstart;
            size_t vend = vstart;
            while (vend < block.size() && block[vend] != ',' && block[vend] != '}' && block[vend] != '"' && block[vend] != '\r' && block[vend] != '\n')
                ++vend;
            return block.substr(vstart, vend - vstart);
        };

        try {
            std::string o_str = get_val("origin");
            std::string d_str = get_val("destination");
            std::string m_str = get_val("demand");
            std::string desc  = get_val("description");

            if (!o_str.empty() && !d_str.empty() && !m_str.empty()) {
                int orig = std::stoi(o_str);
                int dest = std::stoi(d_str);
                double dem = std::stod(m_str) * demand_scale;
                list.emplace_back(orig, dest, dem, desc);
            }
        } catch (...) {
            // skip malformed entries
        }
    }

    if (list.empty()) {
        std::cerr << "[Assignment] No valid demand entries found in " << filepath << " – using default.\n";
        return get_default_demand(demand_scale);
    }

    return list;
}

// ── save_to_json ──────────────────────────────────────────────────────────────
bool TrafficAssignment::save_to_json(
    const Graph& /*g*/,
    const AssignmentResult& res,
    const std::string& filepath)
{
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "[Assignment] Error: Unable to open " << filepath << " for writing.\n";
        return false;
    }

    out << "{\n";
    out << "  \"summary\": {\n";
    out << "    \"method\": \"" << res.method << "\",\n";
    out << "    \"iterations\": " << res.iterations << ",\n";
    out << "    \"final_gap\": " << std::fixed << std::setprecision(6) << res.final_gap << ",\n";
    out << "    \"total_travel_time_veh_min\": " << std::fixed << std::setprecision(2) << res.total_travel_time << ",\n";
    out << "    \"total_travel_time_veh_hrs\": " << std::fixed << std::setprecision(2) << (res.total_travel_time / 60.0) << ",\n";
    out << "    \"max_vc\": " << std::fixed << std::setprecision(3) << res.max_vc << ",\n";
    out << "    \"edges_over_capacity\": " << res.edges_over_capacity << "\n";
    out << "  },\n";

    out << "  \"edges\": [\n";
    for (size_t i = 0; i < res.edge_details.size(); ++i) {
        const auto& d = res.edge_details[i];
        out << "    {\n";
        out << "      \"from\": " << d.from << ",\n";
        out << "      \"to\": " << d.to << ",\n";
        out << "      \"from_name\": \"" << d.from_name << "\",\n";
        out << "      \"to_name\": \"" << d.to_name << "\",\n";
        out << "      \"flow\": " << std::fixed << std::setprecision(1) << d.flow << ",\n";
        out << "      \"capacity\": " << std::fixed << std::setprecision(1) << d.capacity << ",\n";
        out << "      \"free_flow_time\": " << std::fixed << std::setprecision(2) << d.free_flow_time << ",\n";
        out << "      \"travel_time\": " << std::fixed << std::setprecision(3) << d.travel_time << ",\n";
        out << "      \"vc_ratio\": " << std::fixed << std::setprecision(3) << d.vc_ratio << "\n";
        out << "    }" << (i + 1 < res.edge_details.size() ? ",\n" : "\n");
    }
    out << "  ]\n";
    out << "}\n";

    return true;
}
