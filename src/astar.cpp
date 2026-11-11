// =============================================================================
// astar.cpp  –  A* shortest-path algorithm
// Phase 2
//
// HOW A* DIFFERS FROM DIJKSTRA:
//   Dijkstra expands nodes in order of g(u) = actual cost from source.
//   A* expands nodes in order of f(u) = g(u) + h(u), where h(u) is a
//   HEURISTIC ESTIMATE of the remaining cost from u to the destination.
//
//   By biasing toward nodes that are geometrically closer to the goal,
//   A* typically explores far fewer nodes than Dijkstra.
//
// THE HEURISTIC – WHY IT MUST BE ADMISSIBLE:
//   An admissible heuristic NEVER OVERESTIMATES the true remaining cost.
//   If h(u) > true_cost(u, dst) for even one node, A* might skip the
//   optimal path and return a suboptimal one.
//
//   We use:
//     h(u) = euclidean_distance(u, dst) / MAX_SPEED
//
//   where MAX_SPEED = 1.5 coordinate-units/min  (conservative upper bound
//   on how fast any vehicle can travel in our city — well above the actual
//   maximum of ~0.85 units/min seen in sample_city.json).
//
//   Because straight-line distance is always ≤ actual road distance, and
//   the actual travel time ≥ road_distance / MAX_SPEED, we have:
//     h(u) = coord_dist / 1.5  ≤  actual_time(u, dst)       ✓ admissible
//
//   The heuristic is also CONSISTENT (monotone): for every edge u→v,
//   h(u) ≤ edge_cost(u,v) + h(v).  Consistency implies admissibility and
//   allows us to use a closed (visited) set safely — the first time we pop
//   a node its g-score is already optimal.
//
// TIME COMPLEXITY:
//   O((V + E) log V) worst case (same as Dijkstra).
//   In practice A* explores only a fraction of V nodes because the
//   heuristic prunes branches that point away from the destination.
// =============================================================================
#include "pathfinder.h"
#include "minheap.h"
#include "graph.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>   // std::reverse
#include <limits>
#include <cmath>       // std::sqrt

// Maximum plausible travel speed in coordinate units per minute.
// Our data has free_flow_time in minutes and coordinates in km-like units.
// The fastest edge in sample_city.json is ~0.85 units/min.
// Using 1.5 gives a comfortable safety margin while still pruning well.
static constexpr double ASTAR_MAX_SPEED = 1.5;

// =============================================================================
// astar
// =============================================================================
PathResult astar(const Graph& g, int src, int dst)
{
    const double INF = std::numeric_limits<double>::infinity();

    // Heuristic: straight-line distance divided by max possible speed.
    // Returns 0 if u IS the destination (h of goal = 0).
    auto heuristic = [&](int u) -> double {
        if (u == dst) return 0.0;
        return g.euclidean_dist(u, dst) / ASTAR_MAX_SPEED;
    };

    // ── Initialise ────────────────────────────────────────────────────────
    std::unordered_map<int, double> g_score; // actual cost from src to u
    std::unordered_map<int, int>    parent;
    std::unordered_set<int>         visited;  // closed set (finalised nodes)

    for (const auto& kv : g.nodes) {
        g_score[kv.first] = INF;
        parent[kv.first]  = -1;
    }
    g_score[src] = 0.0;

    // The heap is ordered by f = g + h (the A* priority).
    MinHeap<int> heap;
    heap.push(heuristic(src), src); // f = 0 + h(src)

    int nodes_explored = 0;

    // ── Main loop ─────────────────────────────────────────────────────────
    while (!heap.empty()) {
        auto item = heap.pop();
        int u = item.value;
        // Note: item.priority is the f-score, not the g-score.

        // Lazy deletion via visited (closed) set.
        // Because h is consistent, the first pop of u is guaranteed optimal.
        if (visited.count(u)) continue;
        visited.insert(u);
        ++nodes_explored;

        if (u == dst) break;

        auto adj_it = g.adj.find(u);
        if (adj_it == g.adj.end()) continue;

        for (const Edge& e : adj_it->second) {
            if (e.closed) continue;

            int    v       = e.to;
            double new_g   = g_score[u] + e.bpr_time(); // actual cost via u

            if (new_g < g_score[v]) {
                g_score[v] = new_g;
                parent[v]  = u;
                // f = g + h  — this is what the heap orders by
                double f = new_g + heuristic(v);
                heap.push(f, v);
            }
        }
    }

    // ── Build result ──────────────────────────────────────────────────────
    PathResult res;
    res.nodes_explored = nodes_explored;

    double best = g_score.count(dst) ? g_score[dst] : INF;
    res.found      = (best < INF / 2.0);
    res.total_time = res.found ? best : -1.0;

    if (res.found) {
        // Reconstruct path from parent pointers
        std::vector<int>& path = res.path;
        int cur = dst;
        while (cur != -1) {
            path.push_back(cur);
            auto it = parent.find(cur);
            cur = (it != parent.end()) ? it->second : -1;
        }
        std::reverse(path.begin(), path.end());
        if (!path.empty() && path.front() != src) path.clear();
    }

    return res;
}
