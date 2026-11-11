// =============================================================================
// dijkstra.cpp  –  Dijkstra's shortest-path algorithm
// Phase 2
//
// ALGORITHM OVERVIEW (viva summary):
//   We maintain a distance table dist[v] = "shortest time found so far from
//   source to v" and a min-heap ordered by that distance.
//
//   Each iteration we pop the unvisited node u with the smallest dist[u],
//   mark it as finalised (visited), then RELAX each of its outgoing edges:
//   if going through u gives a shorter path to neighbour v, update dist[v]
//   and push (new_dist, v) into the heap.
//
//   LAZY DELETION: instead of decreasing a key inside the heap, we just push
//   a new, better entry.  When we pop an entry for a node we have already
//   visited, we simply skip it.
//
// TIME COMPLEXITY:
//   O((V + E) log V)
//   – Each node is popped at most once: O(V log V) for all pops.
//   – Each edge causes at most one push: O(E log V) for all pushes.
//   With lazy deletion, the heap can hold up to O(E) items, but log(E) ≈ log(V)
//   for sparse graphs, so the bound stays O((V+E) log V).
//
// WHY DIJKSTRA DOESN'T WORK WITH NEGATIVE WEIGHTS:
//   Once we finalise a node u (mark it visited), we assume dist[u] is optimal.
//   A negative-weight edge could later offer a shorter path to u through a
//   node we haven't yet visited, breaking this assumption.
// =============================================================================
#include "pathfinder.h"
#include "minheap.h"
#include "graph.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>   // std::reverse
#include <limits>

// Reconstruct the path by following parent pointers from dst back to src.
static std::vector<int>
reconstruct_path(const std::unordered_map<int,int>& parent, int src, int dst)
{
    std::vector<int> path;
    int cur = dst;
    while (cur != -1) {
        path.push_back(cur);
        // Guard against missing parent (shouldn't happen but be safe)
        auto it = parent.find(cur);
        cur = (it != parent.end()) ? it->second : -1;
    }
    std::reverse(path.begin(), path.end());
    // Verify path starts at src (sanity check)
    if (!path.empty() && path.front() != src) path.clear();
    return path;
}

// =============================================================================
// dijkstra
// =============================================================================
PathResult dijkstra(const Graph& g, int src, int dst)
{
    const double INF = std::numeric_limits<double>::infinity();

    // ── Initialise ────────────────────────────────────────────────────────
    std::unordered_map<int, double> dist;    // best known time to each node
    std::unordered_map<int, int>    parent;  // previous node on best path
    std::unordered_set<int>         visited; // finalised nodes

    for (const auto& kv : g.nodes) {
        dist[kv.first]   = INF;
        parent[kv.first] = -1;
    }
    dist[src] = 0.0;

    // Push source with distance 0
    MinHeap<int> heap;
    heap.push(0.0, src);

    int nodes_explored = 0;

    // ── Main loop ─────────────────────────────────────────────────────────
    while (!heap.empty()) {
        auto item = heap.pop();
        int    u = item.value;
        double d = item.priority;

        // ── Lazy deletion ────────────────────────────────────────────────
        // If we have already finalised u, this is a stale heap entry – skip.
        if (visited.count(u)) continue;
        visited.insert(u);
        ++nodes_explored;

        // Early exit: once we finalise the destination, we're done.
        if (u == dst) break;

        // ── Edge relaxation ───────────────────────────────────────────────
        // For each road leaving u, check if going via u improves the known
        // distance to the neighbour v.
        auto adj_it = g.adj.find(u);
        if (adj_it == g.adj.end()) continue;

        for (const Edge& e : adj_it->second) {
            if (e.closed) continue;             // skip blocked roads

            double new_dist = d + e.bpr_time(); // current travel time (BPR)

            if (new_dist < dist[e.to]) {
                dist[e.to]   = new_dist;
                parent[e.to] = u;
                // Push updated entry (old entry will be skipped by lazy deletion)
                heap.push(new_dist, e.to);
            }
        }
    }

    // ── Build result ──────────────────────────────────────────────────────
    PathResult res;
    res.nodes_explored = nodes_explored;

    double best = dist.count(dst) ? dist[dst] : INF;
    res.found      = (best < INF / 2.0);
    res.total_time = res.found ? best : -1.0;

    if (res.found)
        res.path = reconstruct_path(parent, src, dst);

    return res;
}
