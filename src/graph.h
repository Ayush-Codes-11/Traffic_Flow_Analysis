// =============================================================================
// graph.h  –  Core data model for the Traffic Simulation System
// Phase 1: Graph, Node, Edge, BPR travel-time function
// =============================================================================
#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <cmath>    // pow, sqrt
#include <iostream>

// =============================================================================
// Node
// Represents an intersection (a vertex in the road-network graph).
// x, y are map coordinates – used by the A* heuristic in Phase 2.
// =============================================================================
struct Node {
    int         id;
    std::string name;
    double      x, y;   // 2-D position (arbitrary units, e.g. km)
};

// =============================================================================
// Edge
// Represents a directed road segment (u → v).
// For a two-way road we store two Edge objects (u→v and v→u).
//
// Key fields
//   free_flow_time  t0   – travel time (minutes) when road is empty
//   capacity        c    – max vehicles per hour before heavy slowdown
//   flow            x    – current vehicles per hour on this segment
//
// BPR (Bureau of Public Roads) travel-time formula:
//   t(x) = t0 * (1 + 0.15 * (x/c)^4)
// At x=0  → t = t0          (free flow)
// At x=c  → t = 1.15 * t0   (15 % slower at capacity)
// At x>c  → t grows sharply  (congestion)
// Time complexity of bpr_time(): O(1)
// =============================================================================
struct Edge {
    int    from, to;
    double length;          // km
    double capacity;        // vehicles per hour (vph)
    double free_flow_time;  // minutes (t0)
    double flow;            // current vph on this edge (updated each step)
    bool   closed;          // true if road is blocked (Phase 5)

    // Default constructor – flow starts at 0, road open
    Edge() : from(0), to(0), length(0), capacity(1),
             free_flow_time(1), flow(0), closed(false) {}

    Edge(int f, int t, double len, double cap, double fft)
        : from(f), to(t), length(len), capacity(cap),
          free_flow_time(fft), flow(0), closed(false) {}

    // BPR travel-time  –  the weight used by Dijkstra / A*
    // Returns infinity if the road is closed.
    double bpr_time() const {
        if (closed) return 1e18;          // treat closed road as impassable
        double ratio = flow / capacity;
        return free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0));
    }

    // Volume-to-capacity ratio  (useful for congestion colouring in the UI)
    double vc_ratio() const {
        return (capacity > 0) ? (flow / capacity) : 0.0;
    }
};

// =============================================================================
// Graph
// Adjacency-list representation of the road network.
//
// Space complexity: O(V + E)
//   – nodes    : unordered_map  → O(V) lookup / insert
//   – adj      : unordered_map of vectors → O(1) amortised lookup
//
// All public methods are documented with their time complexity.
// =============================================================================
class Graph {
public:
    // ── Storage ──────────────────────────────────────────────────────────────
    std::unordered_map<int, Node>              nodes; // id  → Node
    std::unordered_map<int, std::vector<Edge>> adj;   // id  → outgoing edges

    // ── Modifiers ─────────────────────────────────────────────────────────────

    // Add a node.  Time: O(1) amortised
    void add_node(const Node& n);

    // Add a directed edge u→v.
    // For bidirectional roads call add_edge twice (u→v and v→u).
    // Time: O(1) amortised
    void add_edge(const Edge& e);

    // ── I/O ───────────────────────────────────────────────────────────────────

    // Load the graph from our sample_city.json format.
    // Returns true on success.  Time: O(V + E)
    bool load_from_json(const std::string& filepath);

    // Pretty-print every node and its outgoing edges.  Time: O(V + E)
    void print() const;

    // ── Queries ───────────────────────────────────────────────────────────────

    // Euclidean distance between nodes a and b.
    // Used as the A* admissible heuristic (never overestimates travel time
    // because speed ≥ 1 unit/min is assumed).
    // Time: O(1)
    double euclidean_dist(int a, int b) const;

    // Return the BPR travel time for the directed edge from→to.
    // Returns -1.0 if no such edge exists.
    // Time: O(degree(from))
    double get_bpr_time(int from, int to) const;

    // Update flow on directed edge from→to.  Time: O(degree(from))
    bool set_flow(int from, int to, double new_flow);

    // Close / reopen a road segment.  Time: O(degree(from))
    bool set_closed(int from, int to, bool closed);

    // ── Counts ────────────────────────────────────────────────────────────────
    int node_count() const { return static_cast<int>(nodes.size()); }
    int edge_count() const;   // counts directed edges
};
