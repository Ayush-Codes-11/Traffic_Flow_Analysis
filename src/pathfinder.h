// =============================================================================
// pathfinder.h  –  Shared result type and function declarations for
//                  Dijkstra and A* (Phase 2)
// =============================================================================
#pragma once

#include <vector>
#include "graph.h"

// =============================================================================
// PathResult
// Returned by both dijkstra() and astar().
// =============================================================================
struct PathResult {
    bool             found;           // was the destination reachable?
    double           total_time;      // total BPR travel time (minutes)
    int              nodes_explored;  // how many nodes were fully processed
    std::vector<int> path;            // node ids from source to destination
};

// =============================================================================
// dijkstra
// Finds the shortest-time path from src to dst using BPR travel times.
// Closed edges are treated as impassable.
// Time complexity: O((V + E) log V) with a binary min-heap.
// =============================================================================
PathResult dijkstra(const Graph& g, int src, int dst);

// =============================================================================
// astar
// Same goal as Dijkstra, but uses a Euclidean heuristic to guide the search
// toward the destination.  Usually explores far fewer nodes than Dijkstra.
// Time complexity: O((V + E) log V) worst-case, much less in practice.
// =============================================================================
PathResult astar(const Graph& g, int src, int dst);
