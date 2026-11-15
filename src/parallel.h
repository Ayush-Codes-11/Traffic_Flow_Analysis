#ifndef TRAFFIC_PARALLEL_H
#define TRAFFIC_PARALLEL_H

#include "graph.h"
#include <vector>
#include <utility>

namespace TrafficParallel {

// Generate a synthetic rectangular grid network of (rows x cols) nodes
// with bidirectional horizontal and vertical edges.
Graph create_grid_network(int rows, int cols);

// Benchmark result structure
struct BenchmarkResult {
    int num_threads;
    double elapsed_ms;
    double speedup;
    double queries_per_sec;
};

// Run parallel Dijkstra / A* route queries for N pairs across 1, 2, 4, and max threads.
// Validates path cost equivalence between sequential and multi-threaded runs.
bool run_parallel_benchmark(const Graph& city_graph,
                            void (*report_cb)(const std::string&, bool));

} // namespace TrafficParallel

#endif // TRAFFIC_PARALLEL_H
