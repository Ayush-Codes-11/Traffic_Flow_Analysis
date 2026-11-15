#ifndef TRAFFIC_PARALLEL_H
#define TRAFFIC_PARALLEL_H

#include "graph.h"
#include <vector>
#include <string>

namespace TrafficParallel {

// Generate a synthetic rectangular grid network of (rows x cols) nodes
// with bidirectional horizontal and vertical edges.
Graph create_grid_network(int rows, int cols);

struct BenchmarkRun {
    int threads;
    double elapsed_ms;
    double speedup;
    double qps;
};

struct BenchmarkSummary {
    std::vector<BenchmarkRun> city_runs;
    std::vector<BenchmarkRun> grid_runs;
    bool city_all_match = true;
    bool grid_all_match = true;
    double best_grid_speedup = 1.0;
};

// Run parallel Dijkstra / A* route queries for N pairs across 1, 2, 4, and max threads.
// Validates path cost equivalence between sequential and multi-threaded runs.
BenchmarkSummary run_parallel_benchmark(const Graph& city_graph,
                                        void (*report_cb)(const std::string&, bool));

} // namespace TrafficParallel

#endif // TRAFFIC_PARALLEL_H
