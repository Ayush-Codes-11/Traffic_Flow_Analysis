// =============================================================================
// simulation.h  –  Traffic Simulation class declaration
// Phase 3
// =============================================================================
#pragma once

#include "graph.h"
#include "vehicle.h"
#include "vehicle_queue.h"
#include "minheap.h"

#include <unordered_map>
#include <vector>
#include <string>

// =============================================================================
// Simulation
//
// Event-driven simulation of vehicles on the road network.
// Uses a MinHeap as the event queue (smallest timestamp = next event).
// Each intersection has a fixed-cycle signal; vehicles queue per directed edge.
//
// Key data structures
//   event_heap : MinHeap<EventData>  — priority queue of future events
//   queues     : per-edge VehicleQueue<int> — vehicles waiting to enter edge
//   edge_active: per-edge count of vehicles currently traversing it
// =============================================================================
class Simulation {
public:
    // ── Configuration ─────────────────────────────────────────────────────────
    struct Config {
        double green_duration = 2.0;  // minutes each outgoing direction is green
                                      // (0.0 = signals disabled, always green)
        double headway        = 0.1;  // min gap between consecutive entries onto edge
        int    log_vehicle_id = 0;    // which vehicle's timeline to print (-1 = none)
    };

    explicit Simulation(Graph& g);
    Simulation(Graph& g, Config cfg);

    // Add one vehicle.  Route is computed with A* using current edge flows.
    void add_vehicle(int src, int dst, double spawn_time);

    // Run until max_time (minutes) or until all events are processed.
    void run(double max_time = 300.0);

    // Print end-of-run summary: completed count, avg time, avg wait, top edges.
    void print_summary() const;

    // Print timestamped timeline for vehicle `vid`.
    void print_vehicle_timeline(int vid) const;

    // Read-only access to vehicle list (for unit tests)
    const std::vector<Vehicle>& get_vehicles() const { return vehicles_; }

private:
    Graph&  g_;
    Config  cfg_;

    std::vector<Vehicle> vehicles_;

    // Per-edge queues (vehicles waiting to enter).
    // Key: from * 100 + to   (safe for ≤ 99 node IDs)
    std::unordered_map<int, VehicleQueue<int>> queues_;

    // Number of vehicles currently ON each directed edge (traversing).
    std::unordered_map<int, int> edge_active_;

    // Per-edge stats recorded throughout the run.
    struct EdgeStat {
        int    from_node  = -1;
        int    to_node    = -1;
        int    peak_queue = 0;
        int    peak_active= 0;
        int    total_in   = 0;    // total vehicles that entered this edge
        double peak_vc    = 0.0;  // peak V/C ratio seen
    };
    std::unordered_map<int, EdgeStat> edge_stats_;

    // The event queue — a MinHeap ordered by event time (priority).
    MinHeap<EventData> event_heap_;

    // Per-vehicle timeline strings (for the chosen log_vehicle_id).
    std::unordered_map<int, std::vector<std::string>> timelines_;

    // Time each vehicle joined the current edge queue (for wait accounting).
    std::unordered_map<int, double> veh_join_time_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    int    edge_key(int from, int to) const { return from * 100 + to; }

    // Index of edge (from→to) in adj[from].  -1 if not found.
    int    get_edge_index(int from, int to) const;

    // Is the signal at node `from` green for edge (from→to) at time `t`?
    bool   is_green(int from, int to, double t) const;

    // Earliest future time when signal at `from` will be green for (from→to).
    double next_green_time(int from, int to, double t) const;

    // BPR travel time for edge (from→to) using current active vehicle count.
    // flow_vph = active_count * (60 / free_flow_time)  [vehicles/hour]
    double bpr_time(int from, int to) const;

    // Sync Graph::Edge::flow with current active count so A* sees live congestion.
    void   update_graph_flow(int from, int to);

    // Convenience pointer to the const Edge in adj.  nullptr if not found.
    const Edge* get_edge(int from, int to) const;

    void push_event(double t, const EventData& e);
    void log_vehicle(int vid, double t, const std::string& msg);
    void record_edge_stats(int from, int to);

    // ── Event handlers ────────────────────────────────────────────────────────
    void on_arrives_at_edge(const EventData& e, double t);
    void on_enters_edge    (const EventData& e, double t);
    void on_exits_edge     (const EventData& e, double t);
    void on_signal_check   (const EventData& e, double t);
};
