// =============================================================================
// simulation.cpp  –  Event-driven traffic simulation
// Phase 3
//
// EVENT LOOP OVERVIEW (viva summary)
// ────────────────────────────────────────────────────────────────────────────
// Time is in minutes.  All future events live in a MinHeap ordered by time.
// Each iteration we pop the event with the SMALLEST time and handle it:
//
//  VEHICLE_ARRIVES_AT_EDGE
//    Vehicle joins the queue for edge (from→to).
//    If it is the ONLY vehicle in the queue it needs a trigger:
//      • Signal is green  → schedule VEHICLE_ENTERS_EDGE immediately.
//      • Signal is red    → schedule SIGNAL_CHECK at the next green time;
//                           accumulate the wait in Vehicle::total_wait.
//    If other vehicles are already queued, the existing cascade will
//    eventually reach this vehicle.
//
//  VEHICLE_ENTERS_EDGE
//    Pop the front vehicle from the queue; it starts traversing the edge.
//    Increment edge_active, compute BPR travel time with the new active count.
//    Schedule VEHICLE_EXITS_EDGE at current_time + travel_time.
//    If the queue still has vehicles and the signal is still green, schedule
//    the next VEHICLE_ENTERS_EDGE at current_time + headway (cascade).
//
//  VEHICLE_EXITS_EDGE
//    Decrement edge_active, update the graph's edge flow (so A* re-routes use
//    current congestion).  Advance the vehicle along its route.
//    If destination reached → mark completed.
//    Otherwise → schedule VEHICLE_ARRIVES_AT_EDGE for the next edge.
//
//  SIGNAL_CHECK
//    The signal at node `from` for edge (from→to) has cycled to green.
//    If the queue is non-empty → schedule VEHICLE_ENTERS_EDGE for the front.
//    If (by floating-point rounding) it is still red → reschedule.
// =============================================================================
#include "simulation.h"
#include "pathfinder.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

// ── Constructors ────────────────────────────────────────────────────────────
Simulation::Simulation(Graph& g)
    : g_(g), cfg_() {}

Simulation::Simulation(Graph& g, Config cfg)
    : g_(g), cfg_(cfg) {}

// ── add_vehicle ───────────────────────────────────────────────────────────────
// Route with A* at current congestion, then schedule the first edge arrival.
void Simulation::add_vehicle(int src, int dst, double spawn_time)
{
    Vehicle v;
    v.id         = static_cast<int>(vehicles_.size());
    v.src        = src;
    v.dst        = dst;
    v.entry_time = spawn_time;
    v.route_pos  = 0;

    // A* uses current Graph::Edge::flow values, updated by update_graph_flow().
    PathResult pr = astar(g_, src, dst);
    if (!pr.found || static_cast<int>(pr.path.size()) < 2) {
        // No reachable path (src==dst or disconnected); mark done immediately.
        v.completed   = true;
        v.finish_time = spawn_time;
        v.route       = {src};
        vehicles_.push_back(v);
        return;
    }
    v.route = pr.path;
    vehicles_.push_back(v);

    // Schedule arrival at the first edge's queue.
    EventData e;
    e.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
    e.vehicle_id = v.id;
    e.from_node  = src;
    e.to_node    = pr.path[1];
    push_event(spawn_time, e);
}

// ── run ───────────────────────────────────────────────────────────────────────
// Process events in chronological order until the heap is empty or max_time
// is exceeded.
void Simulation::run(double max_time)
{
    while (!event_heap_.empty()) {
        auto item  = event_heap_.pop();
        double t   = item.priority;
        const EventData& e = item.value;

        if (t > max_time) break;

        switch (e.type) {
            case EventType::VEHICLE_ARRIVES_AT_EDGE:
                on_arrives_at_edge(e, t); break;
            case EventType::VEHICLE_ENTERS_EDGE:
                on_enters_edge(e, t);     break;
            case EventType::VEHICLE_EXITS_EDGE:
                on_exits_edge(e, t);      break;
            case EventType::SIGNAL_CHECK:
                on_signal_check(e, t);    break;
        }
    }
}

// =============================================================================
// Event handlers
// =============================================================================

// ── on_arrives_at_edge ────────────────────────────────────────────────────────
void Simulation::on_arrives_at_edge(const EventData& e, double t)
{
    int vid  = e.vehicle_id;
    int from = e.from_node;
    int to   = e.to_node;
    int key  = edge_key(from, to);

    bool was_empty = queues_[key].isEmpty();
    queues_[key].enqueue(vid);
    veh_join_time_[vid] = t;   // record join time for wait accounting

    // Track peak queue length
    record_edge_stats(from, to);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "t=" << t
        << "  ARRIVE at queue for "
        << g_.nodes.at(from).name << "→" << g_.nodes.at(to).name
        << "  (q=" << queues_[key].size() << ")";
    log_vehicle(vid, t, oss.str());

    if (!was_empty) return;  // cascade from existing pipeline will reach this vehicle

    // This vehicle is the first in the queue – give it a trigger.
    if (is_green(from, to, t)) {
        // Green now: enter immediately
        EventData enter;
        enter.type       = EventType::VEHICLE_ENTERS_EDGE;
        enter.vehicle_id = vid;
        enter.from_node  = from;
        enter.to_node    = to;
        push_event(t, enter);
    } else {
        // Red: schedule a wakeup when it next turns green
        double green_t = next_green_time(from, to, t);
        EventData chk;
        chk.type       = EventType::SIGNAL_CHECK;
        chk.vehicle_id = -1;
        chk.from_node  = from;
        chk.to_node    = to;
        push_event(green_t, chk);

        std::ostringstream w;
        w << std::fixed << std::setprecision(2)
          << "t=" << t
          << "  RED signal  → waiting until t=" << green_t
          << " (" << (green_t - t) << " min)";
        log_vehicle(vid, t, w.str());
    }
}

// ── on_enters_edge ────────────────────────────────────────────────────────────
void Simulation::on_enters_edge(const EventData& e, double t)
{
    int from = e.from_node;
    int to   = e.to_node;
    int key  = edge_key(from, to);

    if (queues_[key].isEmpty()) return;   // defensive

    // Guard: signal might have flipped since this event was scheduled
    if (!is_green(from, to, t)) {
        double green_t = next_green_time(from, to, t);
        EventData chk;
        chk.type      = EventType::SIGNAL_CHECK;
        chk.vehicle_id= -1;
        chk.from_node = from;
        chk.to_node   = to;
        push_event(green_t, chk);
        return;
    }

    int vid = queues_[key].dequeue();     // remove from front

    // Account for time spent waiting in the queue.
    // FIX: original had operator-precedence bug: (t - veh_join_time_.count(vid)) ? ...
    // The ternary must guard on .count(vid), not on the subtraction result.
    double wait = veh_join_time_.count(vid) ? (t - veh_join_time_[vid]) : 0.0;
    vehicles_[vid].total_wait += wait;

    // Vehicle is now ON the edge
    edge_active_[key]++;
    update_graph_flow(from, to);         // inform A* of updated congestion

    double travel_t = bpr_time(from, to);  // BPR with current active count

    // Update edge stats
    EdgeStat& st = edge_stats_[key];
    st.from_node = from;
    st.to_node   = to;
    st.total_in++;
    if (edge_active_[key] > st.peak_active) st.peak_active = edge_active_[key];

    // Compute and record V/C ratio
    const Edge* ep = get_edge(from, to);
    if (ep && ep->capacity > 0) {
        double vc = (edge_active_[key] * 60.0 / ep->free_flow_time) / ep->capacity;
        if (vc > st.peak_vc) st.peak_vc = vc;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "t=" << t
        << "  ENTER  " << g_.nodes.at(from).name << "→" << g_.nodes.at(to).name
        << "  active=" << edge_active_[key]
        << "  bpr=" << travel_t << " min"
        << "  waited=" << wait << " min";
    log_vehicle(vid, t, oss.str());

    // Schedule exit
    EventData exit_e;
    exit_e.type       = EventType::VEHICLE_EXITS_EDGE;
    exit_e.vehicle_id = vid;
    exit_e.from_node  = from;
    exit_e.to_node    = to;
    push_event(t + travel_t, exit_e);

    // Cascade: if more vehicles in queue and signal still green, let the next
    // one enter after a small headway gap.
    if (!queues_[key].isEmpty() && is_green(from, to, t + cfg_.headway)) {
        EventData nxt;
        nxt.type       = EventType::VEHICLE_ENTERS_EDGE;
        nxt.vehicle_id = -1;   // actual id dequeued when this fires
        nxt.from_node  = from;
        nxt.to_node    = to;
        push_event(t + cfg_.headway, nxt);
    }
}

// ── on_exits_edge ─────────────────────────────────────────────────────────────
void Simulation::on_exits_edge(const EventData& e, double t)
{
    int vid  = e.vehicle_id;
    int from = e.from_node;
    int to   = e.to_node;
    int key  = edge_key(from, to);

    // Vehicle leaves the edge
    edge_active_[key] = std::max(0, edge_active_[key] - 1);
    update_graph_flow(from, to);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "t=" << t
        << "  EXIT   " << g_.nodes.at(from).name << "→" << g_.nodes.at(to).name;
    log_vehicle(vid, t, oss.str());

    Vehicle& v = vehicles_[vid];
    v.route_pos++;  // now AT node `to`

    // Destination reached?
    if (to == v.dst || v.route_pos >= static_cast<int>(v.route.size()) - 1) {
        v.completed   = true;
        v.finish_time = t;
        std::ostringstream done;
        done << std::fixed << std::setprecision(2)
             << "t=" << t
             << "  DONE! total=" << (t - v.entry_time)
             << " min  wait=" << v.total_wait << " min";
        log_vehicle(vid, t, done.str());
        return;
    }

    // Schedule arrival at next edge
    int next_to = v.route[v.route_pos + 1];
    EventData next_e;
    next_e.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
    next_e.vehicle_id = vid;
    next_e.from_node  = to;
    next_e.to_node    = next_to;
    push_event(t, next_e);
}

// ── on_signal_check ───────────────────────────────────────────────────────────
void Simulation::on_signal_check(const EventData& e, double t)
{
    int from = e.from_node;
    int to   = e.to_node;
    int key  = edge_key(from, to);

    if (queues_[key].isEmpty()) return;   // nobody waiting

    if (is_green(from, to, t)) {
        // Green: kick off the front vehicle
        EventData enter;
        enter.type       = EventType::VEHICLE_ENTERS_EDGE;
        enter.vehicle_id = -1;
        enter.from_node  = from;
        enter.to_node    = to;
        push_event(t, enter);
    } else {
        // Still red (floating-point edge): reschedule
        double green_t = next_green_time(from, to, t);
        EventData chk = e;
        push_event(green_t, chk);
    }
}

// =============================================================================
// Helpers
// =============================================================================

int Simulation::get_edge_index(int from, int to) const
{
    auto it = g_.adj.find(from);
    if (it == g_.adj.end()) return -1;
    const auto& edges = it->second;
    for (int i = 0; i < static_cast<int>(edges.size()); ++i)
        if (edges[i].to == to) return i;
    return -1;
}

// is_green: phase at node `from` for edge (from→to).
// Phase = floor(t / green_duration) % n_outgoing_edges.
// A single-outgoing-edge intersection is always green (no competing traffic).
bool Simulation::is_green(int from, int to, double t) const
{
    if (cfg_.green_duration <= 0.0) return true;   // signals disabled
    int idx = get_edge_index(from, to);
    if (idx < 0) return true;
    int n = static_cast<int>(g_.adj.at(from).size());
    if (n <= 1) return true;                        // only one road out
    int phase = static_cast<int>(t / cfg_.green_duration) % n;
    return phase == idx;
}

// next_green_time: when does the signal at `from` next go green for (from→to)?
// Uses: cur_phase_start + phases_ahead * green_duration
double Simulation::next_green_time(int from, int to, double t) const
{
    if (cfg_.green_duration <= 0.0) return t;
    if (is_green(from, to, t)) return t;
    int idx = get_edge_index(from, to);
    if (idx < 0) return t;
    int n = static_cast<int>(g_.adj.at(from).size());
    int cur_phase    = static_cast<int>(t / cfg_.green_duration) % n;
    int phases_ahead = (idx - cur_phase + n) % n;
    if (phases_ahead == 0) return t;
    double phase_start = std::floor(t / cfg_.green_duration) * cfg_.green_duration;
    return phase_start + phases_ahead * cfg_.green_duration;
}

// bpr_time: BPR travel-time using the number of vehicles currently ON the edge.
// flow_vph = active * (60 / free_flow_time)  — vehicles per hour approximation.
// When k vehicles occupy an edge whose free-flow transit takes t0 minutes,
// they represent a throughput of k/t0 vehicles per minute = k*60/t0 vph.
double Simulation::bpr_time(int from, int to) const
{
    const Edge* ep = get_edge(from, to);
    if (!ep) return 1.0;
    int    active   = edge_active_.count(edge_key(from, to))
                      ? edge_active_.at(edge_key(from, to)) : 0;
    double flow_vph = (ep->free_flow_time > 0)
                      ? active * 60.0 / ep->free_flow_time : 0.0;
    double ratio    = flow_vph / ep->capacity;
    return ep->free_flow_time * (1.0 + 0.15 * std::pow(ratio, 4.0));
}

void Simulation::update_graph_flow(int from, int to)
{
    auto it = g_.adj.find(from);
    if (it == g_.adj.end()) return;
    for (Edge& ed : it->second) {
        if (ed.to == to) {
            int active = edge_active_.count(edge_key(from, to))
                         ? edge_active_.at(edge_key(from, to)) : 0;
            ed.flow = (ed.free_flow_time > 0)
                      ? active * 60.0 / ed.free_flow_time : 0.0;
            return;
        }
    }
}

const Edge* Simulation::get_edge(int from, int to) const
{
    auto it = g_.adj.find(from);
    if (it == g_.adj.end()) return nullptr;
    for (const Edge& e : it->second)
        if (e.to == to) return &e;
    return nullptr;
}

void Simulation::push_event(double t, const EventData& e)
{
    event_heap_.push(t, e);
}

void Simulation::log_vehicle(int vid, double /*t*/, const std::string& msg)
{
    if (vid == cfg_.log_vehicle_id)
        timelines_[vid].push_back(msg);
}

void Simulation::record_edge_stats(int from, int to)
{
    int key      = edge_key(from, to);
    EdgeStat& st = edge_stats_[key];
    st.from_node = from;
    st.to_node   = to;
    int q = queues_[key].size();
    if (q > st.peak_queue) st.peak_queue = q;
}

// =============================================================================
// Output
// =============================================================================

void Simulation::print_summary() const
{
    int    n_done   = 0;
    double sum_time = 0.0;
    double sum_wait = 0.0;

    for (const Vehicle& v : vehicles_) {
        if (!v.completed || v.finish_time < 0.0) continue;
        ++n_done;
        sum_time += (v.finish_time - v.entry_time);
        sum_wait += v.total_wait;
    }

    std::cout << "\n── Simulation Summary ──────────────────────────────────\n";
    std::cout << "  Vehicles spawned   : " << vehicles_.size() << "\n";
    std::cout << "  Vehicles completed : " << n_done << "\n";
    if (n_done > 0) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Avg travel time    : " << sum_time / n_done << " min\n";
        std::cout << "  Avg signal wait    : " << sum_wait / n_done << " min\n";
    }

    // Top 5 edges by total vehicles entered
    using KS = std::pair<int, EdgeStat>;
    std::vector<KS> sorted(edge_stats_.begin(), edge_stats_.end());
    std::sort(sorted.begin(), sorted.end(),
              [](const KS& a, const KS& b){
                  return a.second.total_in > b.second.total_in; });

    std::cout << "\n  Top 5 busiest edges (by vehicles entered):\n";
    std::cout << "  " << std::left
              << std::setw(22) << "From"
              << std::setw(22) << "To"
              << std::setw(10) << "Total"
              << std::setw(10) << "PeakActv"
              << std::setw(10) << "PeakQ"
              << "PeakV/C\n";
    std::cout << "  " << std::string(78, '-') << "\n";

    int shown = 0;
    for (const auto& kv : sorted) {
        if (shown >= 5) break;
        const EdgeStat& st = kv.second;
        if (st.from_node < 0 || !g_.nodes.count(st.from_node)) continue;
        std::cout << "  " << std::left
                  << std::setw(22) << g_.nodes.at(st.from_node).name
                  << std::setw(22) << g_.nodes.at(st.to_node).name
                  << std::setw(10) << st.total_in
                  << std::setw(10) << st.peak_active
                  << std::setw(10) << st.peak_queue
                  << std::fixed << std::setprecision(3) << st.peak_vc << "\n";
        ++shown;
    }
    std::cout << "────────────────────────────────────────────────────────\n";
}

void Simulation::print_vehicle_timeline(int vid) const
{
    auto it = timelines_.find(vid);
    if (it == timelines_.end() || it->second.empty()) {
        std::cout << "  [No timeline recorded for vehicle " << vid << "]\n";
        return;
    }
    std::cout << "\n  Timeline for Vehicle " << vid;
    if (vid < static_cast<int>(vehicles_.size())) {
        const Vehicle& v = vehicles_[vid];
        if (!v.route.empty() && g_.nodes.count(v.src) && g_.nodes.count(v.dst))
            std::cout << " (" << g_.nodes.at(v.src).name
                      << " → " << g_.nodes.at(v.dst).name << ")";
    }
    std::cout << ":\n";
    for (const std::string& line : it->second)
        std::cout << "    " << line << "\n";
}
