// =============================================================================
// simulation.cpp  –  Event-driven traffic simulation
// Phase 3 + Phase 5 (Incidents, Dynamic Rerouting, Sensor Ingestion, Alerts)
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
void Simulation::add_vehicle(int src, int dst, double spawn_time)
{
    Vehicle v;
    v.id         = static_cast<int>(vehicles_.size());
    v.src        = src;
    v.dst        = dst;
    v.entry_time = spawn_time;
    v.route_pos  = 0;

    if (src == dst) {
        v.completed   = true;
        v.finish_time = spawn_time;
        v.route       = {src};
        vehicles_.push_back(v);
        return;
    }

    // A* uses current Graph::Edge::flow values, updated by update_graph_flow().
    PathResult pr = astar(g_, src, dst);
    if (!pr.found || static_cast<int>(pr.path.size()) < 2) {
        // Destination is currently unreachable
        v.completed      = false;
        v.failed         = true;
        v.finish_time    = spawn_time;
        v.failure_reason = "No path found at spawn";
        vehicles_.push_back(v);
        return;
    }
    v.route = pr.path;
    v.original_route = pr.path;
    vehicles_.push_back(v);

    // Schedule arrival at the first edge's queue.
    EventData e;
    e.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
    e.vehicle_id = v.id;
    e.from_node  = src;
    e.to_node    = pr.path[1];
    push_event(spawn_time, e);
}

// ── add_incident ──────────────────────────────────────────────────────────────
void Simulation::add_incident(const Incident& inc)
{
    int id = static_cast<int>(scheduled_incidents_.size());
    Incident copy = inc;
    copy.id = id;
    scheduled_incidents_.push_back(copy);

    // Schedule incident start
    EventData start_e;
    start_e.type        = EventType::INCIDENT_START;
    start_e.incident_id = id;
    start_e.from_node   = inc.from_node;
    start_e.to_node     = inc.to_node;
    push_event(inc.start_time, start_e);

    // Schedule incident end
    EventData end_e;
    end_e.type        = EventType::INCIDENT_END;
    end_e.incident_id = id;
    end_e.from_node   = inc.from_node;
    end_e.to_node     = inc.to_node;
    push_event(inc.end_time, end_e);
}

// ── add_sensor_reading ────────────────────────────────────────────────────────
void Simulation::add_sensor_reading(const SensorReading& r)
{
    sensor_feed_.add_reading(r);
    if (cfg_.enable_sensor_feed) {
        update_graph_flow(r.from_node, r.to_node);
    }
}

// ── load_sensor_csv ───────────────────────────────────────────────────────────
int Simulation::load_sensor_csv(const std::string& filepath)
{
    int count = sensor_feed_.load_from_csv(filepath);
    if (cfg_.enable_sensor_feed) {
        for (const auto& r : sensor_feed_.get_all_readings()) {
            update_graph_flow(r.from_node, r.to_node);
        }
    }
    return count;
}

// ── run ───────────────────────────────────────────────────────────────────────
void Simulation::run(double max_time)
{
    // Schedule periodic timeseries ticks every 1.0 minute
    EventData tick;
    tick.type = EventType::SIM_TIMESERIES_TICK;
    push_event(1.0, tick);

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
            case EventType::INCIDENT_START:
                on_incident_start(e, t);  break;
            case EventType::INCIDENT_END:
                on_incident_end(e, t);    break;
            case EventType::SENSOR_READING:
                on_sensor_reading(e, t);  break;
            case EventType::SIM_TIMESERIES_TICK:
                on_timeseries_tick(t);    break;
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

    // If edge is closed and rerouting is enabled, reroute immediately
    const Edge* ep = get_edge(from, to);
    if (ep && ep->closed && cfg_.enable_rerouting) {
        Vehicle& v = vehicles_[vid];
        PathResult pr = astar(g_, from, v.dst);
        if (pr.found && pr.path.size() >= 2) {
            std::vector<int> new_route(v.route.begin(), v.route.begin() + v.route_pos);
            new_route.insert(new_route.end(), pr.path.begin(), pr.path.end());
            v.route = new_route;
            if (!v.was_rerouted) {
                v.was_rerouted = true;
                v.reroute_time = t;
                v.reroute_node = from;
            }
            int new_to = pr.path[1];

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "t=" << t << "  REROUTE (at queue arrival: "
                << from << "->" << to << " closed) -> detour via " << new_to;
            log_vehicle(vid, t, oss.str());

            EventData arr;
            arr.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
            arr.vehicle_id = vid;
            arr.from_node  = from;
            arr.to_node    = new_to;
            push_event(t, arr);
            return;
        } else {
            v.failed = true;
            v.completed = false;
            v.finish_time = t;
            v.failure_reason = "Destination unreachable on edge arrival";
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "t=" << t << "  FAILED: destination " << v.dst
                << " unreachable from " << from;
            log_vehicle(vid, t, oss.str());
            return;
        }
    }

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

    // If edge is closed without rerouting, vehicle waits in queue for reopen
    if (ep && ep->closed) {
        return;
    }

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

    const Edge* ep = get_edge(from, to);

    // Road is closed check
    if (ep && ep->closed) {
        if (cfg_.enable_rerouting && !queues_[key].isEmpty()) {
            int vid = queues_[key].dequeue();
            Vehicle& v = vehicles_[vid];
            double wait = veh_join_time_.count(vid) ? (t - veh_join_time_[vid]) : 0.0;
            v.total_wait += wait;

            PathResult pr = astar(g_, from, v.dst);
            if (pr.found && pr.path.size() >= 2) {
                std::vector<int> new_route(v.route.begin(), v.route.begin() + v.route_pos);
                new_route.insert(new_route.end(), pr.path.begin(), pr.path.end());
                v.route = new_route;

                EventData arr;
                arr.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
                arr.vehicle_id = vid;
                arr.from_node  = from;
                arr.to_node    = pr.path[1];
                push_event(t, arr);
            } else {
                v.failed = true;
                v.completed = false;
                v.finish_time = t;
                v.failure_reason = "Destination unreachable on closed edge dequeue";
            }
        } else {
            // Rerouting off: wait for reopen
            EventData chk;
            chk.type       = EventType::SIGNAL_CHECK;
            chk.vehicle_id = -1;
            chk.from_node  = from;
            chk.to_node    = to;
            push_event(t + 1.0, chk);
        }
        return;
    }

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

    double wait = veh_join_time_.count(vid) ? (t - veh_join_time_[vid]) : 0.0;
    vehicles_[vid].total_wait += wait;

    // Vehicle is now ON the edge
    edge_active_[key]++;
    edge_entries_[key].push_back({vid, t}); // record entry timestamp

    update_graph_flow(from, to);            // inform A* of updated congestion

    double travel_t = bpr_time(from, to);  // BPR with current active count

    // Update edge stats
    EdgeStat& st = edge_stats_[key];
    st.from_node = from;
    st.to_node   = to;
    st.total_in++;
    if (edge_active_[key] > st.peak_active) st.peak_active = edge_active_[key];

    // Compute and record V/C ratio
    if (ep && ep->capacity > 0) {
        double vc = (edge_active_[key] * 60.0 / ep->free_flow_time) / ep->capacity;
        if (vc > st.peak_vc) st.peak_vc = vc;
        alerts_.check_vc_congestion(t, from, to, vc);
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

    // Cascade: if more vehicles in queue, handle them
    if (!queues_[key].isEmpty()) {
        if (is_green(from, to, t + cfg_.headway)) {
            EventData nxt;
            nxt.type       = EventType::VEHICLE_ENTERS_EDGE;
            nxt.vehicle_id = -1;
            nxt.from_node  = from;
            nxt.to_node    = to;
            push_event(t + cfg_.headway, nxt);
        } else {
            double green_t = next_green_time(from, to, t + cfg_.headway);
            EventData chk;
            chk.type       = EventType::SIGNAL_CHECK;
            chk.vehicle_id = -1;
            chk.from_node  = from;
            chk.to_node    = to;
            push_event(green_t, chk);
        }
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

    int next_to = v.route[v.route_pos + 1];

    // Dynamic Rerouting Check
    if (cfg_.enable_rerouting) {
        const Edge* next_ep = get_edge(to, next_to);
        bool next_is_closed = (next_ep && next_ep->closed);

        if (next_is_closed) {
            // Next edge is closed: must reroute
            PathResult pr = astar(g_, to, v.dst);
            if (pr.found && pr.path.size() >= 2) {
                std::vector<int> new_route(v.route.begin(), v.route.begin() + v.route_pos);
                new_route.insert(new_route.end(), pr.path.begin(), pr.path.end());
                v.route = new_route;
                if (!v.was_rerouted) {
                    v.was_rerouted = true;
                    v.reroute_time = t;
                    v.reroute_node = to;
                }
                next_to = v.route[v.route_pos + 1];

                std::ostringstream r_log;
                r_log << std::fixed << std::setprecision(2)
                      << "t=" << t << "  REROUTE (next edge " << to << "->"
                      << next_ep->to << " closed): detour via " << next_to;
                log_vehicle(vid, t, r_log.str());
            } else {
                v.failed = true;
                v.completed = false;
                v.finish_time = t;
                v.failure_reason = "Destination unreachable due to closure";
                std::ostringstream f_log;
                f_log << std::fixed << std::setprecision(2)
                      << "t=" << t << "  FAILED: destination " << v.dst
                      << " unreachable from node " << to;
                log_vehicle(vid, t, f_log.str());
                return;
            }
        } else if (cfg_.reroute_threshold > 0.0) {
            // Proactive congestion detour check
            double current_rem_time = 0.0;
            for (size_t i = v.route_pos; i + 1 < v.route.size(); ++i) {
                double b = g_.get_bpr_time(v.route[i], v.route[i + 1]);
                if (b < 0) b = 1e9;
                current_rem_time += b;
            }

            PathResult pr = astar(g_, to, v.dst);
            if (pr.found && pr.path.size() >= 2 &&
                pr.total_time < current_rem_time * (1.0 - cfg_.reroute_threshold)) {
                std::vector<int> new_route(v.route.begin(), v.route.begin() + v.route_pos);
                new_route.insert(new_route.end(), pr.path.begin(), pr.path.end());
                v.route = new_route;
                if (!v.was_rerouted) {
                    v.was_rerouted = true;
                    v.reroute_time = t;
                    v.reroute_node = to;
                }
                next_to = v.route[v.route_pos + 1];

                std::ostringstream p_log;
                p_log << std::fixed << std::setprecision(2)
                      << "t=" << t << "  REROUTE (congestion speedup): saved "
                      << (current_rem_time - pr.total_time) << " min";
                log_vehicle(vid, t, p_log.str());
            }
        }
    }

    // Schedule arrival at next edge
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

    const Edge* ep = get_edge(from, to);
    if (ep && ep->closed) {
        if (!cfg_.enable_rerouting) {
            // Rerouting off: wait for reopen
            EventData chk = e;
            push_event(t + 1.0, chk);
        }
        return;
    }

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

// ── on_incident_start ─────────────────────────────────────────────────────────
void Simulation::on_incident_start(const EventData& e, double t)
{
    int inc_id = e.incident_id;
    if (inc_id < 0 || inc_id >= static_cast<int>(scheduled_incidents_.size())) return;

    const Incident& inc = scheduled_incidents_[inc_id];
    incidents_.apply_incident_start(inc, g_, alerts_);

    if (inc.type == IncidentType::CLOSURE && cfg_.enable_rerouting) {
        int key = edge_key(inc.from_node, inc.to_node);
        std::vector<int> waiting;
        while (!queues_[key].isEmpty()) {
            waiting.push_back(queues_[key].dequeue());
        }

        for (int vid : waiting) {
            Vehicle& v = vehicles_[vid];
            double wait = veh_join_time_.count(vid) ? (t - veh_join_time_[vid]) : 0.0;
            v.total_wait += wait;

            PathResult pr = astar(g_, inc.from_node, v.dst);
            if (pr.found && pr.path.size() >= 2) {
                std::vector<int> new_route(v.route.begin(), v.route.begin() + v.route_pos);
                new_route.insert(new_route.end(), pr.path.begin(), pr.path.end());
                v.route = new_route;
                if (!v.was_rerouted) {
                    v.was_rerouted = true;
                    v.reroute_time = t;
                    v.reroute_node = inc.from_node;
                }

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "t=" << t << "  REROUTE (queue evacuated due to closure of "
                    << inc.from_node << "->" << inc.to_node << "): new next is " << pr.path[1];
                log_vehicle(vid, t, oss.str());

                EventData arr;
                arr.type       = EventType::VEHICLE_ARRIVES_AT_EDGE;
                arr.vehicle_id = vid;
                arr.from_node  = inc.from_node;
                arr.to_node    = pr.path[1];
                push_event(t, arr);
            } else {
                v.failed         = true;
                v.completed      = false;
                v.finish_time    = t;
                v.failure_reason = "Unreachable destination on closure evacuation";

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "t=" << t << "  FAILED: destination " << v.dst
                    << " unreachable from node " << inc.from_node;
                log_vehicle(vid, t, oss.str());
            }
        }
    }
}

// ── on_incident_end ───────────────────────────────────────────────────────────
void Simulation::on_incident_end(const EventData& e, double t)
{
    int inc_id = e.incident_id;
    if (inc_id < 0 || inc_id >= static_cast<int>(scheduled_incidents_.size())) return;

    const Incident& inc = scheduled_incidents_[inc_id];
    incidents_.apply_incident_end(inc, g_, alerts_);

    int key = edge_key(inc.from_node, inc.to_node);
    if (!queues_[key].isEmpty()) {
        if (is_green(inc.from_node, inc.to_node, t)) {
            EventData enter;
            enter.type       = EventType::VEHICLE_ENTERS_EDGE;
            enter.vehicle_id = -1;
            enter.from_node  = inc.from_node;
            enter.to_node    = inc.to_node;
            push_event(t, enter);
        } else {
            double green_t = next_green_time(inc.from_node, inc.to_node, t);
            EventData chk;
            chk.type       = EventType::SIGNAL_CHECK;
            chk.vehicle_id = -1;
            chk.from_node  = inc.from_node;
            chk.to_node    = inc.to_node;
            push_event(green_t, chk);
        }
    }
}

// ── on_sensor_reading ─────────────────────────────────────────────────────────
void Simulation::on_sensor_reading(const EventData& e, double t)
{
    SensorReading r(t, e.from_node, e.to_node, e.sensor_val, 45.0);
    sensor_feed_.add_reading(r);
    if (cfg_.enable_sensor_feed) {
        update_graph_flow(e.from_node, e.to_node);
    }
}

// ── on_timeseries_tick ────────────────────────────────────────────────────────
void Simulation::on_timeseries_tick(double t)
{
    for (const auto& [u, edges] : g_.adj) {
        for (const auto& e : edges) {
            int key = edge_key(u, e.to);
            int q_len = queues_.count(key) ? queues_.at(key).size() : 0;
            int act   = edge_active_.count(key) ? edge_active_.at(key) : 0;
            double sim_f = (e.free_flow_time > 0.0) ? (act * 60.0 / e.free_flow_time) : 0.0;
            double flow = sim_f;
            if (cfg_.enable_sensor_feed) {
                flow = sensor_feed_.get_blended_flow(u, e.to, sim_f, t, &alerts_);
            }
            double vc = (e.capacity > 0.0) ? (flow / e.capacity) : 0.0;
            alerts_.record_timeseries_sample(t, u, e.to, flow, q_len, vc);
        }
    }

    // Schedule next tick in 1.0 minute
    EventData next_tick;
    next_tick.type = EventType::SIM_TIMESERIES_TICK;
    push_event(t + 1.0, next_tick);
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

double Simulation::bpr_time(int from, int to) const
{
    const Edge* ep = get_edge(from, to);
    if (!ep) return 1.0;
    if (ep->closed) return 1e18;
    int    active   = edge_active_.count(edge_key(from, to))
                      ? edge_active_.at(edge_key(from, to)) : 0;
    double flow_vph = (ep->free_flow_time > 0)
                      ? active * 60.0 / ep->free_flow_time : 0.0;
    if (cfg_.enable_sensor_feed) {
        flow_vph = const_cast<SensorFeed&>(sensor_feed_).get_blended_flow(from, to, flow_vph, 0.0, nullptr);
    }
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
            double sim_vph = (ed.free_flow_time > 0)
                             ? active * 60.0 / ed.free_flow_time : 0.0;
            if (cfg_.enable_sensor_feed) {
                ed.flow = sensor_feed_.get_blended_flow(from, to, sim_vph, 0.0, nullptr);
            } else {
                ed.flow = sim_vph;
            }
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

const std::vector<std::pair<int, double>>& Simulation::get_edge_entries(int from, int to) const
{
    static const std::vector<std::pair<int, double>> empty_vec;
    int key = edge_key(from, to);
    auto it = edge_entries_.find(key);
    if (it != edge_entries_.end()) return it->second;
    return empty_vec;
}

Simulation::RunMetrics Simulation::get_metrics() const
{
    RunMetrics m;
    double sum_time = 0.0;
    for (const auto& v : vehicles_) {
        if (v.completed && v.finish_time >= 0.0) {
            m.completed++;
            double tt = v.finish_time - v.entry_time;
            sum_time += tt;
            if (tt > m.max_travel_time) m.max_travel_time = tt;
        } else if (v.failed) {
            m.failed++;
        }
        m.total_wait += v.total_wait;
    }
    if (m.completed > 0) {
        m.avg_travel_time = sum_time / m.completed;
    }
    return m;
}

// =============================================================================
// Output
// =============================================================================

void Simulation::print_summary() const
{
    RunMetrics m = get_metrics();

    std::cout << "\n── Simulation Summary ──────────────────────────────────\n";
    std::cout << "  Vehicles spawned   : " << vehicles_.size() << "\n";
    std::cout << "  Vehicles completed : " << m.completed << "\n";
    std::cout << "  Vehicles failed    : " << m.failed << "\n";
    if (m.completed > 0) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Avg travel time    : " << m.avg_travel_time << " min\n";
        std::cout << "  Max travel time    : " << m.max_travel_time << " min\n";
        std::cout << "  Avg signal wait    : " << (m.total_wait / m.completed) << " min\n";
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
