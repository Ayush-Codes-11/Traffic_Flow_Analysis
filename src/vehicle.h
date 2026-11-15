// =============================================================================
// vehicle.h  –  Vehicle data, event types, and event payload
// Phase 3
// =============================================================================
#pragma once
#include <vector>

// =============================================================================
// Vehicle
// One vehicle in the simulation.  Tracks its route, timing, and wait.
// =============================================================================
struct Vehicle {
    int  id;
    int  src, dst;

    // route[0] = src, route.back() = dst, populated by A* at spawn time
    std::vector<int> route;
    std::vector<int> original_route; // Initial route at spawn time

    // route_pos: index in route of the node the vehicle is AT right now.
    // The edge being (or about to be) traversed is route[route_pos] → route[route_pos+1].
    int route_pos;

    double entry_time;    // time the vehicle entered the network (spawn time)
    double finish_time;   // time it reached dst  (-1.0 until completed)
    double total_wait;    // accumulated signal/queue wait time (minutes)
    bool   completed;
    bool   failed;        // true if route could not be found (unreachable dst)
    std::string failure_reason;

    bool   was_rerouted;  // True if route was dynamically updated mid-journey
    double reroute_time;  // Timestamp when rerouting was triggered
    int    reroute_node;  // Node ID where rerouting took place

    Vehicle()
        : id(-1), src(-1), dst(-1), route_pos(0),
          entry_time(0), finish_time(-1.0), total_wait(0.0), completed(false),
          failed(false), failure_reason(""),
          was_rerouted(false), reroute_time(-1.0), reroute_node(-1) {}
};

// =============================================================================
// EventType
// The four kinds of discrete events in the event-driven simulation loop.
// =============================================================================
enum class EventType {
    VEHICLE_ARRIVES_AT_EDGE,  // vehicle joins the queue at the upstream node
    VEHICLE_ENTERS_EDGE,      // vehicle dequeued + begins traversal
    VEHICLE_EXITS_EDGE,       // vehicle finishes traversal, reaches downstream node
    SIGNAL_CHECK,             // a red signal has cycled to green; wake waiting vehicles
    INCIDENT_START,           // an incident begins (closure or capacity reduction)
    INCIDENT_END,             // an incident ends (restoration of edge)
    SENSOR_READING,           // a periodic or external sensor reading arrives
    SIM_TIMESERIES_TICK       // 1-minute sampling interval for timeseries export
};

// =============================================================================
// EventData
// Payload stored alongside the event timestamp in the MinHeap.
// vehicle_id == -1 for SIGNAL_CHECK, INCIDENT_*, SENSOR_* events.
// =============================================================================
struct EventData {
    EventType type;
    int vehicle_id;   // -1 for non-vehicle events
    int from_node;
    int to_node;
    int incident_id;  // ID of incident for INCIDENT_START / INCIDENT_END
    double sensor_val;// flow reading or speed value for sensor events

    EventData()
        : type(EventType::SIGNAL_CHECK), vehicle_id(-1), from_node(-1),
          to_node(-1), incident_id(-1), sensor_val(0.0) {}
};
