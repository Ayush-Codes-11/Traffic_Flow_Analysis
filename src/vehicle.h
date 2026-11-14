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

    // route_pos: index in route of the node the vehicle is AT right now.
    // The edge being (or about to be) traversed is route[route_pos] → route[route_pos+1].
    int route_pos;

    double entry_time;    // time the vehicle entered the network (spawn time)
    double finish_time;   // time it reached dst  (-1.0 until completed)
    double total_wait;    // accumulated signal/queue wait time (minutes)
    bool   completed;

    Vehicle()
        : id(-1), src(-1), dst(-1), route_pos(0),
          entry_time(0), finish_time(-1.0), total_wait(0.0), completed(false) {}
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
};

// =============================================================================
// EventData
// Payload stored alongside the event timestamp in the MinHeap.
// vehicle_id == -1 for SIGNAL_CHECK events (they don't belong to one vehicle).
// =============================================================================
struct EventData {
    EventType type;
    int vehicle_id;   // -1 for SIGNAL_CHECK / cascade ENTERS events
    int from_node;
    int to_node;
};
