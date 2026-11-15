// =============================================================================
// incidents.h  –  Incident data model and management
// Phase 5
// =============================================================================
#pragma once

#include "graph.h"
#include "alerts.h"

#include <string>
#include <vector>
#include <unordered_map>

enum class IncidentType {
    CLOSURE,
    CAPACITY_REDUCTION
};

inline std::string incident_type_to_string(IncidentType t) {
    switch (t) {
        case IncidentType::CLOSURE:            return "CLOSURE";
        case IncidentType::CAPACITY_REDUCTION: return "CAPACITY_REDUCTION";
    }
    return "UNKNOWN";
}

struct Incident {
    int id;
    int from_node;
    int to_node;
    double start_time;
    double end_time;
    IncidentType type;
    double factor;  // e.g. 0.5 for 50% capacity reduction; unused for CLOSURE
    std::string description;

    Incident()
        : id(-1), from_node(-1), to_node(-1), start_time(0.0), end_time(0.0),
          type(IncidentType::CLOSURE), factor(0.5), description("") {}

    Incident(int i, int f, int t, double st, double et, IncidentType itype, double fac = 0.5, const std::string& desc = "")
        : id(i), from_node(f), to_node(t), start_time(st), end_time(et),
          type(itype), factor(fac), description(desc) {}
};

class IncidentManager {
public:
    IncidentManager() = default;

    // Apply incident start on the graph and alert manager.
    // Preserves original capacity / state to prevent corruption from overlapping incidents.
    bool apply_incident_start(const Incident& inc, Graph& g, AlertManager& alerts);

    // Apply incident end on the graph and alert manager.
    // Restores original capacity / closure status when active incidents resolve.
    bool apply_incident_end(const Incident& inc, Graph& g, AlertManager& alerts);

    // Check if an edge currently has any active closure incident
    bool is_edge_closed(int from, int to) const;

    // Reset all incident states
    void clear();

private:
    int edge_key(int from, int to) const { return from * 1000 + to; }

    // Edge key -> original capacity (recorded before any capacity reduction incident)
    std::unordered_map<int, double> original_capacities_;

    // Edge key -> count of active closure incidents
    std::unordered_map<int, int> active_closures_;

    // Edge key -> list of active incidents currently applied
    std::unordered_map<int, std::vector<Incident>> active_incidents_;
};
