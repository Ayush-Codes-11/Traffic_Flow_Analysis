// =============================================================================
// incidents.cpp  –  Implementation of incident management
// Phase 5
// =============================================================================
#include "incidents.h"

#include <iostream>
#include <algorithm>

bool IncidentManager::apply_incident_start(const Incident& inc, Graph& g, AlertManager& alerts)
{
    int key = edge_key(inc.from_node, inc.to_node);

    // Save baseline capacity on first incident for this edge
    if (original_capacities_.find(key) == original_capacities_.end()) {
        double cap = g.get_capacity(inc.from_node, inc.to_node);
        if (cap > 0.0) {
            original_capacities_[key] = cap;
        }
    }

    active_incidents_[key].push_back(inc);

    if (inc.type == IncidentType::CLOSURE) {
        active_closures_[key]++;
        g.set_closed(inc.from_node, inc.to_node, true);

        std::string desc = inc.description.empty() ? "Road closure" : inc.description;
        alerts.add_closure_alert(inc.start_time, inc.from_node, inc.to_node,
                                 desc + " (closed at t=" + std::to_string(inc.start_time) + ")");
    } else if (inc.type == IncidentType::CAPACITY_REDUCTION) {
        double orig_cap = original_capacities_[key];
        double new_cap  = orig_cap * inc.factor;
        g.set_capacity(inc.from_node, inc.to_node, new_cap);

        std::string desc = inc.description.empty() ? "Capacity reduced" : inc.description;
        alerts.add_closure_alert(inc.start_time, inc.from_node, inc.to_node,
                                 desc + " (factor=" + std::to_string(inc.factor) + ")");
    }

    return true;
}

bool IncidentManager::apply_incident_end(const Incident& inc, Graph& g, AlertManager& alerts)
{
    int key = edge_key(inc.from_node, inc.to_node);

    // Remove from active incident list
    auto& list = active_incidents_[key];
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](const Incident& item) { return item.id == inc.id; }),
               list.end());

    if (inc.type == IncidentType::CLOSURE) {
        if (active_closures_.count(key) && active_closures_[key] > 0) {
            active_closures_[key]--;
        }
        // Only reopen edge if no other closure incident is active on this edge
        if (active_closures_[key] == 0) {
            g.set_closed(inc.from_node, inc.to_node, false);
            alerts.add_closure_alert(inc.end_time, inc.from_node, inc.to_node,
                                     "Road reopened at t=" + std::to_string(inc.end_time));
        }
    } else if (inc.type == IncidentType::CAPACITY_REDUCTION) {
        // Find if another capacity reduction incident is active
        double effective_factor = 1.0;
        bool has_other = false;
        for (const auto& item : list) {
            if (item.type == IncidentType::CAPACITY_REDUCTION) {
                effective_factor = std::min(effective_factor, item.factor);
                has_other = true;
            }
        }

        double orig_cap = original_capacities_[key];
        if (has_other) {
            g.set_capacity(inc.from_node, inc.to_node, orig_cap * effective_factor);
        } else {
            g.set_capacity(inc.from_node, inc.to_node, orig_cap);
        }
    }

    return true;
}

bool IncidentManager::is_edge_closed(int from, int to) const
{
    int key = edge_key(from, to);
    auto it = active_closures_.find(key);
    return (it != active_closures_.end() && it->second > 0);
}

void IncidentManager::clear()
{
    original_capacities_.clear();
    active_closures_.clear();
    active_incidents_.clear();
}
