// =============================================================================
// alerts.cpp  –  Implementation of traffic alert system and time series export
// Phase 5
// =============================================================================
#include "alerts.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

bool AlertManager::check_vc_congestion(double t, int from, int to, double vc)
{
    int key = edge_key(from, to);
    bool is_congested = congested_edges_[key];

    // Threshold 0.90 to enter congested state
    if (vc > 0.90 && !is_congested) {
        congested_edges_[key] = true;
        Alert a;
        a.time = t;
        a.from_node = from;
        a.to_node = to;
        a.type = AlertType::CONGESTION;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "Congestion detected on edge " << from << "->" << to
            << " (V/C = " << vc << " > 0.90)";
        a.message = oss.str();
        alerts_.push_back(a);
        return true;
    }

    // Threshold 0.80 to clear congested state (hysteresis)
    if (vc < 0.80 && is_congested) {
        congested_edges_[key] = false;
    }

    return false;
}

void AlertManager::add_closure_alert(double t, int from, int to, const std::string& msg)
{
    Alert a;
    a.time = t;
    a.from_node = from;
    a.to_node = to;
    a.type = AlertType::CLOSURE;
    a.message = msg;
    alerts_.push_back(a);
}

void AlertManager::add_dropout_alert(double t, int from, int to, const std::string& msg)
{
    int key = edge_key(from, to);
    if (!dropout_edges_[key]) {
        dropout_edges_[key] = true;
        Alert a;
        a.time = t;
        a.from_node = from;
        a.to_node = to;
        a.type = AlertType::SENSOR_DROPOUT;
        a.message = msg;
        alerts_.push_back(a);
    }
}

void AlertManager::add_alert(const Alert& a)
{
    alerts_.push_back(a);
}

void AlertManager::record_timeseries_sample(double t, int from, int to, double flow, int q_len, double vc)
{
    TimeSeriesPoint p;
    p.time = t;
    p.from_node = from;
    p.to_node = to;
    p.flow = flow;
    p.queue_length = q_len;
    p.vc_ratio = vc;
    timeseries_.push_back(p);
}

bool AlertManager::save_alerts_to_json(const std::string& filepath) const
{
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "[AlertManager] Error: Unable to open " << filepath << " for writing\n";
        return false;
    }

    out << "{\n  \"alerts\": [\n";
    for (size_t i = 0; i < alerts_.size(); ++i) {
        const auto& a = alerts_[i];
        out << "    {\n"
            << "      \"time\": " << std::fixed << std::setprecision(2) << a.time << ",\n"
            << "      \"from\": " << a.from_node << ",\n"
            << "      \"to\": " << a.to_node << ",\n"
            << "      \"type\": \"" << alert_type_to_string(a.type) << "\",\n"
            << "      \"message\": \"" << a.message << "\"\n"
            << "    }" << (i + 1 < alerts_.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
    return true;
}

bool AlertManager::save_timeseries_to_json(const std::string& filepath) const
{
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "[AlertManager] Error: Unable to open " << filepath << " for writing\n";
        return false;
    }

    out << "{\n  \"timeseries\": [\n";
    for (size_t i = 0; i < timeseries_.size(); ++i) {
        const auto& p = timeseries_[i];
        out << "    {\n"
            << "      \"time\": " << std::fixed << std::setprecision(2) << p.time << ",\n"
            << "      \"from\": " << p.from_node << ",\n"
            << "      \"to\": " << p.to_node << ",\n"
            << "      \"flow\": " << std::fixed << std::setprecision(1) << p.flow << ",\n"
            << "      \"queue_length\": " << p.queue_length << ",\n"
            << "      \"vc_ratio\": " << std::fixed << std::setprecision(3) << p.vc_ratio << "\n"
            << "    }" << (i + 1 < timeseries_.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
    return true;
}

void AlertManager::clear()
{
    alerts_.clear();
    timeseries_.clear();
    congested_edges_.clear();
    dropout_edges_.clear();
}
