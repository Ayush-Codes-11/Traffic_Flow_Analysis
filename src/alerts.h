// =============================================================================
// alerts.h  –  Traffic alert system and time series recording
// Phase 5
// =============================================================================
#pragma once

#include <string>
#include <vector>
#include <unordered_map>

enum class AlertType {
    CONGESTION,
    CLOSURE,
    SENSOR_DROPOUT
};

inline std::string alert_type_to_string(AlertType t) {
    switch (t) {
        case AlertType::CONGESTION:      return "CONGESTION";
        case AlertType::CLOSURE:         return "CLOSURE";
        case AlertType::SENSOR_DROPOUT:  return "SENSOR_DROPOUT";
    }
    return "UNKNOWN";
}

struct Alert {
    double time;
    int from_node;
    int to_node;
    AlertType type;
    std::string message;
};

struct TimeSeriesPoint {
    double time;
    int from_node;
    int to_node;
    double flow;
    int queue_length;
    double vc_ratio;
};

class AlertManager {
public:
    AlertManager() = default;

    // Check V/C ratio on edge (from, to) at time t.
    // Hysteresis logic:
    //   If vc > 0.90 and !congested_edges_[key]:
    //       raise CONGESTION alert, mark congested_edges_[key] = true
    //   If vc < 0.80 and congested_edges_[key]:
    //       mark congested_edges_[key] = false (cleared)
    // Returns true if a new alert was raised.
    bool check_vc_congestion(double t, int from, int to, double vc);

    // Add closure alert
    void add_closure_alert(double t, int from, int to, const std::string& msg);

    // Add sensor dropout alert
    void add_dropout_alert(double t, int from, int to, const std::string& msg);

    // Generic add alert
    void add_alert(const Alert& a);

    // Add timeseries sample
    void record_timeseries_sample(double t, int from, int to, double flow, int q_len, double vc);

    const std::vector<Alert>& get_alerts() const { return alerts_; }
    const std::vector<TimeSeriesPoint>& get_timeseries() const { return timeseries_; }

    // JSON export
    bool save_alerts_to_json(const std::string& filepath) const;
    bool save_timeseries_to_json(const std::string& filepath) const;

    void clear();

private:
    int edge_key(int from, int to) const { return from * 1000 + to; }

    std::vector<Alert> alerts_;
    std::vector<TimeSeriesPoint> timeseries_;
    std::unordered_map<int, bool> congested_edges_;
    std::unordered_map<int, bool> dropout_edges_;
};
