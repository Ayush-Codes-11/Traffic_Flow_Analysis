// =============================================================================
// sensor_feed.cpp  –  Implementation of sensor data ingestion and smoothing
// Phase 5
// =============================================================================
#include "sensor_feed.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>

void SensorFeed::add_reading(const SensorReading& r)
{
    int key = edge_key(r.from_node, r.to_node);
    buffers_[key].push(r.vehicle_count);
    last_reading_time_[key] = r.timestamp;
    reported_dropout_[key]  = false;  // reading received -> dropout cleared
    readings_history_.push_back(r);
}

int SensorFeed::load_from_csv(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[SensorFeed] Warning: Could not open CSV file: " << filepath << "\n";
        return 0;
    }

    std::string line;
    int line_num = 0;
    int ingested = 0;
    int skipped  = 0;

    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;

        // Skip CSV header if present
        if (line_num == 1 && line.find("timestamp") != std::string::npos) {
            continue;
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            // Trim leading and trailing whitespace
            size_t start = token.find_first_not_of(" \t\r\n");
            size_t end   = token.find_last_not_of(" \t\r\n");
            if (start != std::string::npos && end != std::string::npos) {
                tokens.push_back(token.substr(start, end - start + 1));
            } else {
                tokens.push_back("");
            }
        }

        if (tokens.size() < 5) {
            std::cerr << "[SensorFeed] WARNING: Skipping malformed line " << line_num
                      << " (insufficient tokens): " << line << "\n";
            skipped++;
            continue;
        }

        try {
            double t     = std::stod(tokens[0]);
            int    from  = std::stoi(tokens[1]);
            int    to    = std::stoi(tokens[2]);
            double count = std::stod(tokens[3]);
            double speed = std::stod(tokens[4]);

            if (from < 0 || to < 0 || count < 0.0 || speed < 0.0) {
                throw std::invalid_argument("negative sensor field value");
            }

            SensorReading r(t, from, to, count, speed);
            add_reading(r);
            ingested++;
        } catch (const std::exception& ex) {
            std::cerr << "[SensorFeed] WARNING: Skipping malformed CSV line " << line_num
                      << " (" << ex.what() << "): " << line << "\n";
            skipped++;
        }
    }

    std::cout << "[SensorFeed] Ingested " << ingested << " readings from "
              << filepath << " (skipped " << skipped << " bad lines)\n";
    return ingested;
}

double SensorFeed::get_sensor_flow(int from, int to) const
{
    int key = edge_key(from, to);
    auto it = buffers_.find(key);
    if (it == buffers_.end()) return 0.0;
    return it->second.average();
}

bool SensorFeed::is_dropout(int from, int to, double current_time) const
{
    int key = edge_key(from, to);
    auto it = last_reading_time_.find(key);
    if (it == last_reading_time_.end()) {
        return true;  // never received any sensor reading
    }
    double elapsed = current_time - it->second;
    return (elapsed >= 3.0);  // dropout after 3 minutes without reading
}

// =============================================================================
// Flow Blending:
//   flow = (1 - alpha) * sim_flow + alpha * sensor_flow
// If sensor dropout is detected (>= 3 minutes with no fresh reading),
// we fall back 100% to simulated flow to prevent stale telemetry bias.
// =============================================================================
double SensorFeed::get_blended_flow(int from, int to, double sim_flow, double current_time, AlertManager* alerts)
{
    int key = edge_key(from, to);
    if (is_dropout(from, to, current_time)) {
        if (alerts && !reported_dropout_[key]) {
            reported_dropout_[key] = true;
            std::ostringstream msg;
            msg << "Sensor dropout on edge " << from << "->" << to
                << " at t=" << std::fixed << std::setprecision(2) << current_time
                << " (>= 3 min stale). Falling back to simulation flow.";
            alerts->add_dropout_alert(current_time, from, to, msg.str());
        }
        return sim_flow;
    }

    double sensor_obs = get_sensor_flow(from, to);
    return (1.0 - blend_alpha_) * sim_flow + blend_alpha_ * sensor_obs;
}

std::vector<SensorReading> SensorFeed::generate_synthetic_readings(const Graph& g, double t, unsigned int seed)
{
    std::vector<SensorReading> readings;
    uint32_t state = seed + static_cast<uint32_t>(t * 1000.0);

    for (const auto& [u, edges] : g.adj) {
        for (const auto& e : edges) {
            // Linear congruential pseudo-random perturbation
            state = (1664525u * state + 1013904223u);
            double noise_factor = 0.85 + 0.30 * (static_cast<double>(state & 0xFFFF) / 65535.0);

            double baseline = (e.flow > 0.0) ? e.flow : (e.capacity * 0.40);
            double obs_flow = std::max(0.0, baseline * noise_factor);

            // Speed estimation based on BPR slowdown
            double ratio = (e.capacity > 0) ? (obs_flow / e.capacity) : 0.0;
            double free_speed = (e.free_flow_time > 0) ? (e.length / (e.free_flow_time / 60.0)) : 50.0;
            double obs_speed = free_speed / (1.0 + 0.15 * std::pow(ratio, 4.0));

            SensorReading r(t, u, e.to, obs_flow, obs_speed);
            readings.push_back(r);
        }
    }
    return readings;
}

void SensorFeed::clear()
{
    buffers_.clear();
    last_reading_time_.clear();
    reported_dropout_.clear();
    readings_history_.clear();
}

// =============================================================================
// Threaded producer-consumer demonstration
// Shows thread-safe asynchronous telemetry ingestion using std::thread and
// a mutex-protected condition variable queue.
// =============================================================================
bool SensorFeed::run_threaded_demo()
{
    ThreadSafeQueue<SensorReading> queue;
    const int kNumReadings = 15;
    int received_count = 0;

    // Producer thread: simulates an external IoT gateway streaming sensor packets
    std::thread producer([&queue, kNumReadings]() {
        for (int i = 0; i < kNumReadings; ++i) {
            SensorReading r(static_cast<double>(i) * 0.5, 0, 13, 1200.0 + i * 20.0, 45.0);
            queue.push(r);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    // Consumer thread: simulates the simulation engine consuming real-time telemetry
    std::thread consumer([&queue, &received_count, kNumReadings]() {
        SensorReading item;
        while (received_count < kNumReadings) {
            if (queue.pop_wait(item, std::chrono::milliseconds(50))) {
                received_count++;
            }
        }
    });

    producer.join();
    consumer.join();

    return (received_count == kNumReadings);
}
