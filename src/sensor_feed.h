// =============================================================================
// sensor_feed.h  –  Real-time external sensor ingestion and flow blending
// Phase 5
// =============================================================================
#pragma once

#include "graph.h"
#include "alerts.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <chrono>

// =============================================================================
// SensorReading
// Single traffic telemetry observation from an external sensor/camera.
// =============================================================================
struct SensorReading {
    double timestamp;      // simulation time (minutes)
    int    from_node;
    int    to_node;
    double vehicle_count;  // observed flow rate (vehicles/hour)
    double avg_speed;      // observed speed (km/h)

    SensorReading()
        : timestamp(0.0), from_node(-1), to_node(-1), vehicle_count(0.0), avg_speed(0.0) {}

    SensorReading(double t, int f, int to, double vc, double spd)
        : timestamp(t), from_node(f), to_node(to), vehicle_count(vc), avg_speed(spd) {}
};

// =============================================================================
// CircularBuffer
// Custom circular array of fixed capacity K=5 for moving average calculation.
// Provides O(1) insertion and O(1) moving average computation via running sum.
// =============================================================================
template <typename T, size_t Capacity = 5>
class CircularBuffer {
public:
    CircularBuffer() : head_(0), count_(0), sum_(0) {
        buffer_.resize(Capacity, T());
    }

    void push(T val) {
        if (count_ == Capacity) {
            sum_ -= buffer_[head_];
        } else {
            count_++;
        }
        buffer_[head_] = val;
        sum_ += val;
        head_ = (head_ + 1) % Capacity;
    }

    double average() const {
        if (count_ == 0) return 0.0;
        return static_cast<double>(sum_) / static_cast<double>(count_);
    }

    size_t size() const { return count_; }
    size_t capacity() const { return Capacity; }

    void clear() {
        head_ = 0;
        count_ = 0;
        sum_ = 0;
        buffer_.assign(Capacity, T());
    }

private:
    std::vector<T> buffer_;
    size_t head_;
    size_t count_;
    T sum_;
};

// =============================================================================
// ThreadSafeQueue
// Thread-safe FIFO queue with condition variable synchronization for
// the threaded producer-consumer sensor ingestion demonstration.
// =============================================================================
template <typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;

    void push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
        cv_.notify_one();
    }

    bool try_pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool pop_wait(T& item, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cv_;
};

// =============================================================================
// SensorFeed
// Manages sensor data ingestion, smoothing, dropout detection, and blending.
// =============================================================================
class SensorFeed {
public:
    explicit SensorFeed(double blend_alpha = 0.5)
        : blend_alpha_(blend_alpha) {}

    // Ingest one sensor reading into the edge's circular buffer
    void add_reading(const SensorReading& r);

    // Ingest readings from CSV file. Skips malformed lines with warning.
    // Returns count of successfully ingested readings.
    int load_from_csv(const std::string& filepath);

    // Get current moving average sensor flow (vph) over last k=5 readings
    double get_sensor_flow(int from, int to) const;

    // Check if edge has experienced sensor dropout (> 3.0 min without reading)
    bool is_dropout(int from, int to, double current_time) const;

    // Get blended flow:
    //   flow = (1 - alpha)*sim_flow + alpha*sensor_flow
    // If sensor dropout is active, falls back 100% to sim_flow and emits alert.
    double get_blended_flow(int from, int to, double sim_flow, double current_time, AlertManager* alerts = nullptr);

    // Deterministic synthetic sensor reading generator for Source A
    std::vector<SensorReading> generate_synthetic_readings(const Graph& g, double t, unsigned int seed = 42);

    void clear();

    const std::vector<SensorReading>& get_all_readings() const { return readings_history_; }

    // Run threaded producer/consumer demo
    static bool run_threaded_demo();

private:
    int edge_key(int from, int to) const { return from * 1000 + to; }

    double blend_alpha_;  // blending weight: default 0.5
    std::unordered_map<int, CircularBuffer<double, 5>> buffers_;
    std::unordered_map<int, double> last_reading_time_;
    std::unordered_map<int, bool> reported_dropout_;
    std::vector<SensorReading> readings_history_;
};
