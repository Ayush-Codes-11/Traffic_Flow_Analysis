// =============================================================================
// vehicle_queue.h  –  Custom circular-array FIFO queue
// Phase 3
//
// WHY A CIRCULAR ARRAY?
//   A naive array wastes space: dequeue from the front shifts everything O(n).
//   A circular array keeps two indices (head, tail) that wrap around, so
//   both enqueue and dequeue are O(1).  When the array is full it doubles
//   in size (amortised O(1) enqueue, same as std::vector::push_back).
//
// VIVA: "The head index is where we read from (front of queue), the tail
//        index is where we write to (back of queue).  After each op we
//        advance the index with modulo so it wraps around to the start."
//
// Space: O(n) where n is the number of items currently in the queue.
// =============================================================================
#pragma once
#include <vector>
#include <stdexcept>

template<typename T>
class VehicleQueue {
public:
    // Create an empty queue with an initial backing-array capacity.
    explicit VehicleQueue(int initial_cap = 8)
        : data_(initial_cap), head_(0), tail_(0), count_(0) {}

    // ── enqueue ──────────────────────────────────────────────────────────────
    // Add item at the BACK of the queue.
    // If the array is full, double it first (re-linearise circular layout).
    // Time: O(1) amortised
    void enqueue(const T& item) {
        if (count_ == capacity()) resize();
        data_[tail_] = item;
        tail_ = (tail_ + 1) % capacity();  // wrap around
        ++count_;
    }

    // ── dequeue ───────────────────────────────────────────────────────────────
    // Remove and return the FRONT item.
    // Time: O(1)
    T dequeue() {
        if (isEmpty())
            throw std::underflow_error("VehicleQueue::dequeue() on empty queue");
        T item = data_[head_];
        head_  = (head_ + 1) % capacity();  // advance head, wrap around
        --count_;
        return item;
    }

    // ── front ─────────────────────────────────────────────────────────────────
    // Peek at the front item without removing it.  Time: O(1)
    const T& front() const {
        if (isEmpty())
            throw std::underflow_error("VehicleQueue::front() on empty queue");
        return data_[head_];
    }

    bool isEmpty() const { return count_ == 0; }
    int  size()    const { return count_; }
    void clear()         { head_ = tail_ = count_ = 0; }

private:
    std::vector<T> data_;
    int head_;    // index of next item to dequeue
    int tail_;    // index where next item will be enqueued
    int count_;   // items currently stored

    int capacity() const { return static_cast<int>(data_.size()); }

    // resize: double capacity, re-linearise the circular buffer.
    // After this, head=0, tail=count, items are contiguous.
    // Time: O(n)
    void resize() {
        int old_cap = capacity();
        std::vector<T> tmp(2 * old_cap);
        for (int i = 0; i < count_; ++i)
            tmp[i] = data_[(head_ + i) % old_cap];
        head_ = 0;
        tail_ = count_;
        data_ = std::move(tmp);
    }
};
