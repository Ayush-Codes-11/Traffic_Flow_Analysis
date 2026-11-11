// =============================================================================
// minheap.h  –  Custom Binary Min-Heap (template)
// Phase 2
//
// WHY A CUSTOM HEAP instead of std::priority_queue?
//   So you can explain every line in the viva:
//   push → heapify_up → bubbles new element toward the root
//   pop  → remove root, put last element at root → heapify_down
//
// WHY LAZY DELETION instead of decrease-key?
//   A proper decrease-key requires knowing the index of a node inside the
//   heap array, so we'd need a separate "position map" and O(V) extra memory,
//   plus more complicated swap-and-update logic.
//
//   With lazy deletion we just push a NEW (better) entry for a node.  When
//   we pop a node that's already been finalised (marked visited), we simply
//   skip it.  This keeps the code short, readable, and easy to explain.
//
//   Downside: the heap may store O(E) entries instead of O(V).  For road
//   networks where E ≈ 3V this is negligible.
//
// BINARY HEAP INVARIANT:
//   Parent priority ≤ children's priorities  (min-heap property).
//   Stored as a 1-D array where:
//     parent(i)       = (i-1)/2
//     left_child(i)   = 2*i + 1
//     right_child(i)  = 2*i + 2
// =============================================================================
#pragma once
#include <vector>
#include <stdexcept>

template<typename T>
class MinHeap {
public:
    // ── Item ─────────────────────────────────────────────────────────────────
    // Each heap slot holds a (priority, value) pair.
    // priority is always a double (travel-time, f-score, etc.)
    // value is T (typically int = node id, but templated for reuse)
    struct Item {
        double priority;
        T      value;
    };

    // ── push ─────────────────────────────────────────────────────────────────
    // Add a new item. Append at the end, then bubble it up to its correct spot.
    // Time complexity: O(log n)
    void push(double priority, const T& value) {
        data.push_back({priority, value});
        heapify_up(static_cast<int>(data.size()) - 1);
    }

    // ── pop ──────────────────────────────────────────────────────────────────
    // Remove and return the item with the SMALLEST priority (the root).
    // Steps:
    //   1. Save the root (minimum element).
    //   2. Move the LAST element to the root position.
    //   3. Remove the last slot.
    //   4. Sink the new root down until the heap property is restored.
    // Time complexity: O(log n)
    Item pop() {
        if (data.empty())
            throw std::underflow_error("MinHeap::pop() called on empty heap");

        Item top = data[0];             // save minimum
        data[0]  = data.back();         // move last element to root
        data.pop_back();                // shrink array

        if (!data.empty())
            heapify_down(0);            // restore heap property

        return top;
    }

    // ── peek ─────────────────────────────────────────────────────────────────
    // Look at the minimum without removing it.  Time: O(1)
    const Item& peek() const {
        if (data.empty())
            throw std::underflow_error("MinHeap::peek() called on empty heap");
        return data[0];
    }

    bool empty() const { return data.empty(); }
    int  size()  const { return static_cast<int>(data.size()); }

    // ── clear ────────────────────────────────────────────────────────────────
    void clear() { data.clear(); }

private:
    std::vector<Item> data;   // the backing array

    // ── heapify_up ───────────────────────────────────────────────────────────
    // After inserting at index i, compare it with its parent.
    // If it has a SMALLER priority, swap and move up.
    // Repeat until we reach the root (i==0) or the parent is already smaller.
    //
    // Viva: "It's like bubbling a lighter bubble upward through water."
    // Time: O(log n) — we traverse at most the height of the tree.
    void heapify_up(int i) {
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (data[i].priority < data[parent].priority) {
                std::swap(data[i], data[parent]);
                i = parent;             // move up and continue
            } else {
                break;                  // heap property satisfied
            }
        }
    }

    // ── heapify_down ─────────────────────────────────────────────────────────
    // After placing the last element at root (during pop), it may violate the
    // heap property.  Compare it with its TWO children and swap it with the
    // SMALLEST child.  Repeat until no children have a smaller priority.
    //
    // Viva: "It's like sinking a heavy stone downward through water."
    // Time: O(log n) — same height argument as heapify_up.
    void heapify_down(int i) {
        int n = static_cast<int>(data.size());
        while (true) {
            int left    = 2 * i + 1;   // left child index
            int right   = 2 * i + 2;   // right child index
            int smallest = i;           // assume current is smallest

            // Is left child smaller than current smallest?
            if (left < n && data[left].priority < data[smallest].priority)
                smallest = left;

            // Is right child smaller than current smallest?
            if (right < n && data[right].priority < data[smallest].priority)
                smallest = right;

            if (smallest == i) break;  // heap property satisfied, stop

            std::swap(data[i], data[smallest]);
            i = smallest;              // continue sinking downward
        }
    }
};
