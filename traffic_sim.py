"""
Traffic Management & Simulation System
Full Python Test Harness & Simulation Engine mirroring the C++17 implementation.
Used to execute all Phase 1-4 tests and verify exact convergence metrics.
"""
import json
import heapq
import sys
import os

class SimpleRNG:
    """Deterministic linear congruential generator (pure Python, zero C-extension dependencies)."""
    def __init__(self, seed=42):
        self.state = seed & 0xFFFFFFFF
    def randint(self, low, high):
        self.state = (1664525 * self.state + 1013904223) & 0xFFFFFFFF
        return low + (self.state % (high - low + 1))
    def uniform(self, low, high):
        self.state = (1664525 * self.state + 1013904223) & 0xFFFFFFFF
        return low + (self.state / 4294967295.0) * (high - low)

g_tests_total = 0
g_tests_passed = 0
g_tests_failed = 0

def report_check(name, pass_cond):
    global g_tests_total, g_tests_passed, g_tests_failed
    g_tests_total += 1
    if pass_cond:
        g_tests_passed += 1
        print(f"  {name}: PASS ✓")
    else:
        g_tests_failed += 1
        print(f"  {name}: FAIL ✗")

def section(title):
    print("\n" + "=" * 65)
    print("  " + title)
    print("=" * 65)

# ── Data Model ────────────────────────────────────────────────────────────────
class Node:
    def __init__(self, node_id, name, x, y):
        self.id = node_id
        self.name = name
        self.x = float(x)
        self.y = float(y)

class Edge:
    def __init__(self, u, v, length, capacity, free_flow_time):
        self.u = u
        self.v = v
        self.length = float(length)
        self.capacity = float(capacity)
        self.free_flow_time = float(free_flow_time)
        self.flow = 0.0
        self.closed = False

    def bpr_time(self):
        if self.closed:
            return 1e18
        ratio = (self.flow / self.capacity) if self.capacity > 0 else 0.0
        return self.free_flow_time * (1.0 + 0.15 * (ratio ** 4.0))

class Graph:
    def __init__(self):
        self.nodes = {}
        self.adj = {}

    def add_node(self, node):
        self.nodes[node.id] = node
        if node.id not in self.adj:
            self.adj[node.id] = []

    def add_edge(self, edge):
        if edge.u not in self.adj:
            self.adj[edge.u] = []
        self.adj[edge.u].append(edge)

    def load_from_json(self, filepath):
        if not os.path.exists(filepath):
            return False
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return False

        for n in data.get("nodes", []):
            try:
                self.add_node(Node(int(n["id"]), str(n.get("name", "")), float(n["x"]), float(n["y"])))
            except Exception:
                pass

        for e in data.get("edges", []):
            try:
                u, v = int(e["from"]), int(e["to"])
                # Skip edge if endpoints not in node list (Fix 2 requirement)
                if u not in self.nodes or v not in self.nodes:
                    continue
                length = float(e["length"])
                cap = float(e["capacity"])
                fft = float(e["free_flow_time"])
                if cap <= 0 or fft <= 0:
                    continue
                self.add_edge(Edge(u, v, length, cap, fft))
                self.add_edge(Edge(v, u, length, cap, fft))
            except Exception:
                pass

        return len(self.nodes) > 0

    def euclidean_dist(self, a, b):
        na, nb = self.nodes[a], self.nodes[b]
        dx = na.x - nb.x
        dy = na.y - nb.y
        return (dx * dx + dy * dy) ** 0.5

    def get_max_speed(self):
        max_spd = 1.0
        for u, edges in self.adj.items():
            for e in edges:
                if e.free_flow_time > 0:
                    spd = self.euclidean_dist(u, e.v) / e.free_flow_time
                    if spd > max_spd:
                        max_spd = spd
        return max_spd

    def get_bpr_time(self, u, v):
        for e in self.adj.get(u, []):
            if e.v == v:
                return e.bpr_time()
        return -1.0

    def set_flow(self, u, v, flow):
        for e in self.adj.get(u, []):
            if e.v == v:
                e.flow = flow
                return True
        return False

    def set_closed(self, u, v, closed):
        for e in self.adj.get(u, []):
            if e.v == v:
                e.closed = closed
                return True
        return False

    def edge_count(self):
        return sum(len(edges) for edges in self.adj.values())

# ── MinHeap & Custom Shortest Path ───────────────────────────────────────────
class CustomMinHeap:
    def __init__(self):
        self.data = []

    def push(self, priority, value):
        heapq.heappush(self.data, (priority, value))

    def pop(self):
        p, v = heapq.heappop(self.data)
        return p, v

    def empty(self):
        return len(self.data) == 0

    def size(self):
        return len(self.data)

def dijkstra_custom(g, src, dst, weight_fn=None):
    if weight_fn is None:
        weight_fn = lambda u, v, e: e.bpr_time()

    dist = {nid: float("inf") for nid in g.nodes}
    parent = {nid: None for nid in g.nodes}
    visited = set()

    dist[src] = 0.0
    heap = CustomMinHeap()
    heap.push(0.0, src)
    nodes_explored = 0

    while not heap.empty():
        d, u = heap.pop()
        if u in visited:
            continue
        visited.add(u)
        nodes_explored += 1
        if u == dst:
            break

        for e in g.adj.get(u, []):
            if e.closed:
                continue
            w = max(0.0, weight_fn(u, e.v, e))
            if d + w < dist[e.v]:
                dist[e.v] = d + w
                parent[e.v] = u
                heap.push(d + w, e.v)

    found = dist.get(dst, float("inf")) < float("inf") / 2.0
    path = []
    if found:
        curr = dst
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
    return {"found": found, "total_time": dist.get(dst, -1.0) if found else -1.0,
            "nodes_explored": nodes_explored, "path": path}

def astar(g, src, dst):
    max_spd = g.get_max_speed()
    heuristic = lambda u: 0.0 if u == dst else g.euclidean_dist(u, dst) / max_spd

    g_score = {nid: float("inf") for nid in g.nodes}
    parent = {nid: None for nid in g.nodes}
    visited = set()

    g_score[src] = 0.0
    heap = CustomMinHeap()
    heap.push(heuristic(src), src)
    nodes_explored = 0

    while not heap.empty():
        _, u = heap.pop()
        if u in visited:
            continue
        visited.add(u)
        nodes_explored += 1
        if u == dst:
            break

        for e in g.adj.get(u, []):
            if e.closed:
                continue
            tentative = g_score[u] + e.bpr_time()
            if tentative < g_score[e.v]:
                g_score[e.v] = tentative
                parent[e.v] = u
                heap.push(tentative + heuristic(e.v), e.v)

    found = g_score.get(dst, float("inf")) < float("inf") / 2.0
    path = []
    if found:
        curr = dst
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
    return {"found": found, "total_time": g_score.get(dst, -1.0) if found else -1.0,
            "nodes_explored": nodes_explored, "path": path}

# ── Vehicle Queue ─────────────────────────────────────────────────────────────
class VehicleQueue:
    def __init__(self, capacity=8):
        self.data = [None] * capacity
        self.head = 0
        self.tail = 0
        self.count = 0

    def capacity(self):
        return len(self.data)

    def enqueue(self, item):
        if self.count == self.capacity():
            self._resize()
        self.data[self.tail] = item
        self.tail = (self.tail + 1) % self.capacity()
        self.count += 1

    def dequeue(self):
        if self.isEmpty():
            raise IndexError("VehicleQueue dequeue on empty queue")
        item = self.data[self.head]
        self.head = (self.head + 1) % self.capacity()
        self.count -= 1
        return item

    def isEmpty(self):
        return self.count == 0

    def size(self):
        return self.count

    def _resize(self):
        old_cap = self.capacity()
        new_data = [None] * (old_cap * 2)
        for i in range(self.count):
            new_data[i] = self.data[(self.head + i) % old_cap]
        self.data = new_data
        self.head = 0
        self.tail = self.count

# ── Event-Driven Simulation Engine ───────────────────────────────────────────
class Vehicle:
    def __init__(self, vid, src, dst, entry_time):
        self.id = vid
        self.src = src
        self.dst = dst
        self.route = []
        self.route_pos = 0
        self.entry_time = entry_time
        self.finish_time = -1.0
        self.total_wait = 0.0
        self.completed = False

class Simulation:
    def __init__(self, g, green_duration=2.0, headway=0.1, log_vehicle_id=-1):
        self.g = g
        self.green_duration = green_duration
        self.headway = headway
        self.log_vid = log_vehicle_id
        self.vehicles = []
        self.queues = {}
        self.edge_active = {}
        self.edge_stats = {}
        self.event_heap = []
        self.timelines = {}
        self.veh_join_time = {}

    def add_vehicle(self, src, dst, spawn_time):
        vid = len(self.vehicles)
        v = Vehicle(vid, src, dst, spawn_time)
        pr = astar(self.g, src, dst)
        if not pr["found"] or len(pr["path"]) < 2:
            v.completed = True
            v.finish_time = spawn_time
            self.vehicles.append(v)
            return
        v.route = pr["path"]
        self.vehicles.append(v)
        heapq.heappush(self.event_heap, (spawn_time, "ARRIVE", vid, src, v.route[1]))

    def run(self, max_time=300.0):
        while self.event_heap:
            t, etype, vid, u, v = heapq.heappop(self.event_heap)
            if t > max_time:
                break
            if etype == "ARRIVE":
                self._on_arrive(vid, u, v, t)
            elif etype == "ENTER":
                self._on_enter(u, v, t)
            elif etype == "EXIT":
                self._on_exit(vid, u, v, t)
            elif etype == "SIGNAL":
                self._on_signal(u, v, t)

    def _is_green(self, u, v, t):
        if self.green_duration <= 0.0:
            return True
        edges = self.g.adj.get(u, [])
        if len(edges) <= 1:
            return True
        idx = next((i for i, e in enumerate(edges) if e.v == v), -1)
        if idx < 0:
            return True
        phase = int(t / self.green_duration) % len(edges)
        return phase == idx

    def _next_green(self, u, v, t):
        if self.green_duration <= 0.0 or self._is_green(u, v, t):
            return t
        edges = self.g.adj.get(u, [])
        idx = next((i for i, e in enumerate(edges) if e.v == v), -1)
        if idx < 0 or len(edges) <= 1:
            return t
        n = len(edges)
        cur_phase = int(t / self.green_duration) % n
        ahead = (idx - cur_phase + n) % n
        phase_start = int(t // self.green_duration) * self.green_duration
        return phase_start + ahead * self.green_duration

    def _on_arrive(self, vid, u, v, t):
        key = (u, v)
        if key not in self.queues:
            self.queues[key] = VehicleQueue()
        was_empty = self.queues[key].isEmpty()
        self.queues[key].enqueue(vid)
        self.veh_join_time[vid] = t

        if was_empty:
            if self._is_green(u, v, t):
                heapq.heappush(self.event_heap, (t, "ENTER", -1, u, v))
            else:
                next_g = self._next_green(u, v, t)
                heapq.heappush(self.event_heap, (next_g, "SIGNAL", -1, u, v))

    def _on_enter(self, u, v, t):
        key = (u, v)
        if key not in self.queues or self.queues[key].isEmpty():
            return
        if not self._is_green(u, v, t):
            next_g = self._next_green(u, v, t)
            heapq.heappush(self.event_heap, (next_g, "SIGNAL", -1, u, v))
            return

        vid = self.queues[key].dequeue()
        wait = t - self.veh_join_time.get(vid, t)
        self.vehicles[vid].total_wait += wait

        self.edge_active[key] = self.edge_active.get(key, 0) + 1
        edge = next((e for e in self.g.adj[u] if e.v == v), None)
        active = self.edge_active[key]
        flow_vph = (active * 60.0 / edge.free_flow_time) if edge.free_flow_time > 0 else 0.0
        edge.flow = flow_vph

        ratio = flow_vph / edge.capacity if edge.capacity > 0 else 0.0
        bpr_t = edge.free_flow_time * (1.0 + 0.15 * (ratio ** 4.0))

        heapq.heappush(self.event_heap, (t + bpr_t, "EXIT", vid, u, v))

        # Fix 1: Cascade with else-branch scheduling SIGNAL_CHECK when light turns red
        if not self.queues[key].isEmpty():
            if self._is_green(u, v, t + self.headway):
                heapq.heappush(self.event_heap, (t + self.headway, "ENTER", -1, u, v))
            else:
                green_t = self._next_green(u, v, t + self.headway)
                heapq.heappush(self.event_heap, (green_t, "SIGNAL", -1, u, v))

    def _on_exit(self, vid, u, v, t):
        key = (u, v)
        self.edge_active[key] = max(0, self.edge_active.get(key, 0) - 1)
        edge = next((e for e in self.g.adj[u] if e.v == v), None)
        active = self.edge_active[key]
        edge.flow = (active * 60.0 / edge.free_flow_time) if edge.free_flow_time > 0 else 0.0

        veh = self.vehicles[vid]
        veh.route_pos += 1
        if v == veh.dst or veh.route_pos >= len(veh.route) - 1:
            veh.completed = True
            veh.finish_time = t
        else:
            nxt_v = veh.route[veh.route_pos + 1]
            heapq.heappush(self.event_heap, (t, "ARRIVE", vid, v, nxt_v))

    def _on_signal(self, u, v, t):
        key = (u, v)
        if key not in self.queues or self.queues[key].isEmpty():
            return
        if self._is_green(u, v, t):
            heapq.heappush(self.event_heap, (t, "ENTER", -1, u, v))
        else:
            green_t = self._next_green(u, v, t)
            heapq.heappush(self.event_heap, (green_t, "SIGNAL", -1, u, v))

    def print_summary(self):
        done = [v for v in self.vehicles if v.completed]
        print(f"  Vehicles spawned   : {len(self.vehicles)}")
        print(f"  Vehicles completed : {len(done)}")
        if done:
            avg_tt = sum(v.finish_time - v.entry_time for v in done) / len(done)
            avg_w = sum(v.total_wait for v in done) / len(done)
            print(f"  Avg travel time    : {avg_tt:.2f} min")
            print(f"  Avg signal wait    : {avg_w:.2f} min")

# ── Traffic Assignment (AON, MSA, Frank-Wolfe) ───────────────────────────────
def auxiliary_aon(g, demands, weight_fn):
    y = {}
    for u, edges in g.adj.items():
        for e in edges:
            y[(u, e.v)] = 0.0

    for dem in demands:
        src, dst, vol = dem["origin"], dem["destination"], dem["demand"]
        if vol <= 0 or src == dst:
            continue
        res = dijkstra_custom(g, src, dst, weight_fn)
        if res["found"] and len(res["path"]) >= 2:
            for i in range(len(res["path"]) - 1):
                key = (res["path"][i], res["path"][i+1])
                y[key] = y.get(key, 0.0) + vol
    return y

def compute_tstt(g, flows):
    tstt = 0.0
    for u, edges in g.adj.items():
        for e in edges:
            x = flows.get((u, e.v), 0.0)
            if x > 0:
                ratio = (x / e.capacity) if e.capacity > 0 else 0.0
                t = e.free_flow_time * (1.0 + 0.15 * (ratio ** 4.0))
                tstt += x * t
    return tstt

def run_aon(g, demands):
    flows = auxiliary_aon(g, demands, lambda u, v, e: e.free_flow_time)
    tstt = compute_tstt(g, flows)
    max_vc = max(flows.get((u, e.v), 0.0) / e.capacity for u, edges in g.adj.items() for e in edges)
    over_cap = sum(1 for u, edges in g.adj.items() for e in edges if flows.get((u, e.v), 0.0) > e.capacity)
    return {"method": "AON", "iterations": 1, "final_gap": 0.0, "total_travel_time": tstt,
            "max_vc": max_vc, "edges_over_capacity": over_cap, "flows": flows}

def run_msa(g, demands, system_optimal=False, max_iter=1000, target_gap=1e-3, verbose=False):
    current_x = auxiliary_aon(g, demands, lambda u, v, e: e.free_flow_time)
    last_gap = 1.0
    iter_count = 1

    for it in range(2, max_iter + 1):
        iter_count = it
        if not system_optimal:
            cost_fn = lambda u, v, e: e.free_flow_time * (1.0 + 0.15 * ((current_x.get((u, v), 0.0) / e.capacity) ** 4.0))
        else:
            cost_fn = lambda u, v, e: e.free_flow_time * (1.0 + 0.75 * ((current_x.get((u, v), 0.0) / e.capacity) ** 4.0))

        y = auxiliary_aon(g, demands, cost_fn)

        tot_cost = sum(current_x.get((u, e.v), 0.0) * cost_fn(u, e.v, e) for u, edges in g.adj.items() for e in edges)
        min_cost = sum(y.get((u, e.v), 0.0) * cost_fn(u, e.v, e) for u, edges in g.adj.items() for e in edges)
        gap = max(0.0, (tot_cost - min_cost) / tot_cost) if tot_cost > 1e-9 else 0.0
        last_gap = gap

        if verbose and (it == 2 or it % 50 == 0 or gap < target_gap or it == max_iter):
            cur_tstt = compute_tstt(g, current_x)
            mname = "SO_MSA" if system_optimal else "UE_MSA"
            print(f"  [{mname}] Iter {it:4d} │ Step λ={1.0/it:.4f} │ RelGap={gap:.6f} │ TSTT={cur_tstt:.1f} veh-min/h")

        if gap < target_gap:
            break

        step = 1.0 / it
        for k in current_x:
            current_x[k] += step * (y.get(k, 0.0) - current_x[k])

    tstt = compute_tstt(g, current_x)
    max_vc = max(current_x.get((u, e.v), 0.0) / e.capacity for u, edges in g.adj.items() for e in edges)
    over_cap = sum(1 for u, edges in g.adj.items() for e in edges if current_x.get((u, e.v), 0.0) > e.capacity)
    return {"method": "SO_MSA" if system_optimal else "UE_MSA", "iterations": iter_count,
            "final_gap": last_gap, "total_travel_time": tstt, "max_vc": max_vc,
            "edges_over_capacity": over_cap, "flows": current_x}

def run_frank_wolfe(g, demands, system_optimal=False, max_iter=1000, target_gap=1e-3, verbose=False):
    current_x = auxiliary_aon(g, demands, lambda u, v, e: e.free_flow_time)
    last_gap = 1.0
    iter_count = 1

    for it in range(1, max_iter + 1):
        iter_count = it
        if not system_optimal:
            cost_fn = lambda u, v, e: e.free_flow_time * (1.0 + 0.15 * ((current_x.get((u, v), 0.0) / e.capacity) ** 4.0))
        else:
            cost_fn = lambda u, v, e: e.free_flow_time * (1.0 + 0.75 * ((current_x.get((u, v), 0.0) / e.capacity) ** 4.0))

        y = auxiliary_aon(g, demands, cost_fn)

        tot_cost = sum(current_x.get((u, e.v), 0.0) * cost_fn(u, e.v, e) for u, edges in g.adj.items() for e in edges)
        min_cost = sum(y.get((u, e.v), 0.0) * cost_fn(u, e.v, e) for u, edges in g.adj.items() for e in edges)
        gap = max(0.0, (tot_cost - min_cost) / tot_cost) if tot_cost > 1e-9 else 0.0
        last_gap = gap

        if verbose and (it == 1 or it % 10 == 0 or gap < target_gap or it == max_iter):
            cur_tstt = compute_tstt(g, current_x)
            mname = "SO_FW" if system_optimal else "UE_FW"
            print(f"  [{mname}] Iter {it:4d} │ RelGap={gap:.6f} │ TSTT={cur_tstt:.1f} veh-min/h")

        if gap < target_gap:
            break

        # Exact bisection line search on lambda in [0, 1]
        def dZ_dlambda(lam):
            s = 0.0
            for u, edges in g.adj.items():
                for e in edges:
                    xa = current_x.get((u, e.v), 0.0)
                    ya = y.get((u, e.v), 0.0)
                    wa = max(0.0, xa + lam * (ya - xa))
                    ratio = (wa / e.capacity) if e.capacity > 0 else 0.0
                    if not system_optimal:
                        c = e.free_flow_time * (1.0 + 0.15 * (ratio ** 4.0))
                    else:
                        c = e.free_flow_time * (1.0 + 0.75 * (ratio ** 4.0))
                    s += (ya - xa) * c
            return s

        if dZ_dlambda(0.0) >= 0.0:
            opt_lam = 0.0
        elif dZ_dlambda(1.0) <= 0.0:
            opt_lam = 1.0
        else:
            lo, hi = 0.0, 1.0
            for _ in range(25):
                mid = 0.5 * (lo + hi)
                if dZ_dlambda(mid) < 0.0:
                    lo = mid
                else:
                    hi = mid
            opt_lam = 0.5 * (lo + hi)

        for k in current_x:
            current_x[k] += opt_lam * (y.get(k, 0.0) - current_x[k])

    tstt = compute_tstt(g, current_x)
    max_vc = max(current_x.get((u, e.v), 0.0) / e.capacity for u, edges in g.adj.items() for e in edges)
    over_cap = sum(1 for u, edges in g.adj.items() for e in edges if current_x.get((u, e.v), 0.0) > e.capacity)
    return {"method": "SO_FW" if system_optimal else "UE_FW", "iterations": iter_count,
            "final_gap": last_gap, "total_travel_time": tstt, "max_vc": max_vc,
            "edges_over_capacity": over_cap, "flows": current_x}

def export_assignment_json(g, res, filepath):
    edges_out = []
    for u, edges in g.adj.items():
        for e in edges:
            x = res["flows"].get((u, e.v), 0.0)
            ratio = (x / e.capacity) if e.capacity > 0 else 0.0
            t = e.free_flow_time * (1.0 + 0.15 * (ratio ** 4.0))
            edges_out.append({
                "from": u,
                "to": e.v,
                "from_name": g.nodes[u].name,
                "to_name": g.nodes[e.v].name,
                "flow": round(x, 1),
                "capacity": round(e.capacity, 1),
                "free_flow_time": round(e.free_flow_time, 2),
                "travel_time": round(t, 3),
                "vc_ratio": round(ratio, 3)
            })
    doc = {
        "summary": {
            "method": res["method"],
            "iterations": res["iterations"],
            "final_gap": round(res["final_gap"], 6),
            "total_travel_time_veh_min": round(res["total_travel_time"], 2),
            "total_travel_time_veh_hrs": round(res["total_travel_time"] / 60.0, 2),
            "max_vc": round(res["max_vc"], 3),
            "edges_over_capacity": res["edges_over_capacity"]
        },
        "edges": edges_out
    }
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
    return True

# ── Main Test Runner ──────────────────────────────────────────────────────────
def main():
    print("Traffic Management & Simulation System")
    print("Phase 1 + Phase 2 + Phase 3 + Phase 4 Tests")
    print("*" * 65)

    # ── Phase 1 ─────────────────────────────────────────────────────────
    section("P1-TEST 1 – Load graph from JSON")
    g1 = Graph()
    ok1 = g1.load_from_json("data/sample_city.json")
    print(f"Nodes: {len(g1.nodes)}  |  Directed edges: {g1.edge_count()}")
    report_check("P1-TEST 1: Load sample_city.json", ok1 and len(g1.nodes) == 15 and g1.edge_count() == 50)

    section("P1-TEST 2 – Load deliberately bad JSON (error handling)")
    bad = Graph()
    ok2 = bad.load_from_json("data/bad_city.json")
    print(f"Return value  : {'true (partial)' if ok2 else 'false'}")
    print(f"Nodes loaded  : {len(bad.nodes)}")
    print(f"Edges loaded  : {bad.edge_count()}")
    report_check("P1-TEST 2a: Partial load with skipped malformed entries", ok2 and len(bad.nodes) == 1)

    ghost = Graph()
    ok_ghost = ghost.load_from_json("data/does_not_exist.json")
    report_check("P1-TEST 2b: Nonexistent file returns false", not ok_ghost)

    section("P1-TEST 3 – BPR travel-time (edge 0→13, t0=3 min, cap=2000)")
    t_zero = g1.get_bpr_time(0, 13)
    g1.set_flow(0, 13, 2000.0)
    t_cap = g1.get_bpr_time(0, 13)
    g1.set_flow(0, 13, 0.0)
    report_check("P1-TEST 3: BPR matches analytical t0 and capacity slowdown",
                 abs(t_zero - 3.0) < 1e-4 and abs(t_cap - 3.45) < 1e-4)

    section("P1-TEST 4 – Euclidean distance (A* heuristic preview)")
    report_check("P1-TEST 4: Euclidean distances correct", g1.euclidean_dist(0, 8) == 6.0)

    section("P1-TEST 5 – Road closure")
    g1.set_closed(0, 5, True)
    c_t = g1.get_bpr_time(0, 5)
    g1.set_closed(0, 5, False)
    o_t = g1.get_bpr_time(0, 5)
    report_check("P1-TEST 5: Road closure and reopening", c_t > 1e10 and abs(o_t - 5.0) < 1e-4)

    # ── Phase 2 ─────────────────────────────────────────────────────────
    section("P2-TEST 1 – Custom MinHeap (push / pop / heapify)")
    mh = CustomMinHeap()
    for v in [7, 3, 8, 1, 9, 2, 10, 4, 6, 5]:
        mh.push(float(v), v)
    out = [mh.pop()[1] for _ in range(10)]
    report_check("P2-TEST 1a: Custom MinHeap pop order sorted ascending", out == sorted(out))

    mh2 = CustomMinHeap()
    mh2.push(10.0, 42)
    mh2.push(5.0, 42)
    report_check("P2-TEST 1b: Lazy-deletion duplicate push order", mh2.pop()[0] == 5.0)

    section("P2-TEST 2 – Dijkstra vs A* (5 source/destination pairs)")
    pairs = [(1, 5), (9, 11), (12, 3), (10, 8), (0, 7)]
    all_match = True
    for s, d in pairs:
        rd = dijkstra_custom(g1, s, d)
        ra = astar(g1, s, d)
        if abs(rd["total_time"] - ra["total_time"]) > 1e-6:
            all_match = False
    report_check("P2-TEST 2: Dijkstra and A* return identical travel times", all_match)

    section("P2-TEST 3 – Edge Cases")
    r_same = dijkstra_custom(g1, 0, 0)
    report_check("P2-TEST 3a: src == dst returns total_time=0.0", r_same["found"] and r_same["total_time"] == 0.0)

    g1.add_node(Node(15, "Isolated_Island", 100.0, 100.0))
    r_iso_d = dijkstra_custom(g1, 0, 15)
    r_iso_a = astar(g1, 0, 15)
    report_check("P2-TEST 3b: Isolated node unreachable (found=false)", not r_iso_d["found"] and not r_iso_a["found"])

    r_open = dijkstra_custom(g1, 0, 7)
    g1.set_closed(0, 13, True)
    g1.set_closed(13, 0, True)
    r_detour = dijkstra_custom(g1, 0, 7)
    g1.set_closed(0, 13, False)
    g1.set_closed(13, 0, False)
    report_check("P2-TEST 3c: Road closure forces path detour", r_open["path"] != r_detour["path"])

    section("P2-TEST 4 – Congestion Changes the Optimal Route")
    r_base = dijkstra_custom(g1, 0, 7)
    g1.set_flow(0, 13, 5000.0)
    g1.set_flow(13, 0, 5000.0)
    r_cong = dijkstra_custom(g1, 0, 7)
    g1.set_flow(0, 13, 0.0)
    g1.set_flow(13, 0, 0.0)
    report_check("P2-TEST 4: Overloading bottleneck alters shortest path", r_base["path"] != r_cong["path"])

    # ── Phase 3 ─────────────────────────────────────────────────────────
    section("P3-TEST 1 – VehicleQueue FIFO ordering + empty-queue safety")
    vq = VehicleQueue()
    for v in [1, 2, 3, 4, 5]:
        vq.enqueue(v)
    deq = [vq.dequeue() for _ in range(5)]
    report_check("P3-TEST 1a: VehicleQueue FIFO dequeue order", deq == [1, 2, 3, 4, 5])

    threw = False
    try:
        vq.dequeue()
    except IndexError:
        threw = True
    report_check("P3-TEST 1b: VehicleQueue underflow exception", threw)

    vq2 = VehicleQueue()
    for i in range(20):
        vq2.enqueue(i)
    r_deq = [vq2.dequeue() for _ in range(20)]
    report_check("P3-TEST 1c: VehicleQueue dynamic circular resize", r_deq == list(range(20)))

    section("P3-TEST 2 – Single vehicle, no signals (travel time = free-flow)")
    g3 = Graph()
    g3.load_from_json("data/sample_city.json")
    sim1 = Simulation(g3, green_duration=0.0, headway=0.1)
    sim1.add_vehicle(0, 7, 0.0)
    sim1.run(60.0)
    v0 = sim1.vehicles[0]
    report_check("P3-TEST 2: Single vehicle travel time matches free-flow ~7 min",
                 v0.completed and abs((v0.finish_time - v0.entry_time) - 7.0) < 0.2)

    section("P3-TEST 3 – Signal forces a wait (red → green transition)")
    g3b = Graph()
    g3b.load_from_json("data/sample_city.json")
    sim2 = Simulation(g3b, green_duration=2.0, headway=0.1)
    sim2.add_vehicle(0, 10, 0.5)
    sim2.run(60.0)
    v_sig = sim2.vehicles[0]
    report_check("P3-TEST 3: Signal red phase forces vehicle wait time > 0", v_sig.completed and v_sig.total_wait > 0.0)

    section("P3-TEST 4 – 20 vehicles, queue buildup with signals (Fix 1 verification)")
    g3c = Graph()
    g3c.load_from_json("data/sample_city.json")
    sim3 = Simulation(g3c, green_duration=0.5, headway=0.1)
    for _ in range(20):
        sim3.add_vehicle(0, 7, 0.0)
    sim3.run(400.0)
    done_20 = sum(1 for v in sim3.vehicles if v.completed)
    sim3.print_summary()
    report_check("P3-TEST 4: All 20 vehicles complete through short signal cycle (no stranding)", done_20 == 20)

    section("P3-TEST 5 – Full simulation: 50 random vehicles, signals on")
    g3d = Graph()
    g3d.load_from_json("data/sample_city.json")
    sim4 = Simulation(g3d, green_duration=2.0, headway=0.1)
    rng = SimpleRNG(42)
    for _ in range(50):
        src = rng.randint(0, 14)
        dst = rng.randint(0, 14)
        while dst == src:
            dst = rng.randint(0, 14)
        sp = rng.uniform(0.0, 30.0)
        sim4.add_vehicle(src, dst, sp)
    sim4.run(400.0)
    done_50 = sum(1 for v in sim4.vehicles if v.completed)
    sim4.print_summary()
    report_check("P3-TEST 5: All 50 stochastic vehicles complete", done_50 == 50)

    # ── Phase 4 ─────────────────────────────────────────────────────────
    section("P4-TEST 1 – 2-Route Pigou Validation (UE Route Balance & SO Optimality)")
    pigou = Graph()
    pigou.add_node(Node(0, "Origin", 0.0, 0.0))
    pigou.add_node(Node(1, "Route1_Mid", 5.0, 2.0))
    pigou.add_node(Node(2, "Route2_Mid", 5.0, -2.0))
    pigou.add_node(Node(3, "Destination", 10.0, 0.0))
    pigou.add_edge(Edge(0, 1, 5.0, 1000.0, 5.0))
    pigou.add_edge(Edge(1, 3, 5.0, 1000.0, 5.0))
    pigou.add_edge(Edge(0, 2, 5.0, 2000.0, 7.5))
    pigou.add_edge(Edge(2, 3, 5.0, 2000.0, 7.5))

    pigou_dem = [{"origin": 0, "destination": 3, "demand": 2000.0}]
    p_aon = run_aon(pigou, pigou_dem)
    p_ue = run_frank_wolfe(pigou, pigou_dem, system_optimal=False, max_iter=200, target_gap=1e-4)
    p_so = run_frank_wolfe(pigou, pigou_dem, system_optimal=True, max_iter=200, target_gap=1e-4)

    f1 = p_ue["flows"].get((0, 1), 0.0)
    f2 = p_ue["flows"].get((0, 2), 0.0)
    t1 = 2.0 * 5.0 * (1.0 + 0.15 * ((f1 / 1000.0) ** 4.0))
    t2 = 2.0 * 7.5 * (1.0 + 0.15 * ((f2 / 2000.0) ** 4.0))

    print(f"  Pigou AON TSTT: {p_aon['total_travel_time']:.1f} veh-min/h")
    print(f"  Pigou UE  TSTT: {p_ue['total_travel_time']:.1f} veh-min/h  (Route1: {t1:.2f} min, Route2: {t2:.2f} min)")
    print(f"  Pigou SO  TSTT: {p_so['total_travel_time']:.1f} veh-min/h")
    report_check("P4-TEST 1a: Pigou 2-Route UE Travel Times Equal (|t1-t2| < 0.25)", abs(t1 - t2) < 0.25)
    report_check("P4-TEST 1b: Pigou 2-Route SO Total Travel Time <= UE Total Travel Time",
                 p_so["total_travel_time"] <= p_ue["total_travel_time"] + 1e-4)

    section("P4-TEST 2 – Load Scaled OD Demand Table (data/demand.json)")
    with open("data/demand.json", "r", encoding="utf-8") as f:
        raw_dem = json.load(f)
    scale = 0.4
    dem_04 = [{"origin": d["origin"], "destination": d["destination"], "demand": d["demand"] * scale} for d in raw_dem]
    tot_d_04 = sum(d["demand"] for d in dem_04)
    print(f"Loaded {len(dem_04)} OD pairs with scale {scale:.1f} (Total Demand = {tot_d_04:.0f} vph)")
    report_check("P4-TEST 2: Load 10 OD demand pairs at scale 0.4", len(dem_04) == 10 and tot_d_04 == 5520)

    g4 = Graph()
    g4.load_from_json("data/sample_city.json")

    aon_04 = run_aon(g4, dem_04)
    ue_msa_04 = run_msa(g4, dem_04, system_optimal=False, max_iter=1000, target_gap=1e-3, verbose=True)
    ue_fw_04 = run_frank_wolfe(g4, dem_04, system_optimal=False, max_iter=1000, target_gap=1e-3, verbose=True)
    so_fw_04 = run_frank_wolfe(g4, dem_04, system_optimal=True, max_iter=1000, target_gap=1e-3, verbose=True)

    print("\nCity Network Assignment Comparison (Scale = 0.4, Demand = 5520 vph):")
    print(f"{'Method':<12} {'Iters':<8} {'Final RelGap':<14} {'TSTT (veh-min/h)':<18} {'TSTT (veh-hrs)':<16} {'Max V/C':<10} {'Over-Cap Edges'}")
    print("=" * 88)
    for r in [aon_04, ue_msa_04, ue_fw_04, so_fw_04]:
        print(f"{r['method']:<12} {r['iterations']:<8} {r['final_gap']:<14.6f} {r['total_travel_time']:<18.1f} {r['total_travel_time']/60.0:<16.2f} {r['max_vc']:<10.3f} {r['edges_over_capacity']}")
    print("=" * 88)

    report_check("P4-TEST 3: AON successfully evaluated on city network", aon_04["total_travel_time"] > 0)
    report_check("P4-TEST 4a: Frank-Wolfe UE reached RelGap < 1e-3", ue_fw_04["final_gap"] < 1e-3)
    report_check("P4-TEST 4b: Frank-Wolfe SO reached RelGap < 1e-3", so_fw_04["final_gap"] < 1e-3)
    report_check("P4-TEST 4c: Frank-Wolfe SO TSTT <= UE TSTT * 1.001",
                 so_fw_04["total_travel_time"] <= ue_fw_04["total_travel_time"] * 1.001)

    export_ok = export_assignment_json(g4, ue_fw_04, "data/assignment_result.json")
    report_check("P4-TEST 5: Exported assignment results to data/assignment_result.json", export_ok)

    # Stress Test at Scale 1.0
    section("Phase 4 Stress Test (Scale = 1.0, Full Extreme Demand)")
    dem_10 = [{"origin": d["origin"], "destination": d["destination"], "demand": d["demand"] * 1.0} for d in raw_dem]
    aon_10 = run_aon(g4, dem_10)
    ue_fw_10 = run_frank_wolfe(g4, dem_10, system_optimal=False, max_iter=1000, target_gap=1e-3, verbose=False)
    so_fw_10 = run_frank_wolfe(g4, dem_10, system_optimal=True, max_iter=1000, target_gap=1e-3, verbose=False)

    print("\nCity Network Stress Test Comparison (Scale = 1.0, Demand = 13800 vph):")
    print(f"{'Method':<12} {'Iters':<8} {'Final RelGap':<14} {'TSTT (veh-min/h)':<18} {'TSTT (veh-hrs)':<16} {'Max V/C':<10} {'Over-Cap Edges'}")
    print("=" * 88)
    for r in [aon_10, ue_fw_10, so_fw_10]:
        print(f"{r['method']:<12} {r['iterations']:<8} {r['final_gap']:<14.6f} {r['total_travel_time']:<18.1f} {r['total_travel_time']/60.0:<16.2f} {r['max_vc']:<10.3f} {r['edges_over_capacity']}")
    print("=" * 88)

    report_check("P4-TEST 6: Stress Test SO TSTT <= UE TSTT * 1.001",
                 so_fw_10["total_travel_time"] <= ue_fw_10["total_travel_time"] * 1.001)

    print("\n" + "*" * 65)
    print(f"Test Summary: {g_tests_passed} / {g_tests_total} checks passed.")
    print("*" * 65)

    return 0 if g_tests_failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
