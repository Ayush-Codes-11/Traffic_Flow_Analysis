// =============================================================================
// web/logic.js  –  Pure algorithm and utility logic for Traffic Flow Analysis
// Compatible with both browser (window.TrafficLogic) and Node.js (module.exports)
// =============================================================================
(function(root, factory) {
    if (typeof exports === 'object' && typeof module === 'object') {
        module.exports = factory();
    } else {
        root.TrafficLogic = factory();
    }
}(typeof self !== 'undefined' ? self : this, function() {
    'use strict';

    /**
     * Map V/C ratio to status color
     *   Green:  V/C < 0.60
     *   Amber:  0.60 <= V/C <= 0.90
     *   Red:    V/C > 0.90
     *   Grey:   Closed road segment
     */
    function getVcColor(vc, isClosed) {
        if (isClosed) return '#718096'; // dashed slate grey
        if (vc < 0.60) return '#38a169'; // vivid green
        if (vc <= 0.90) return '#d69e2e'; // amber / yellow
        return '#e53e3e';                 // vivid red
    }

    /**
     * Human-readable label for V/C category
     */
    function getVcStatus(vc, isClosed) {
        if (isClosed) return 'CLOSED';
        if (vc < 0.60) return 'FREE_FLOW';
        if (vc <= 0.90) return 'MODERATE';
        return 'CONGESTED';
    }

    /**
     * Compute BPR link travel time
     * t(x) = t0 * (1 + 0.15 * (x / c)^4)
     */
    function bprTravelTime(freeFlowTime, flow, capacity) {
        if (!capacity || capacity <= 0) return freeFlowTime;
        const ratio = flow / capacity;
        return freeFlowTime * (1.0 + 0.15 * Math.pow(ratio, 4.0));
    }

    /**
     * Aggregate network KPIs from edge list
     */
    function calculateNetworkKpis(edges) {
        let tstt_min = 0.0;
        let max_vc = 0.0;
        let over_cap = 0;

        if (!edges || !Array.isArray(edges)) {
            return { tstt_min: 0, tstt_hrs: 0, max_vc: 0, over_capacity_count: 0 };
        }

        for (const e of edges) {
            const flow = e.flow || 0.0;
            const tt = e.travel_time || e.free_flow_time || 0.0;
            const vc = e.vc_ratio || (e.capacity > 0 ? flow / e.capacity : 0.0);

            tstt_min += flow * tt;
            if (vc > max_vc) max_vc = vc;
            if (vc > 1.0) over_cap++;
        }

        return {
            tstt_min: parseFloat(tstt_min.toFixed(2)),
            tstt_hrs: parseFloat((tstt_min / 60.0).toFixed(2)),
            max_vc: parseFloat(max_vc.toFixed(3)),
            over_capacity_count: over_cap
        };
    }

    /**
     * Dijkstra's Algorithm in JavaScript for real-time interactive routing
     * @param {Array} nodes - list of node objects { id, name, x, y }
     * @param {Array} edges - list of edge objects { from, to, travel_time, closed, ... }
     * @param {number} srcId - origin node id
     * @param {number} dstId - destination node id
     * @returns {Object} { found, path: [nodeIds], totalCost, edges: [edgeObjects] }
     */
    function dijkstraShortestPath(nodes, edges, srcId, dstId) {
        if (srcId === dstId) {
            return { found: true, path: [srcId], totalCost: 0.0, edges: [] };
        }

        const adj = new Map();
        for (const n of nodes) {
            adj.set(n.id, []);
        }

        for (const e of edges) {
            if (e.closed) continue; // skip closed edges
            const cost = (typeof e.travel_time === 'number') ? e.travel_time : (e.free_flow_time || 1.0);
            if (adj.has(e.from)) {
                adj.get(e.from).push({ to: e.to, cost, edgeRef: e });
            }
        }

        const dist = new Map();
        const prev = new Map();
        const prevEdge = new Map();
        const visited = new Set();

        for (const n of nodes) {
            dist.set(n.id, Infinity);
        }
        dist.set(srcId, 0.0);

        while (visited.size < nodes.length) {
            // Extract unvisited node with minimum distance
            let u = -1;
            let minDist = Infinity;
            for (const [id, d] of dist.entries()) {
                if (!visited.has(id) && d < minDist) {
                    minDist = d;
                    u = id;
                }
            }

            if (u === -1 || minDist === Infinity) break; // unreachable remaining
            if (u === dstId) break; // reached destination

            visited.add(u);

            const neighbors = adj.get(u) || [];
            for (const edge of neighbors) {
                const v = edge.to;
                if (visited.has(v)) continue;

                const alt = dist.get(u) + edge.cost;
                if (alt < dist.get(v)) {
                    dist.set(v, alt);
                    prev.set(v, u);
                    prevEdge.set(v, edge.edgeRef);
                }
            }
        }

        if (dist.get(dstId) === Infinity) {
            return { found: false, path: [], totalCost: Infinity, edges: [] };
        }

        // Reconstruct path
        const path = [];
        const pathEdges = [];
        let curr = dstId;
        while (curr !== undefined && curr !== srcId) {
            path.push(curr);
            if (prevEdge.has(curr)) {
                pathEdges.push(prevEdge.get(curr));
            }
            curr = prev.get(curr);
        }
        path.push(srcId);
        path.reverse();
        pathEdges.reverse();

        return {
            found: true,
            path: path,
            totalCost: parseFloat(dist.get(dstId).toFixed(3)),
            edges: pathEdges
        };
    }

    /**
     * Export edges list to RFC-4180 CSV string
     */
    function generateEdgeCsv(edges) {
        const headers = ['from_id', 'to_id', 'from_name', 'to_name', 'flow_vph', 'capacity_vph', 'vc_ratio', 'travel_time_min', 'status'];
        const lines = [headers.join(',')];

        for (const e of edges) {
            const row = [
                e.from,
                e.to,
                `"${(e.from_name || '').replace(/"/g, '""')}"`,
                `"${(e.to_name || '').replace(/"/g, '""')}"`,
                (e.flow !== undefined) ? e.flow.toFixed(1) : '0.0',
                (e.capacity !== undefined) ? e.capacity.toFixed(1) : '0.0',
                (e.vc_ratio !== undefined) ? e.vc_ratio.toFixed(3) : '0.000',
                (e.travel_time !== undefined) ? e.travel_time.toFixed(3) : '0.000',
                getVcStatus(e.vc_ratio || 0, !!e.closed)
            ];
            lines.push(row.join(','));
        }
        return lines.join('\n');
    }

    return {
        getVcColor,
        getVcStatus,
        bprTravelTime,
        calculateNetworkKpis,
        dijkstraShortestPath,
        generateEdgeCsv
    };
}));
