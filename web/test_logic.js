// =============================================================================
// web/test_logic.js - Unit tests for web/logic.js using Node.js assert
// Runs in CI on GitHub Actions Ubuntu environment.
// =============================================================================
const assert = require('assert');
const Logic = require('./logic.js');

let passedTests = 0;
let totalTests = 0;

function runTest(name, fn) {
    totalTests++;
    try {
        fn();
        passedTests++;
        console.log(`[PASS] ${name}`);
    } catch (err) {
        console.error(`[FAIL] ${name}:`, err.message);
        throw err;
    }
}

console.log('--- Running Web Logic Unit Tests ---');

// 1. getVcColor tests
runTest('getVcColor returns correct hex codes', () => {
    assert.strictEqual(Logic.getVcColor(0.2, false), '#38a169', 'Free flow < 0.6 should be green');
    assert.strictEqual(Logic.getVcColor(0.59, false), '#38a169', '0.59 should be green');
    assert.strictEqual(Logic.getVcColor(0.60, false), '#d69e2e', '0.60 boundary should be amber');
    assert.strictEqual(Logic.getVcColor(0.85, false), '#d69e2e', '0.85 should be amber');
    assert.strictEqual(Logic.getVcColor(0.90, false), '#d69e2e', '0.90 boundary should be amber');
    assert.strictEqual(Logic.getVcColor(0.91, false), '#e53e3e', '0.91 should be red');
    assert.strictEqual(Logic.getVcColor(1.5, false), '#e53e3e', '1.5 over capacity should be red');
    assert.strictEqual(Logic.getVcColor(0.2, true), '#718096', 'Closed road should be slate grey');
});

// 2. getVcStatus tests
runTest('getVcStatus returns appropriate category string', () => {
    assert.strictEqual(Logic.getVcStatus(0.3, false), 'FREE_FLOW');
    assert.strictEqual(Logic.getVcStatus(0.7, false), 'MODERATE');
    assert.strictEqual(Logic.getVcStatus(1.1, false), 'CONGESTED');
    assert.strictEqual(Logic.getVcStatus(0.5, true), 'CLOSED');
});

// 3. bprTravelTime tests
runTest('bprTravelTime formula evaluation', () => {
    // Zero flow: t(0) = t0
    assert.strictEqual(Logic.bprTravelTime(10.0, 0, 1000), 10.0);
    // Flow equal to capacity: t(c) = t0 * (1 + 0.15 * 1^4) = 1.15 * t0
    const tAtCap = Logic.bprTravelTime(10.0, 1000, 1000);
    assert.ok(Math.abs(tAtCap - 11.5) < 1e-6, `Expected 11.5, got ${tAtCap}`);
    // Flow double capacity: t(2c) = t0 * (1 + 0.15 * 16) = t0 * (1 + 2.4) = 3.4 * t0
    const tAt2Cap = Logic.bprTravelTime(10.0, 2000, 1000);
    assert.ok(Math.abs(tAt2Cap - 34.0) < 1e-6, `Expected 34.0, got ${tAt2Cap}`);
    // Zero or negative capacity guard: returns freeFlowTime
    assert.strictEqual(Logic.bprTravelTime(8.0, 500, 0), 8.0);
});

// 4. calculateNetworkKpis tests
runTest('calculateNetworkKpis aggregates TSTT and over-capacity edges', () => {
    const mockEdges = [
        { from: 1, to: 2, flow: 100, travel_time: 2.0, capacity: 200, vc_ratio: 0.5 },
        { from: 2, to: 3, flow: 250, travel_time: 4.0, capacity: 200, vc_ratio: 1.25 }
    ];
    const kpis = Logic.calculateNetworkKpis(mockEdges);
    // TSTT min = 100 * 2.0 + 250 * 4.0 = 200 + 1000 = 1200 min
    assert.strictEqual(kpis.tstt_min, 1200.0);
    // TSTT hrs = 1200 / 60 = 20.0 hrs
    assert.strictEqual(kpis.tstt_hrs, 20.0);
    assert.strictEqual(kpis.max_vc, 1.25);
    assert.strictEqual(kpis.over_capacity_count, 1);
});

// 5. dijkstraShortestPath tests
runTest('dijkstraShortestPath finds optimal path and handles closures', () => {
    const nodes = [
        { id: 0, name: 'North', x: 0, y: 0 },
        { id: 1, name: 'Central', x: 10, y: 10 },
        { id: 2, name: 'East', x: 20, y: 0 },
        { id: 3, name: 'Isolated', x: 50, y: 50 }
    ];

    // Graph with two paths from 0 to 2:
    // Direct edge 0 -> 2 with cost 10.0
    // 2-hop path 0 -> 1 (cost 3.0) and 1 -> 2 (cost 4.0), total cost 7.0
    const edges = [
        { from: 0, to: 2, travel_time: 10.0, closed: false },
        { from: 0, to: 1, travel_time: 3.0, closed: false },
        { from: 1, to: 2, travel_time: 4.0, closed: false }
    ];

    // Standard shortest path: should pick 0 -> 1 -> 2 (cost 7.0)
    const res1 = Logic.dijkstraShortestPath(nodes, edges, 0, 2);
    assert.ok(res1.found, 'Path should be found');
    assert.deepStrictEqual(res1.path, [0, 1, 2]);
    assert.strictEqual(res1.totalCost, 7.0);

    // If edge 1 -> 2 is closed, should divert to direct edge 0 -> 2 (cost 10.0)
    edges[2].closed = true;
    const res2 = Logic.dijkstraShortestPath(nodes, edges, 0, 2);
    assert.ok(res2.found, 'Path should be found via alternate route');
    assert.deepStrictEqual(res2.path, [0, 2]);
    assert.strictEqual(res2.totalCost, 10.0);

    // Same node query
    const resSame = Logic.dijkstraShortestPath(nodes, edges, 0, 0);
    assert.ok(resSame.found);
    assert.deepStrictEqual(resSame.path, [0]);
    assert.strictEqual(resSame.totalCost, 0.0);

    // Unreachable node 3
    const resUnreachable = Logic.dijkstraShortestPath(nodes, edges, 0, 3);
    assert.strictEqual(resUnreachable.found, false);
    assert.deepStrictEqual(resUnreachable.path, []);
    assert.strictEqual(resUnreachable.totalCost, Infinity);
});

// 6. generateEdgeCsv tests
runTest('generateEdgeCsv formats CSV output correctly', () => {
    const mockEdges = [
        { from: 1, to: 2, from_name: 'Node 1, Main St', to_name: 'Node 2 "Hub"', flow: 500, capacity: 1000, vc_ratio: 0.5, travel_time: 2.5, closed: false }
    ];
    const csv = Logic.generateEdgeCsv(mockEdges);
    const lines = csv.split('\n');
    assert.strictEqual(lines.length, 2, 'Header + 1 data line');
    assert.ok(lines[0].includes('from_id,to_id,from_name,to_name,flow_vph'));
    // Escaped quotes test
    assert.ok(lines[1].includes('"Node 1, Main St"'));
    assert.ok(lines[1].includes('"Node 2 ""Hub"""'));
    assert.ok(lines[1].includes('FREE_FLOW'));
});

console.log(`\n========================================`);
console.log(`All ${passedTests} / ${totalTests} Web Logic unit tests passed!`);
console.log(`========================================\n`);
