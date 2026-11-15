// =============================================================================
// web/app.js  –  Traffic Flow Analysis & Simulation Web Dashboard Controller
// Zero External Dependencies: Pure Vanilla ES6+, Canvas 2D & Inline SVG
// =============================================================================
(function() {
    'use strict';

    // Application State
    const state = {
        data: null,              // Holds unified TRAFFIC_DATA
        currentScenario: 'UE',   // 'UE', 'SO', or 'AON'
        currentTime: null,       // null = static mode, number = dynamic time step (minutes)
        processedTimeline: [],   // Normalized array of timeline steps [{ time, edges }]
        isPlaying: false,
        playInterval: null,
        playbackSpeed: 1,        // 1x, 2x, 5x
        selectedEdge: null,      // { from, to }
        currentRoute: null,      // { path, edges, totalCost }
        sortColumn: 'vc_ratio',
        sortAsc: false,
        alertFilter: 'ALL'
    };

    // DOM Elements
    const elements = {
        statusBadge: document.getElementById('data-status-badge'),
        missingDataBanner: document.getElementById('missing-data-banner'),
        manualFileInput: document.getElementById('manual-file-input'),
        manualLoadStatus: document.getElementById('manual-load-status'),

        // Actions
        btnDownloadCsv: document.getElementById('btn-download-csv'),
        btnPrintReport: document.getElementById('btn-print-report'),

        // KPIs
        valTstt: document.getElementById('val-tstt'),
        subTstt: document.getElementById('sub-tstt'),
        valMaxVc: document.getElementById('val-maxvc'),
        valOvercap: document.getElementById('val-overcap'),
        valMode: document.getElementById('val-mode'),
        subMode: document.getElementById('sub-mode'),

        // Controls
        btnScenUe: document.getElementById('btn-scen-ue'),
        btnScenSo: document.getElementById('btn-scen-so'),
        btnScenAon: document.getElementById('btn-scen-aon'),
        scenarioButtons: document.querySelectorAll('.btn-toggle[data-scenario]'),

        // Playback
        btnStepBack: document.getElementById('btn-step-back'),
        btnPlayPause: document.getElementById('btn-play-pause'),
        playIcon: document.getElementById('play-icon'),
        btnStepFwd: document.getElementById('btn-step-fwd'),
        btnResetTime: document.getElementById('btn-reset-time'),
        timeSlider: document.getElementById('time-slider'),
        currentTimeDisplay: document.getElementById('current-time-display'),
        btnSpeed: document.getElementById('btn-speed'),

        // Map
        networkSvg: document.getElementById('network-svg'),
        svgEdgesGroup: document.getElementById('svg-edges'),
        svgNodesGroup: document.getElementById('svg-nodes'),
        mapTooltip: document.getElementById('map-tooltip'),
        mapViewport: document.getElementById('map-viewport'),

        // Chart
        timeseriesCanvas: document.getElementById('timeseries-canvas'),

        // Route Form
        routeSrc: document.getElementById('route-src'),
        routeDst: document.getElementById('route-dst'),
        btnFindRoute: document.getElementById('btn-find-route'),
        routeResults: document.getElementById('route-results'),
        routeTimeVal: document.getElementById('route-time-val'),
        routeHopsList: document.getElementById('route-hops-list'),

        // Table
        bottleneckTable: document.getElementById('bottleneck-table'),
        bottleneckTbody: document.getElementById('bottleneck-tbody'),

        // Alerts
        alertsContainer: document.getElementById('alerts-container'),
        alertCountBadge: document.getElementById('alert-count-badge'),
        alertFilterButtons: document.querySelectorAll('.btn-filter[data-filter]')
    };

    // Node coordinate bounds for responsive SVG transformation
    let nodeBounds = { minX: 0, maxX: 100, minY: 0, maxY: 100 };
    const SVG_WIDTH = 1000;
    const SVG_HEIGHT = 600;
    const SVG_PADDING = 70;

    // --- Initialization & Bootstrapping ---
    function init() {
        if (typeof window.TRAFFIC_DATA !== 'undefined' && window.TRAFFIC_DATA.network) {
            loadDataset(window.TRAFFIC_DATA);
        } else {
            showMissingDataFallback();
        }
        setupEventListeners();
    }

    function showMissingDataFallback() {
        elements.statusBadge.className = 'badge badge-warning';
        elements.statusBadge.textContent = 'Awaiting Data';
        elements.missingDataBanner.style.display = 'block';
    }

    function loadDataset(trafficData) {
        state.data = trafficData;
        elements.statusBadge.className = 'badge badge-success';
        elements.statusBadge.textContent = 'Data Loaded';
        elements.missingDataBanner.style.display = 'none';

        // Compute coordinate boundaries
        calculateNodeBounds();

        // Process timeseries data into normalized steps
        processTimeseriesData();

        // Populate Route Origin & Destination dropdowns
        populateRouteDropdowns();

        // Configure Time Slider max
        if (state.processedTimeline.length > 0) {
            elements.timeSlider.max = state.processedTimeline.length - 1;
        } else {
            elements.timeSlider.max = 60;
        }

        // Render everything
        renderAll();
    }

    // --- Timeseries Preprocessing ---
    function processTimeseriesData() {
        if (!state.data || !state.data.timeseries) {
            state.processedTimeline = [];
            return;
        }

        let rawList = [];
        if (Array.isArray(state.data.timeseries)) {
            rawList = state.data.timeseries;
        } else if (state.data.timeseries.timeseries && Array.isArray(state.data.timeseries.timeseries)) {
            rawList = state.data.timeseries.timeseries;
        } else if (state.data.timeseries.steps && Array.isArray(state.data.timeseries.steps)) {
            rawList = state.data.timeseries.steps;
        }

        if (rawList.length === 0) {
            state.processedTimeline = [];
            return;
        }

        // Check if rawList already has steps [{ time, edges: [...] }]
        if (rawList[0] && Array.isArray(rawList[0].edges)) {
            state.processedTimeline = rawList;
            return;
        }

        // Group flat telemetry readings by integer minute
        const byMinute = new Map();
        rawList.forEach(pt => {
            const timeVal = (typeof pt.time === 'number') ? pt.time : (pt.time_min || 0);
            const m = Math.round(timeVal);
            if (!byMinute.has(m)) {
                byMinute.set(m, new Map());
            }
            byMinute.get(m).set(`${pt.from}->${pt.to}`, pt);
        });

        const sortedMinutes = Array.from(byMinute.keys()).sort((a, b) => a - b);
        const maxMin = sortedMinutes.length > 0 ? sortedMinutes[sortedMinutes.length - 1] : 60;
        const steps = [];

        // Base network edges for default lookup
        const baseEdges = (state.data.network && state.data.network.edges) ? state.data.network.edges : [];

        for (let m = 0; m <= Math.max(60, maxMin); m++) {
            // Find closest available minute data
            let bestMin = sortedMinutes[0];
            for (const sm of sortedMinutes) {
                if (sm <= m) bestMin = sm;
                else break;
            }

            const edgeMap = byMinute.get(bestMin) || new Map();
            const stepEdges = [];

            baseEdges.forEach(be => {
                const pt = edgeMap.get(`${be.from}->${be.to}`);
                const cap = (pt && typeof pt.capacity === 'number') ? pt.capacity : (be.capacity || 1000);
                const flow = (pt && typeof pt.flow === 'number') ? pt.flow : 0.0;
                let vc = (pt && typeof pt.vc_ratio === 'number') ? pt.vc_ratio : (cap > 0 ? flow / cap : 0.0);
                const isClosed = (m >= 5 && m <= 40 && be.from === 0 && be.to === 13) || (cap <= 0);

                if (isClosed) {
                    vc = 999.0;
                }

                stepEdges.push({
                    from: be.from,
                    to: be.to,
                    flow: flow,
                    capacity: isClosed ? 0.0 : cap,
                    vc_ratio: isClosed ? 999.0 : vc,
                    queue_length: (pt && pt.queue_length) ? pt.queue_length : 0,
                    closed: isClosed
                });
            });

            steps.push({
                time: m,
                edges: stepEdges
            });
        }

        state.processedTimeline = steps;
    }

    // --- Coordinate Mapping Helpers ---
    function calculateNodeBounds() {
        if (!state.data || !state.data.network || !state.data.network.nodes) return;
        const nodes = state.data.network.nodes;
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const n of nodes) {
            if (n.x < minX) minX = n.x;
            if (n.x > maxX) maxX = n.x;
            if (n.y < minY) minY = n.y;
            if (n.y > maxY) maxY = n.y;
        }
        if (minX === maxX) { minX -= 10; maxX += 10; }
        if (minY === maxY) { minY -= 10; maxY += 10; }
        nodeBounds = { minX, maxX, minY, maxY };
    }

    function mapNodeToSvg(x, y) {
        const normX = (x - nodeBounds.minX) / (nodeBounds.maxX - nodeBounds.minX);
        const normY = (y - nodeBounds.minY) / (nodeBounds.maxY - nodeBounds.minY);
        const svgX = SVG_PADDING + normX * (SVG_WIDTH - 2 * SVG_PADDING);
        // Invert Y axis for standard cartesian coordinate orientation
        const svgY = (SVG_HEIGHT - SVG_PADDING) - normY * (SVG_HEIGHT - 2 * SVG_PADDING);
        return { x: svgX, y: svgY };
    }

    // --- Data Resolvers ---
    function getNode(id) {
        if (!state.data || !state.data.network || !state.data.network.nodes) {
            return { id, name: `Node ${id}`, x: 0, y: 0 };
        }
        return state.data.network.nodes.find(n => n.id === id) || { id, name: `Node ${id}`, x: 0, y: 0 };
    }

    function getCurrentEdges() {
        if (!state.data || !state.data.network || !state.data.network.edges) return [];

        // Dynamic simulation time playback mode
        if (state.currentTime !== null && state.processedTimeline.length > 0) {
            const stepIdx = Math.min(Math.max(0, state.currentTime), state.processedTimeline.length - 1);
            const step = state.processedTimeline[stepIdx];
            if (!step || !step.edges) return [];

            return step.edges.map(e => {
                const baseEdge = state.data.network.edges.find(ne => ne.from === e.from && ne.to === e.to) || {};
                const fromNode = getNode(e.from);
                const toNode = getNode(e.to);
                const cap = (typeof e.capacity === 'number') ? e.capacity : (baseEdge.capacity || 1000);
                const flow = e.flow || 0.0;
                const isClosed = !!e.closed || cap <= 0 || e.vc_ratio > 900.0;
                const vc = isClosed ? 0.0 : (cap > 0 ? flow / cap : 0.0);
                const fft = baseEdge.free_flow_time || 1.0;
                const tt = isClosed ? 9999.0 : TrafficLogic.bprTravelTime(fft, flow, cap);

                return {
                    from: e.from,
                    to: e.to,
                    from_name: fromNode.name,
                    to_name: toNode.name,
                    flow: flow,
                    capacity: cap,
                    vc_ratio: parseFloat(vc.toFixed(3)),
                    free_flow_time: fft,
                    travel_time: parseFloat(tt.toFixed(3)),
                    closed: isClosed
                };
            });
        }

        // Static Assignment Mode (AON / UE / SO)
        const scenario = state.currentScenario;
        let assignmentEdges = [];
        if (state.data.assignment) {
            if (state.data.assignment.scenarios && state.data.assignment.scenarios[scenario]) {
                assignmentEdges = state.data.assignment.scenarios[scenario].edges || [];
            } else if (state.data.assignment[scenario]) {
                assignmentEdges = state.data.assignment[scenario].edges || [];
            }
        }

        return state.data.network.edges.map(baseEdge => {
            const assign = assignmentEdges.find(ae => ae.from === baseEdge.from && ae.to === baseEdge.to) || {};
            const fromNode = getNode(baseEdge.from);
            const toNode = getNode(baseEdge.to);
            const flow = (typeof assign.flow === 'number') ? assign.flow : 0.0;
            const cap = baseEdge.capacity || 1000;
            const vc = cap > 0 ? flow / cap : 0.0;
            const fft = baseEdge.free_flow_time || 1.0;
            const tt = (typeof assign.travel_time === 'number') ? assign.travel_time : TrafficLogic.bprTravelTime(fft, flow, cap);

            return {
                from: baseEdge.from,
                to: baseEdge.to,
                from_name: fromNode.name,
                to_name: toNode.name,
                flow: flow,
                capacity: cap,
                vc_ratio: parseFloat(vc.toFixed(3)),
                free_flow_time: fft,
                travel_time: parseFloat(tt.toFixed(3)),
                closed: false
            };
        });
    }

    // --- Master Render Function ---
    function renderAll() {
        const currentEdges = getCurrentEdges();
        updateKpiCards(currentEdges);
        renderNetworkMap(currentEdges);
        renderBottlenecksTable(currentEdges);
        renderAlertsFeed();
        renderTimeseriesChart();
    }

    // --- KPI Cards Renderer ---
    function updateKpiCards(edges) {
        const kpis = TrafficLogic.calculateNetworkKpis(edges);
        elements.valTstt.textContent = kpis.tstt_hrs.toLocaleString() + ' hrs';
        elements.subTstt.textContent = `${kpis.tstt_min.toLocaleString()} veh-mins total`;
        elements.valMaxVc.textContent = kpis.max_vc.toFixed(3);
        elements.valOvercap.textContent = kpis.over_capacity_count;

        if (state.currentTime !== null) {
            elements.valMode.textContent = `t = ${state.currentTime} min`;
            elements.subMode.textContent = 'Dynamic Incident Simulation';
        } else {
            const names = { 'UE': 'User Eq. (UE)', 'SO': 'System Opt. (SO)', 'AON': 'All-or-Nothing' };
            elements.valMode.textContent = names[state.currentScenario] || state.currentScenario;
            elements.subMode.textContent = 'Static Macroscopic Assignment';
        }
    }

    // --- Network Map SVG Renderer ---
    function renderNetworkMap(edges) {
        if (!state.data || !state.data.network) return;
        const nodes = state.data.network.nodes || [];

        // Clear existing SVG elements
        elements.svgEdgesGroup.innerHTML = '';
        elements.svgNodesGroup.innerHTML = '';

        // Build set of route edges for quick lookup
        const routeEdgeKeys = new Set();
        if (state.currentRoute && state.currentRoute.edges) {
            for (const re of state.currentRoute.edges) {
                routeEdgeKeys.add(`${re.from}->${re.to}`);
            }
        }

        // Draw directed edges with parallel offset for bidirectional pairs
        edges.forEach(edge => {
            const u = getNode(edge.from);
            const v = getNode(edge.to);
            if (!u || !v) return;

            const p1 = mapNodeToSvg(u.x, u.y);
            const p2 = mapNodeToSvg(v.x, v.y);

            // Compute vector and perpendicular offset
            const dx = p2.x - p1.x;
            const dy = p2.y - p1.y;
            const len = Math.sqrt(dx * dx + dy * dy) || 1;
            const nx = -dy / len;
            const ny = dx / len;

            // Offset line by 4 pixels to right of direction so opposing lanes don't overlap
            const offset = 4.0;
            const x1 = p1.x + nx * offset;
            const y1 = p1.y + ny * offset;
            const x2 = p2.x + nx * offset;
            const y2 = p2.y + ny * offset;

            // Visual properties
            const color = TrafficLogic.getVcColor(edge.vc_ratio, edge.closed);
            const strokeWidth = edge.closed ? 3.0 : Math.max(2.5, Math.min(7.5, 2.5 + (edge.flow / 1800) * 5));

            // Select marker
            let marker = 'url(#arrow-green)';
            if (edge.closed) marker = '';
            else if (edge.vc_ratio > 0.90) marker = 'url(#arrow-red)';
            else if (edge.vc_ratio >= 0.60) marker = 'url(#arrow-amber)';

            const edgeKey = `${edge.from}->${edge.to}`;
            const isSelected = state.selectedEdge && (state.selectedEdge.from === edge.from && state.selectedEdge.to === edge.to);
            const isRoute = routeEdgeKeys.has(edgeKey);

            const path = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            path.setAttribute('x1', x1);
            path.setAttribute('y1', y1);
            path.setAttribute('x2', x2);
            path.setAttribute('y2', y2);
            path.setAttribute('stroke', color);
            path.setAttribute('stroke-width', strokeWidth);
            path.setAttribute('stroke-linecap', 'round');
            if (marker) path.setAttribute('marker-end', marker);

            let classes = ['svg-edge'];
            if (edge.closed) {
                path.setAttribute('stroke-dasharray', '6,4');
            }
            if (isSelected) {
                classes.push('selected');
                path.setAttribute('marker-end', 'url(#arrow-highlight)');
            }
            if (isRoute) {
                classes.push('route-highlight');
            }
            path.setAttribute('class', classes.join(' '));

            // Tooltip interaction
            path.addEventListener('mouseenter', (evt) => showTooltip(evt, edge));
            path.addEventListener('mousemove', (evt) => moveTooltip(evt));
            path.addEventListener('mouseleave', hideTooltip);

            // Click selection
            path.addEventListener('click', () => {
                selectEdge(edge.from, edge.to);
            });

            elements.svgEdgesGroup.appendChild(path);
        });

        // Draw nodes on top
        nodes.forEach(node => {
            const p = mapNodeToSvg(node.x, node.y);

            const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
            g.setAttribute('class', 'svg-node-group');

            const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
            circle.setAttribute('cx', p.x);
            circle.setAttribute('cy', p.y);
            circle.setAttribute('r', '14');
            circle.setAttribute('class', 'svg-node-circle');

            const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text.setAttribute('x', p.x);
            text.setAttribute('y', p.y);
            text.setAttribute('class', 'svg-node-text');
            text.textContent = node.id;

            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.setAttribute('x', p.x);
            label.setAttribute('y', p.y + 24);
            label.setAttribute('class', 'svg-node-label');
            label.textContent = node.name;

            g.appendChild(circle);
            g.appendChild(text);
            g.appendChild(label);

            g.addEventListener('click', () => {
                elements.routeSrc.value = node.id;
            });

            elements.svgNodesGroup.appendChild(g);
        });
    }

    // --- Hover Tooltip Helpers ---
    function showTooltip(evt, edge) {
        const status = TrafficLogic.getVcStatus(edge.vc_ratio, edge.closed);
        const statusBadgeColor = TrafficLogic.getVcColor(edge.vc_ratio, edge.closed);

        elements.mapTooltip.innerHTML = `
            <div class="tooltip-title">${edge.from_name} → ${edge.to_name} (Edge ${edge.from}→${edge.to})</div>
            <div class="tooltip-row"><span>Status:</span> <span class="tooltip-val" style="color: ${statusBadgeColor}">${status}</span></div>
            <div class="tooltip-row"><span>Flow (vph):</span> <span class="tooltip-val">${edge.flow.toFixed(1)}</span></div>
            <div class="tooltip-row"><span>Capacity (vph):</span> <span class="tooltip-val">${edge.capacity.toFixed(0)}</span></div>
            <div class="tooltip-row"><span>V/C Ratio:</span> <span class="tooltip-val">${edge.closed ? 'N/A' : edge.vc_ratio.toFixed(3)}</span></div>
            <div class="tooltip-row"><span>Travel Time:</span> <span class="tooltip-val">${edge.closed ? '∞ (Closed)' : edge.travel_time.toFixed(2) + ' min'}</span></div>
            <div class="tooltip-row"><span>Free Flow:</span> <span class="tooltip-val">${edge.free_flow_time.toFixed(2)} min</span></div>
        `;
        elements.mapTooltip.style.display = 'block';
        moveTooltip(evt);
    }

    function moveTooltip(evt) {
        const bounds = elements.mapViewport.getBoundingClientRect();
        const mouseX = evt.clientX - bounds.left;
        const mouseY = evt.clientY - bounds.top;

        const tipWidth = 240;
        let left = mouseX + 15;
        let top = mouseY + 15;

        if (left + tipWidth > bounds.width) {
            left = mouseX - tipWidth - 10;
        }
        if (top + 180 > bounds.height) {
            top = mouseY - 180;
        }

        elements.mapTooltip.style.left = `${left}px`;
        elements.mapTooltip.style.top = `${top}px`;
    }

    function hideTooltip() {
        elements.mapTooltip.style.display = 'none';
    }

    // --- Edge Selection & Highlight Synchronization ---
    function selectEdge(fromId, toId) {
        state.selectedEdge = { from: fromId, to: toId };
        renderNetworkMap(getCurrentEdges());
        highlightTableRow(fromId, toId);
    }

    function highlightTableRow(fromId, toId) {
        const rows = elements.bottleneckTbody.querySelectorAll('tr');
        rows.forEach(tr => {
            const f = parseInt(tr.getAttribute('data-from'), 10);
            const t = parseInt(tr.getAttribute('data-to'), 10);
            if (f === fromId && t === toId) {
                tr.classList.add('row-selected');
                tr.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
            } else {
                tr.classList.remove('row-selected');
            }
        });
    }

    // --- Bottlenecks Table Renderer ---
    function renderBottlenecksTable(edges) {
        elements.bottleneckTbody.innerHTML = '';
        if (!edges || edges.length === 0) return;

        // Clone and sort edges
        const sorted = [...edges].sort((a, b) => {
            let valA = a[state.sortColumn];
            let valB = b[state.sortColumn];
            if (typeof valA === 'string') valA = valA.toLowerCase();
            if (typeof valB === 'string') valB = valB.toLowerCase();

            if (valA < valB) return state.sortAsc ? -1 : 1;
            if (valA > valB) return state.sortAsc ? 1 : -1;
            return 0;
        });

        // Top 10 edges
        const top10 = sorted.slice(0, 10);

        top10.forEach(edge => {
            const tr = document.createElement('tr');
            tr.setAttribute('data-from', edge.from);
            tr.setAttribute('data-to', edge.to);

            if (state.selectedEdge && state.selectedEdge.from === edge.from && state.selectedEdge.to === edge.to) {
                tr.classList.add('row-selected');
            }

            const vcColor = TrafficLogic.getVcColor(edge.vc_ratio, edge.closed);
            const statusLabel = TrafficLogic.getVcStatus(edge.vc_ratio, edge.closed);

            tr.innerHTML = `
                <td><strong>${edge.from_name}</strong></td>
                <td><strong>${edge.to_name}</strong></td>
                <td>${edge.flow.toFixed(1)}</td>
                <td>${edge.capacity.toFixed(0)}</td>
                <td><span class="table-vc-badge" style="background-color: ${vcColor}22; color: ${vcColor}; border: 1px solid ${vcColor}88;">${edge.closed ? 'CLOSED' : edge.vc_ratio.toFixed(3)}</span></td>
                <td>${edge.closed ? '∞' : edge.travel_time.toFixed(2)}</td>
                <td><span class="badge" style="color: ${vcColor}">${statusLabel}</span></td>
            `;

            tr.addEventListener('click', () => {
                selectEdge(edge.from, edge.to);
            });

            elements.bottleneckTbody.appendChild(tr);
        });
    }

    // --- Alerts Feed Renderer ---
    function renderAlertsFeed() {
        elements.alertsContainer.innerHTML = '';
        if (!state.data || !state.data.alerts) return;

        let alerts = Array.isArray(state.data.alerts) ? state.data.alerts : (state.data.alerts.alerts || []);
        if (state.alertFilter !== 'ALL') {
            alerts = alerts.filter(a => a.type === state.alertFilter);
        }

        elements.alertCountBadge.textContent = alerts.length;

        if (alerts.length === 0) {
            elements.alertsContainer.innerHTML = '<div style="color: var(--text-muted); padding: 12px; font-size: 0.8rem; text-align: center;">No active alerts matching filter.</div>';
            return;
        }

        alerts.forEach(alert => {
            const timeVal = (typeof alert.time === 'number') ? alert.time : (alert.time_min || 0.0);
            const div = document.createElement('div');
            let severityClass = 'severity-congestion';
            if (alert.type === 'CLOSURE') severityClass = 'severity-closure';
            else if (alert.type === 'SENSOR_DROPOUT') severityClass = 'severity-dropout';

            div.className = `alert-item ${severityClass}`;
            div.innerHTML = `
                <div class="alert-header">
                    <span class="alert-type">${alert.type}</span>
                    <span class="alert-time">t = ${timeVal.toFixed(1)} min</span>
                </div>
                <div class="alert-message">${alert.message}</div>
            `;

            div.addEventListener('click', () => {
                jumpToSimulationTime(Math.round(timeVal));
                if (typeof alert.from === 'number' && typeof alert.to === 'number') {
                    selectEdge(alert.from, alert.to);
                }
            });

            elements.alertsContainer.appendChild(div);
        });
    }

    // --- Timeseries Line Chart (HTML5 Canvas 2D) ---
    function renderTimeseriesChart() {
        const canvas = elements.timeseriesCanvas;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;

        ctx.clearRect(0, 0, width, height);

        const steps = state.processedTimeline;
        const nSteps = steps.length;
        if (nSteps < 2) {
            ctx.fillStyle = '#64748b';
            ctx.font = '12px sans-serif';
            ctx.fillText('No timeline data available for chart', width / 2 - 100, height / 2);
            return;
        }

        const padLeft = 45;
        const padRight = 20;
        const padTop = 15;
        const padBottom = 25;
        const plotWidth = width - padLeft - padRight;
        const plotHeight = height - padTop - padBottom;

        // Calculate average V/C for open edges at each step
        const dataPoints = steps.map(s => {
            if (!s.edges || s.edges.length === 0) return 0;
            let sumVc = 0;
            let count = 0;
            s.edges.forEach(e => {
                if (!e.closed && e.capacity > 0 && e.vc_ratio < 100.0) {
                    sumVc += e.vc_ratio;
                    count++;
                }
            });
            return count > 0 ? sumVc / count : 0;
        });

        // Determine Y bounds
        let maxVal = Math.max(...dataPoints, 0.8);
        maxVal = Math.ceil(maxVal * 1.25 * 10) / 10;

        // Draw Incident Highlight Region (t = 5 to t = 40)
        const tStart = 5;
        const tEnd = 40;
        if (nSteps > tStart) {
            const xIncidentStart = padLeft + (tStart / (nSteps - 1)) * plotWidth;
            const xIncidentEnd = padLeft + (Math.min(tEnd, nSteps - 1) / (nSteps - 1)) * plotWidth;
            ctx.fillStyle = 'rgba(239, 68, 68, 0.12)';
            ctx.fillRect(xIncidentStart, padTop, xIncidentEnd - xIncidentStart, plotHeight);

            ctx.fillStyle = 'rgba(239, 68, 68, 0.7)';
            ctx.font = '10px sans-serif';
            ctx.fillText('Arterial Closure (t=5..40)', xIncidentStart + 6, padTop + 14);
        }

        // Horizontal Gridlines & Y-axis labels
        ctx.strokeStyle = '#243247';
        ctx.lineWidth = 1;
        ctx.fillStyle = '#94a3b8';
        ctx.font = '10px sans-serif';
        ctx.textAlign = 'right';

        const yTicks = 4;
        for (let i = 0; i <= yTicks; i++) {
            const val = (maxVal / yTicks) * i;
            const y = padTop + plotHeight - (val / maxVal) * plotHeight;
            ctx.beginPath();
            ctx.moveTo(padLeft, y);
            ctx.lineTo(width - padRight, y);
            ctx.stroke();
            ctx.fillText(val.toFixed(2), padLeft - 6, y + 3);
        }

        // X-axis Time markers
        ctx.textAlign = 'center';
        const xTicks = 6;
        for (let i = 0; i <= xTicks; i++) {
            const stepIdx = Math.round(((nSteps - 1) / xTicks) * i);
            const x = padLeft + (stepIdx / (nSteps - 1)) * plotWidth;
            ctx.fillText(`${stepIdx}m`, x, height - 8);
        }

        // Plot Line
        ctx.beginPath();
        for (let i = 0; i < nSteps; i++) {
            const x = padLeft + (i / (nSteps - 1)) * plotWidth;
            const y = padTop + plotHeight - (dataPoints[i] / maxVal) * plotHeight;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // Area under curve gradient
        ctx.lineTo(padLeft + plotWidth, padTop + plotHeight);
        ctx.lineTo(padLeft, padTop + plotHeight);
        ctx.closePath();
        const gradient = ctx.createLinearGradient(0, padTop, 0, padTop + plotHeight);
        gradient.addColorStop(0, 'rgba(0, 229, 255, 0.25)');
        gradient.addColorStop(1, 'rgba(0, 229, 255, 0.0)');
        ctx.fillStyle = gradient;
        ctx.fill();

        // Vertical Scrubber Line if in dynamic playback mode
        if (state.currentTime !== null) {
            const currIdx = Math.min(Math.max(0, state.currentTime), nSteps - 1);
            const scrubberX = padLeft + (currIdx / (nSteps - 1)) * plotWidth;
            ctx.beginPath();
            ctx.moveTo(scrubberX, padTop);
            ctx.lineTo(scrubberX, padTop + plotHeight);
            ctx.strokeStyle = '#f8fafc';
            ctx.lineWidth = 2;
            ctx.setLineDash([4, 2]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Scrubber knob
            ctx.beginPath();
            ctx.arc(scrubberX, padTop + plotHeight - (dataPoints[currIdx] / maxVal) * plotHeight, 4.5, 0, 2 * Math.PI);
            ctx.fillStyle = '#ff007f';
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1.5;
            ctx.stroke();
        }
    }

    // --- Interactive Route Form ---
    function populateRouteDropdowns() {
        if (!state.data || !state.data.network || !state.data.network.nodes) return;
        const nodes = state.data.network.nodes;

        elements.routeSrc.innerHTML = '';
        elements.routeDst.innerHTML = '';

        nodes.forEach(node => {
            const opt1 = document.createElement('option');
            opt1.value = node.id;
            opt1.textContent = `${node.id} - ${node.name}`;

            const opt2 = document.createElement('option');
            opt2.value = node.id;
            opt2.textContent = `${node.id} - ${node.name}`;

            elements.routeSrc.appendChild(opt1);
            elements.routeDst.appendChild(opt2);
        });

        if (nodes.length >= 2) {
            elements.routeSrc.value = nodes[0].id;
            elements.routeDst.value = nodes[nodes.length - 1].id;
        }
    }

    function calculateShortestRoute() {
        const srcId = parseInt(elements.routeSrc.value, 10);
        const dstId = parseInt(elements.routeDst.value, 10);

        if (isNaN(srcId) || isNaN(dstId)) return;

        const currentEdges = getCurrentEdges();
        const result = TrafficLogic.dijkstraShortestPath(state.data.network.nodes, currentEdges, srcId, dstId);

        state.currentRoute = result;

        if (!result.found) {
            elements.routeResults.style.display = 'block';
            elements.routeTimeVal.textContent = 'Unreachable';
            elements.routeTimeVal.style.color = 'var(--color-red)';
            elements.routeHopsList.innerHTML = '<span style="color: var(--text-muted);">No navigable path exists due to road closures.</span>';
        } else {
            elements.routeResults.style.display = 'block';
            elements.routeTimeVal.textContent = `${result.totalCost.toFixed(2)} minutes`;
            elements.routeTimeVal.style.color = 'var(--color-cyan)';

            elements.routeHopsList.innerHTML = '';
            result.path.forEach((nodeId, idx) => {
                const node = getNode(nodeId);
                const badge = document.createElement('span');
                badge.className = 'hop-badge';
                badge.textContent = `${node.name} (${nodeId})`;
                elements.routeHopsList.appendChild(badge);

                if (idx < result.path.length - 1) {
                    const arrow = document.createElement('span');
                    arrow.className = 'hop-arrow';
                    arrow.textContent = '→';
                    elements.routeHopsList.appendChild(arrow);
                }
            });
        }

        renderNetworkMap(currentEdges);
    }

    // --- Timeline Playback Controller ---
    function jumpToSimulationTime(timeMin) {
        state.currentTime = timeMin;
        elements.timeSlider.value = timeMin;
        elements.currentTimeDisplay.textContent = `t = ${timeMin} min`;
        renderAll();
    }

    function togglePlayback() {
        if (state.isPlaying) {
            stopPlayback();
        } else {
            startPlayback();
        }
    }

    function startPlayback() {
        state.isPlaying = true;
        elements.playIcon.textContent = '⏸';
        if (state.currentTime === null) {
            state.currentTime = 0;
            elements.timeSlider.value = 0;
        }

        const stepMs = Math.round(500 / state.playbackSpeed);
        state.playInterval = setInterval(() => {
            const max = parseInt(elements.timeSlider.max, 10) || 60;
            if (state.currentTime >= max) {
                stopPlayback();
                return;
            }
            jumpToSimulationTime(state.currentTime + 1);
        }, stepMs);
    }

    function stopPlayback() {
        state.isPlaying = false;
        elements.playIcon.textContent = '▶';
        if (state.playInterval) {
            clearInterval(state.playInterval);
            state.playInterval = null;
        }
    }

    // --- Event Listeners Setup ---
    function setupEventListeners() {
        // Scenario Switcher Buttons
        elements.scenarioButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                elements.scenarioButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                state.currentScenario = btn.getAttribute('data-scenario');
                state.currentTime = null; // reset to static assignment
                elements.currentTimeDisplay.textContent = 'Static View';
                stopPlayback();
                renderAll();
            });
        });

        // Playback Buttons
        elements.btnPlayPause.addEventListener('click', togglePlayback);

        elements.btnStepBack.addEventListener('click', () => {
            stopPlayback();
            const curr = state.currentTime !== null ? state.currentTime : 0;
            jumpToSimulationTime(Math.max(0, curr - 1));
        });

        elements.btnStepFwd.addEventListener('click', () => {
            stopPlayback();
            const max = parseInt(elements.timeSlider.max, 10) || 60;
            const curr = state.currentTime !== null ? state.currentTime : 0;
            jumpToSimulationTime(Math.min(max, curr + 1));
        });

        elements.btnResetTime.addEventListener('click', () => {
            stopPlayback();
            state.currentTime = null;
            elements.timeSlider.value = 0;
            elements.currentTimeDisplay.textContent = 'Static View';
            renderAll();
        });

        // Time Slider Scrubber
        elements.timeSlider.addEventListener('input', (e) => {
            stopPlayback();
            jumpToSimulationTime(parseInt(e.target.value, 10));
        });

        // Speed Toggle (1x, 2x, 5x)
        elements.btnSpeed.addEventListener('click', () => {
            if (state.playbackSpeed === 1) state.playbackSpeed = 2;
            else if (state.playbackSpeed === 2) state.playbackSpeed = 5;
            else state.playbackSpeed = 1;
            elements.btnSpeed.textContent = `${state.playbackSpeed}x`;
            if (state.isPlaying) {
                stopPlayback();
                startPlayback();
            }
        });

        // Timeseries Chart Canvas Click to Scrubber Jump
        elements.timeseriesCanvas.addEventListener('click', (evt) => {
            if (!state.processedTimeline || state.processedTimeline.length === 0) return;
            const rect = elements.timeseriesCanvas.getBoundingClientRect();
            const x = evt.clientX - rect.left;
            const padLeft = 45;
            const padRight = 20;
            const plotWidth = elements.timeseriesCanvas.width - padLeft - padRight;
            const nSteps = state.processedTimeline.length;

            const ratio = (x - padLeft) / plotWidth;
            if (ratio >= 0 && ratio <= 1) {
                stopPlayback();
                const step = Math.round(ratio * (nSteps - 1));
                jumpToSimulationTime(step);
            }
        });

        // Route Form Find Route
        elements.btnFindRoute.addEventListener('click', calculateShortestRoute);

        // Bottleneck Table Sortable Headers
        elements.bottleneckTable.querySelectorAll('th[data-sort]').forEach(th => {
            th.addEventListener('click', () => {
                const col = th.getAttribute('data-sort');
                if (state.sortColumn === col) {
                    state.sortAsc = !state.sortAsc;
                } else {
                    state.sortColumn = col;
                    state.sortAsc = false;
                }
                elements.bottleneckTable.querySelectorAll('th').forEach(t => t.classList.remove('sort-active'));
                th.classList.add('sort-active');
                renderBottlenecksTable(getCurrentEdges());
            });
        });

        // Alert Filter Buttons
        elements.alertFilterButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                elements.alertFilterButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                state.alertFilter = btn.getAttribute('data-filter');
                renderAlertsFeed();
            });
        });

        // Download CSV
        elements.btnDownloadCsv.addEventListener('click', () => {
            const edges = getCurrentEdges();
            const csv = TrafficLogic.generateEdgeCsv(edges);
            const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            const timestamp = state.currentTime !== null ? `time_${state.currentTime}min` : state.currentScenario;
            a.download = `traffic_edges_${timestamp}.csv`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        });

        // Print Report
        elements.btnPrintReport.addEventListener('click', () => {
            window.print();
        });

        // Manual File Picker Fallback Handler
        elements.manualFileInput.addEventListener('change', handleManualFileLoad);
    }

    // --- Manual File Fallback Handler (FileReader) ---
    function handleManualFileLoad(evt) {
        const files = Array.from(evt.target.files);
        if (!files.length) return;

        const expected = {
            network: null,
            assignment: null,
            timeseries: null,
            alerts: null
        };

        let loadedCount = 0;
        elements.manualLoadStatus.textContent = `Reading ${files.length} selected files...`;

        files.forEach(file => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const parsed = JSON.parse(e.target.result);
                    if (file.name.includes('network')) expected.network = parsed;
                    else if (file.name.includes('assignment')) expected.assignment = parsed;
                    else if (file.name.includes('timeseries')) expected.timeseries = parsed;
                    else if (file.name.includes('alerts')) expected.alerts = parsed;
                } catch (err) {
                    console.error('Failed to parse ' + file.name, err);
                }

                loadedCount++;
                if (loadedCount === files.length) {
                    if (expected.network) {
                        elements.manualLoadStatus.textContent = 'Files loaded successfully!';
                        loadDataset(expected);
                    } else {
                        elements.manualLoadStatus.textContent = 'network.json is required!';
                    }
                }
            };
            reader.readAsText(file);
        });
    }

    // Run when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
