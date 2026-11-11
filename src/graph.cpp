// =============================================================================
// graph.cpp  –  Implementation of Graph, plus a simple JSON loader
// Phase 1
// =============================================================================
#include "graph.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iomanip>   // setw

// =============================================================================
// ── Tiny JSON helpers ────────────────────────────────────────────────────────
// We do NOT use an external library so that every line is explainable.
// The helpers below are enough to parse our specific sample_city.json layout.
// =============================================================================

// split_json_array
// ----------------
// Given the entire file contents and an array key (e.g. "nodes"),
// returns a vector of raw strings, one per JSON object inside that array.
//
// Algorithm:
//   1. Find "key": [
//   2. Walk character by character, tracking { } depth.
//   3. Each time depth goes from 1 → 0 we have a complete object string.
//
// Time: O(N) where N = file size
static std::vector<std::string>
split_json_array(const std::string& json, const std::string& key)
{
    std::vector<std::string> objects;

    // Step 1: locate "key": [
    std::string search_key = "\"" + key + "\"";
    size_t pos = json.find(search_key);
    if (pos == std::string::npos) return objects;

    pos = json.find('[', pos);
    if (pos == std::string::npos) return objects;
    pos++; // move past '['

    // Step 2: collect objects
    int    depth      = 0;
    size_t obj_start  = std::string::npos;

    for (size_t i = pos; i < json.size(); ++i) {
        char c = json[i];

        if (c == '{') {
            if (depth == 0) obj_start = i;   // mark start of new object
            ++depth;
        } else if (c == '}') {
            --depth;
            if (depth == 0 && obj_start != std::string::npos) {
                // We closed an object – capture it
                objects.push_back(json.substr(obj_start, i - obj_start + 1));
                obj_start = std::string::npos;
            }
        } else if (c == ']' && depth == 0) {
            break;  // end of the array
        }
    }
    return objects;
}

// get_json_value
// --------------
// Extract the raw string value of a key from a JSON object string.
// Handles both quoted strings and bare numbers.
// e.g.  get_json_value(`{"id": 5, "name": "Airport"}`, "name") → "Airport"
//       get_json_value(`{"id": 5, "name": "Airport"}`, "id")   → "5"
//
// Time: O(N) where N = object string length
static std::string
get_json_value(const std::string& obj, const std::string& key)
{
    // Look for  "key"  inside the object string
    std::string search = "\"" + key + "\"";
    size_t pos = obj.find(search);
    if (pos == std::string::npos) return "";

    // Skip to the colon
    pos = obj.find(':', pos + search.size());
    if (pos == std::string::npos) return "";
    ++pos;

    // Skip whitespace after the colon
    while (pos < obj.size() && std::isspace(static_cast<unsigned char>(obj[pos])))
        ++pos;

    if (pos >= obj.size()) return "";

    if (obj[pos] == '"') {
        // ── String value ──────────────────────────────────────────────────
        ++pos; // skip opening quote
        size_t end = obj.find('"', pos);
        if (end == std::string::npos) return "";
        return obj.substr(pos, end - pos);
    } else {
        // ── Numeric value ─────────────────────────────────────────────────
        // Read until we hit a delimiter: , } newline
        size_t end = pos;
        while (end < obj.size() &&
               obj[end] != ',' && obj[end] != '}' &&
               obj[end] != '\n' && obj[end] != '\r')
        {
            ++end;
        }
        std::string val = obj.substr(pos, end - pos);
        // Trim trailing whitespace
        while (!val.empty() && std::isspace(static_cast<unsigned char>(val.back())))
            val.pop_back();
        return val;
    }
}

// =============================================================================
// ── Graph method implementations ─────────────────────────────────────────────
// =============================================================================

void Graph::add_node(const Node& n)
{
    nodes[n.id] = n;
    // Make sure the adjacency list entry exists even if the node has no edges yet
    if (adj.find(n.id) == adj.end())
        adj[n.id] = {};
}

void Graph::add_edge(const Edge& e)
{
    adj[e.from].push_back(e);
}

// load_from_json
// --------------
// Reads sample_city.json.
// For every entry in "edges" we add TWO directed edges (bidirectional road).
// Time: O(V + E)
bool Graph::load_from_json(const std::string& filepath)
{
    // ── 1. Read entire file into a string ─────────────────────────────────
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[Graph] Cannot open file: " << filepath << "\n";
        return false;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    std::string json = ss.str();

    // ── 2. Parse nodes ────────────────────────────────────────────────────
    // Each node is parsed inside a try-catch so one bad entry does not kill
    // the whole load.  std::stoi / std::stod throw std::invalid_argument if
    // the string is empty or non-numeric, and std::out_of_range if too large.
    auto node_objects = split_json_array(json, "nodes");
    int  node_errors  = 0;

    for (const auto& obj : node_objects) {
        try {
            std::string id_str = get_json_value(obj, "id");
            std::string x_str  = get_json_value(obj, "x");
            std::string y_str  = get_json_value(obj, "y");

            // Check for missing required fields before attempting conversion
            if (id_str.empty() || x_str.empty() || y_str.empty()) {
                throw std::invalid_argument("missing required field (id, x, or y)");
            }

            Node n;
            n.id   = std::stoi(id_str);   // throws if non-numeric
            n.name = get_json_value(obj, "name");
            n.x    = std::stod(x_str);    // throws if non-numeric
            n.y    = std::stod(y_str);
            add_node(n);
        } catch (const std::exception& ex) {
            // Print a clear warning and skip this entry – do not crash
            std::cerr << "[Graph] WARNING: Skipping bad node entry: "
                      << ex.what() << "\n"
                      << "         raw: " << obj.substr(0, 80) << "\n";
            ++node_errors;
        }
    }

    // ── 3. Parse edges ────────────────────────────────────────────────────
    // Each JSON entry represents a *bidirectional* road, so we add u→v and v→u.
    auto edge_objects = split_json_array(json, "edges");
    int  edge_errors  = 0;

    for (const auto& obj : edge_objects) {
        try {
            std::string from_str = get_json_value(obj, "from");
            std::string to_str   = get_json_value(obj, "to");
            std::string len_str  = get_json_value(obj, "length");
            std::string cap_str  = get_json_value(obj, "capacity");
            std::string fft_str  = get_json_value(obj, "free_flow_time");

            if (from_str.empty() || to_str.empty() || len_str.empty() ||
                cap_str.empty()  || fft_str.empty())
            {
                throw std::invalid_argument("missing required edge field");
            }

            Edge e;
            e.from           = std::stoi(from_str);
            e.to             = std::stoi(to_str);
            e.length         = std::stod(len_str);
            e.capacity       = std::stod(cap_str);   // throws if "BAD_VALUE"
            e.free_flow_time = std::stod(fft_str);
            e.flow           = 0.0;
            e.closed         = false;

            // Sanity-check values
            if (e.capacity <= 0)
                throw std::invalid_argument("capacity must be > 0");
            if (e.free_flow_time <= 0)
                throw std::invalid_argument("free_flow_time must be > 0");

            // forward direction
            add_edge(e);
            // reverse direction (bidirectional road)
            Edge rev = e;
            std::swap(rev.from, rev.to);
            add_edge(rev);

        } catch (const std::exception& ex) {
            std::cerr << "[Graph] WARNING: Skipping bad edge entry: "
                      << ex.what() << "\n"
                      << "         raw: " << obj.substr(0, 80) << "\n";
            ++edge_errors;
        }
    }

    if (node_errors > 0 || edge_errors > 0) {
        std::cerr << "[Graph] Load completed with " << node_errors
                  << " node error(s) and " << edge_errors << " edge error(s).\n";
    }

    std::cout << "[Graph] Loaded " << node_count() << " nodes and "
              << edge_count() << " directed edges from " << filepath << "\n";

    // Return false only if the file had NO usable data at all
    return (node_count() > 0 || edge_count() > 0);
}

// print
// -----
// Dump a human-readable view of the graph.  Useful for debugging.
// Time: O(V + E)
void Graph::print() const
{
    std::cout << "\n========== GRAPH ==========\n";
    std::cout << "Nodes (" << node_count() << "):\n";
    for (const auto& [id, n] : nodes) {
        std::cout << "  [" << std::setw(2) << id << "] "
                  << std::left << std::setw(20) << n.name
                  << "  pos=(" << n.x << ", " << n.y << ")\n";
    }

    std::cout << "\nEdges (" << edge_count() << " directed):\n";
    for (const auto& [src, edges] : adj) {
        for (const auto& e : edges) {
            std::cout << "  " << std::setw(2) << e.from << " → "
                      << std::setw(2) << e.to
                      << "  len=" << std::setw(4) << e.length << " km"
                      << "  cap=" << std::setw(5) << e.capacity << " vph"
                      << "  t0=" << std::setw(4) << e.free_flow_time << " min"
                      << "  flow=" << e.flow
                      << "  bpr=" << std::fixed << std::setprecision(2)
                      << e.bpr_time() << " min"
                      << (e.closed ? "  [CLOSED]" : "")
                      << "\n";
        }
    }
    std::cout << "============================\n\n";
}

// euclidean_dist
// --------------
// Straight-line distance between nodes a and b.
// This is the admissible heuristic for A* (Phase 2).
// Time: O(1)
double Graph::euclidean_dist(int a, int b) const
{
    const Node& na = nodes.at(a);
    const Node& nb = nodes.at(b);
    double dx = na.x - nb.x;
    double dy = na.y - nb.y;
    return std::sqrt(dx * dx + dy * dy);
}

// get_bpr_time
// ------------
// Linear scan through adjacency list of 'from' to find edge to 'to'.
// Time: O(out-degree of 'from')), which is small in road networks (avg 3-4).
double Graph::get_bpr_time(int from, int to) const
{
    auto it = adj.find(from);
    if (it == adj.end()) return -1.0;
    for (const auto& e : it->second) {
        if (e.to == to) return e.bpr_time();
    }
    return -1.0;
}

// set_flow
// --------
// Update the vehicle flow on a directed edge.  Called each simulation step.
// Time: O(out-degree of 'from')
bool Graph::set_flow(int from, int to, double new_flow)
{
    auto it = adj.find(from);
    if (it == adj.end()) return false;
    for (auto& e : it->second) {
        if (e.to == to) {
            e.flow = new_flow;
            return true;
        }
    }
    return false;
}

// set_closed
// ----------
// Close or reopen a directed road segment (Phase 5: accidents, closures).
// Time: O(out-degree of 'from')
bool Graph::set_closed(int from, int to, bool closed)
{
    auto it = adj.find(from);
    if (it == adj.end()) return false;
    for (auto& e : it->second) {
        if (e.to == to) {
            e.closed = closed;
            return true;
        }
    }
    return false;
}

// edge_count
// ----------
// Count total directed edges across all adjacency lists.
// Time: O(V)
int Graph::edge_count() const
{
    int count = 0;
    for (const auto& [id, edges] : adj)
        count += static_cast<int>(edges.size());
    return count;
}
