#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <queue>
#include <limits>
#include <cmath>

using namespace std;

class Graph {
public:
    struct Node {
        string name;
        int id;
        bool is_active{true};
    };

    struct Edge {
        int from;
        int to;
        double distance{0.0};
        bool is_active{true};
    };

private:
    vector<Node> nodes;

    vector<vector<Edge>> adj;

    int find_node_index(int id) const noexcept {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].id == id) return int(i);
        }
        return -1;
    }

public:
    Graph() = default;

    void add_node(const string& name, int id, bool is_active = true) {
        if (find_node_index(id) != -1) {
            cout<<("add_node: node id already exists");
        }
        nodes.push_back(Node{ name, id, is_active });
        adj.push_back(vector<Edge>());
    }

    void connect(int from_id, int to_id, double distance, bool is_active = true) {
        if (distance < 0.0) {
            cout<<("connect: distance must be non-negative");
        }
        int fi = find_node_index(from_id);
        if (fi == -1) cout<<("connect: from node does not exist");
        int ti = find_node_index(to_id);
        if (ti == -1) cout<<("connect: to node does not exist");

        auto &edges = adj[fi];
        for (auto &e : edges) {
            if (e.to == to_id) {
                e.distance = distance;
                e.is_active = is_active;
                return;
            }
        }

        edges.push_back(Edge{ from_id, to_id, distance, is_active });
    }

    void enable_node(int id) {
        int idx = find_node_index(id);
        if (idx == -1) cout<<("enable_node: node not found");
        nodes[idx].is_active = true;
    }

    void disable_node(int id) {
        int idx = find_node_index(id);
        if (idx == -1) cout<<("disable_node: node not found");
        nodes[idx].is_active = false;
    }

    void enable_edge(int from_id, int to_id) {
        int fi = find_node_index(from_id);
        if (fi == -1) cout<<("enable_edge: from node not found");
        auto &edges = adj[fi];
        for (auto &e : edges) {
            if (e.to == to_id) {
                e.is_active = true;
                return;
            }
        }
        cout<<("enable_edge: edge not found");
    }

    void disable_edge(int from_id, int to_id) {
        int fi = find_node_index(from_id);
        if (fi == -1) cout<<("disable_edge: from node not found");
        auto &edges = adj[fi];
        for (auto &e : edges) {
            if (e.to == to_id) {
                e.is_active = false;
                return;
            }
        }
        cout<<("disable_edge: edge not found");
    }

    Node get_node(int id) const {
        int idx = find_node_index(id);
        if (idx == -1) cout<<("get_node: node not found");
        return nodes[idx];
    }

    Edge get_edge(int from_id, int to_id) const {
        int fi = find_node_index(from_id);
        if (fi == -1) cout<<("get_edge: from node not found");
        const auto &edges = adj[fi];
        for (const auto &e : edges) {
            if (e.to == to_id) return e;
        }
        cout<<("get_edge: edge not found");
    }

    vector<Edge> neighbors(int id, bool include_inactive = false) const {
        int idx = find_node_index(id);
        if (idx == -1) cout<<("neighbors: node not found");

        vector<Edge> result;
        const auto &edges = adj[idx];
        result.reserve(edges.size());
        for (const auto &e : edges) {
            if (!include_inactive && !e.is_active) continue;
            result.push_back(e);
        }
        return result;
    }

    bool has_node(int id) const noexcept {
        return find_node_index(id) != -1;
    }

    bool has_edge(int from_id, int to_id) const noexcept {
        int fi = find_node_index(from_id);
        if (fi == -1) return false;
        const auto &edges = adj[fi];
        return any_of(edges.begin(), edges.end(), [&](const Edge &e){ return e.to == to_id; });
    }
    
    
    vector<vector<int>> find_islands(bool closed_means_inactive = true) {
        vector<vector<int>> islands;
        vector<bool> visited(nodes.size(), false);

        for (int start = 0; start < (int)nodes.size(); ++start) {
            if (!visited[start]) {
                vector<int> island;
                queue<int> q;
                visited[start] = true;
                q.push(start);

                while (!q.empty()) {
                    int u = q.front(); q.pop();
                    island.push_back(u);

                    for (const auto& e : adj[u]) {
                        bool isClosed = closed_means_inactive ? !e.is_active : e.is_active;
                        if (isClosed) {
                            int v_idx = find_node_index(e.to);
                            if (v_idx >= 0 && !visited[v_idx]) {
                                visited[v_idx] = true;
                                q.push(v_idx);
                            }
                        }
                    }
                }
                islands.push_back(island);
                }
        }
        return islands;
    }
    
    
    bool isClosedEdge(const Edge& e, bool closed_means_inactive) const {
        return closed_means_inactive ? (!e.is_active) : (e.is_active);
    }

    bool dfsCycle(int u_idx, int parent_idx,
                vector<bool>& visited,
                bool closed_means_inactive) const
    {
        visited[u_idx] = true;

        for (const auto& e : adj[u_idx]) {
            if (!isClosedEdge(e, closed_means_inactive)) continue;

            int v_idx = find_node_index(e.to);
            if (v_idx < 0) continue;

            if (!visited[v_idx]) {
                if (dfsCycle(v_idx, u_idx, visited, closed_means_inactive)) {
                    return true;
                }
            } else if (v_idx != parent_idx) {
                return true;
            }
        }
        return false;
    }

    bool hasCycle(bool closed_means_inactive = true) const {
        const int N = static_cast<int>(nodes.size());
        if (N == 0) return false;

        vector<bool> visited(N, false);

        for (int i = 0; i < N; ++i) {
            if (!visited[i]) {
                if (dfsCycle(i, -1, visited, closed_means_inactive)) {
                    return true;
                }
            }
        }
        return false;
    }

    
    pair<double, vector<int>> dijkstra_idx(int start_idx, int goal_idx) const {
        const int n = (int)nodes.size();
        const double INF = numeric_limits<double>::infinity();

        vector<double> dist(n, INF);
        vector<int> parent(n, -1);
        dist[start_idx] = 0.0;

        using NodeState = pair<double,int>;
        priority_queue<NodeState, vector<NodeState>, greater<NodeState>> pq;
        pq.emplace(0.0, start_idx);

        while (!pq.empty()) {
            auto [d, u_idx] = pq.top(); pq.pop();
            if (d > dist[u_idx]) continue;
            if (u_idx == goal_idx) break;

            for (const auto& e : adj[u_idx]) {
                if (!e.is_active) continue;         
                int v_idx = find_node_index(e.to); 
                if (v_idx < 0) continue;

                double w = e.distance;
                if (dist[v_idx] > dist[u_idx] + w) {
                    dist[v_idx] = dist[u_idx] + w;
                    parent[v_idx] = u_idx;
                    pq.emplace(dist[v_idx], v_idx);
                }
            }
        }

        vector<int> path;
        if (!isfinite(dist[goal_idx])) return {INF, path};
        for (int node = goal_idx; node != -1; node = parent[node]) path.push_back(node);
        reverse(path.begin(), path.end());
        return {dist[goal_idx], path};
    }
};