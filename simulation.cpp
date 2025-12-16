// demo_graph.cpp
// Demonstrates Graph API from graph.hpp
// - add_node, connect
// - neighbors, get_node, get_edge
// - enable/disable node and edge
// - find_islands (returns internal indices) -> convert to IDs because we add nodes in order
// - hasCycle
// - dijkstra_idx (uses internal indices)
//
// Compile: g++ -std:c++17 demo_graph.cpp -o demo_graph.exe

#include "graph.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

using std::cout;
using std::endl;

int main() {
    Graph g;

    g.add_node("A", 1);
    g.add_node("B", 2);
    g.add_node("C", 3);
    g.add_node("D", 4);
    g.add_node("E", 5);
    g.add_node("F", 6);
    // Active triangle: 1->2 (1.0), 2->3 (1.0), 3->1 (1.0)
    g.connect(1, 2, 1.0, true);
    g.connect(2, 3, 1.0, true);
    g.connect(3, 1, 1.0, true);
    g.connect(2, 4, 3.0, true);

    g.connect(4, 5, 2.0, false);
    g.connect(5, 6, 2.0, false);

    cout <<"=== Neighbors for node 1 ===\n";
    for (auto &e : g.neighbors(1)) {
        cout << "  " << e.from << " -> " << e.to << " dist=" << e.distance << " active=" << (e.is_active ? "yes" : "no") << "\n";
    }

    cout <<"=== All outgoing from node 1 ===\n";
    for (auto &e : g.neighbors(1, true)) {
        cout << "  " << e.from << " -> " << e.to << " dist=" << e.distance << " active=" << (e.is_active ? "yes" : "no") << "\n";
    }
    cout<<"\n";
    // Disable an edge
    cout << "Disable edge 1->2\n";
    g.disable_edge(1, 2);
    auto out1 = g.neighbors(1);
    cout << " neighbors(1) = " << out1.size() << "\n";

    cout << "enable edge 1->2\n";
    g.enable_edge(1, 2);
    cout <<" edge 1->2 active = "<<(g.get_edge(1,2).is_active ? "yes":"no")<<"\n";

    // enable/disable node
    cout << "Disable node 2\n";
    g.disable_node(2);
    cout << "node 2 active = " << (g.get_node(2).is_active ? "yes" : "no") << "\n";
    cout << "Enable node 2\n";
    g.enable_node(2);

    // Find islands
    cout << "=== find_islands ===\n";
    auto islands_active = g.find_islands(false);
    for (size_t i = 0; i < islands_active.size(); ++i) {
        cout << " island " << i << ":";
        for (int idx : islands_active[i]) {
            cout << " " << (idx + 1);
        }
        cout << "\n";
    }

    // Cycle detection
    cout << "\n";
    cout << "Cycle check : " << (g.hasCycle(false) ? "CYCLE FOUND" : "no cycle") << "\n";

    cout << "\n=== Dijkstra from id=1 to id=4 ===\n";
    auto [dist, path_idx] = g.dijkstra_idx(0, 3);
    if (!isfinite(dist)) {
        cout << " unreachable\n";
    } else {
        cout <<"distance = " << dist << "   path (indices) :";
        for (int idx : path_idx) cout << " " << idx <<"\n";
        cout <<"path :";
        for (int idx : path_idx) cout << " " << (idx + 1);
        cout << "\n";
    }

    return 0;
}