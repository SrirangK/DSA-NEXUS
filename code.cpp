#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include "json.hpp" 

using namespace std;
using json = nlohmann::json;

struct Edge {
    int to;
    double weight;
};

struct ParkingSlot {
    int id;
    bool occupied;
};

class ParkingLot {
private:
    int numNodes;
    unordered_map<int, vector<Edge>> adj;
    unordered_map<int, ParkingSlot> slots;

public:
    void loadGraph(const string& filename) {
        ifstream file(filename);
        if (!file) {
            cerr << "Error: Cannot open file " << filename << endl;
            return;
        }

        json graphData;
        file >> graphData;

        numNodes = graphData["nodes"].size();

        // Load nodes
        for (auto& node : graphData["nodes"]) {
            int id = node["id"];
            if (node["is_parking"]) {
                slots[id] = {id, false};
            }
        }

        // Load edges
        for (auto& edge : graphData["edges"]) {
            int from = edge["from"], to = edge["to"];
            double weight = edge["weight"];
            adj[from].push_back({to, weight});
            adj[to].push_back({from, weight});
        }

        cout << "Graph successfully loaded from " << filename << "!" << endl;
    }

    // Dijkstra's Algorithm for shortest path
    vector<double> dijkstra(int src) {
        vector<double> dist(numNodes, numeric_limits<double>::infinity());
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            auto [currentDist, u] = pq.top();
            pq.pop();

            if (currentDist > dist[u]) continue;

            for (const auto& edge : adj[u]) {
                int v = edge.to;
                double weight = edge.weight;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }
        return dist;
    }

    // Find the best parking slot using a weighted balance
    int findOptimalSlot(int entry, int buildingEntrance, int exit, double alpha, double beta, double gamma) {
        vector<double> fromEntry = dijkstra(entry);
        vector<double> toBuilding = dijkstra(buildingEntrance);
        vector<double> toExit = dijkstra(exit);

        double bestScore = numeric_limits<double>::infinity();
        int bestSlot = -1;

        for (const auto& [id, slot] : slots) {
            if (!slot.occupied) {
                double score = alpha * fromEntry[id] + beta * toBuilding[id] + gamma * toExit[id];
                if (score < bestScore) {
                    bestScore = score;
                    bestSlot = id;
                }
            }
        }
        return bestSlot;
    }
    void printPath(int src, int dest, const vector<int>& prev) {
        vector<int> path;
        for (int at = dest; at != -1; at = prev[at])
            path.push_back(at);
    
        reverse(path.begin(), path.end());
    
        cout << "Path from " << src << " to " << dest << ": ";
        for (int node : path)
            cout << node << " ";
        cout << endl;
    }

    // Dijkstra's Algorithm with path reconstruction
    pair<vector<double>, vector<int>> dijkstraWithPath(int src, int numNodes, unordered_map<int, vector<Edge>>& adj) {
        vector<double> dist(numNodes, numeric_limits<double>::infinity());
        vector<int> prev(numNodes, -1);
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    
        dist[src] = 0;
        pq.push({0, src});
    
        while (!pq.empty()) {
            auto [currentDist, u] = pq.top();
            pq.pop();
    
            if (currentDist > dist[u]) continue;
    
            for (const auto& edge : adj[u]) {
                int v = edge.to;
                double weight = edge.weight;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
    
        return {dist, prev};
    }
    
    // Park a vehicle
    void parkVehicle(int entry, int buildingEntrance, int exit, double alpha, double beta, double gamma) {
        auto [fromEntry, prevFromEntry] = dijkstraWithPath(entry, numNodes, adj);
        auto [toBuilding, _] = dijkstraWithPath(buildingEntrance, numNodes, adj);
        auto [toExit, prevToExit] = dijkstraWithPath(exit, numNodes, adj);
    
        double bestScore = numeric_limits<double>::infinity();
        int bestSlot = -1;
    
        for (const auto& [id, slot] : slots) {
            if (!slot.occupied) {
                double score = alpha * fromEntry[id] + beta * toBuilding[id] + gamma * toExit[id];
                if (score < bestScore) {
                    bestScore = score;
                    bestSlot = id;
                }
            }
        }
    
        if (bestSlot != -1) {
            slots[bestSlot].occupied = true;
            cout << "Vehicle parked at slot: " << bestSlot << endl;
    
            // Print entry to slot path
            printPath(entry, bestSlot, prevFromEntry);

        } else {
            cout << "No available slots." << endl;
        }
    }
    
    // Vacate a parking slot
    void leaveParking(int slot) {
        if (slots.find(slot) != slots.end() && slots[slot].occupied) {
            slots[slot].occupied = false;
            cout << "Slot " << slot << " is now free." << endl;
        } else {
            cout << "Slot " << slot << " is already free" << endl;
        }
    }
};

int main() {
    ParkingLot lot;
    lot.loadGraph("parking_graph.json"); // Load graph generated from image

    // Define key locations
    int parkingEntryPoint = 0;
    int buildingEntrance = 5;
    int parkingExitPoint = 3;

    double alpha = 0.4; // Weight for entry distance
    double beta =   0.4;  // Weight for building entrance distance
    double gamma = 0.2; // Weight for exit distance

    // Park a vehicle
    lot.parkVehicle(parkingEntryPoint, buildingEntrance, parkingExitPoint, alpha, beta, gamma);
    // lot.leaveParking(4);
    lot.parkVehicle(1, buildingEntrance, parkingExitPoint, alpha, beta, gamma);

    // Leave a slot
    lot.leaveParking(2);

    return 0;
}
