#include "Graph.hpp"


void Graph::addVertex(const std::string& label) {
    vertices[label] = new Vertex(label);
}

void Graph::addEdge(const std::string& label1, const std::string& label2, unsigned long weight) {
    if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
        std::cerr << "Error: Vertex not found." << std::endl;
        return;
    }

    Vertex* v1 = vertices[label1];
    Vertex* v2 = vertices[label2];

    Edge* edge = new Edge(v1, v2, weight);
    v1->addEdge(v2, edge);
    v2->addEdge(v1, edge);
}

void Graph::printAdjacencyList() const {
    for (const auto& entry : vertices) {
        Vertex* vertex = entry.second;
        const std::string& label = entry.first;
        
        std::cout << "Vertex " << label << " neighbors: ";
        for (const auto& neighborEntry : vertex->getEdges()) {
            Vertex* neighbor = neighborEntry.first;
            Edge* edge = neighborEntry.second;
            std::cout << neighbor->getLabel() << " (Weight: " << edge->getWeight() << ") ";
        }
        std::cout << std::endl;
    }
}

void Graph::removeVertex(const std::string& label) {
    if (vertices.find(label) == vertices.end()) {
        std::cerr << "Error: Vertex not found." << std::endl;
        return;
    }

    Vertex* vertexToRemove = vertices[label];

    // Remove edges associated with the vertex
    for (const auto& neighborEntry : vertexToRemove->getEdges()) {
        Vertex* neighbor = neighborEntry.first;
        Edge* edge = neighborEntry.second;
        neighbor->removeEdge(vertexToRemove);
        delete edge;
    }

    // Remove the vertex itself
    vertices.erase(label);
    delete vertexToRemove;
}

void Graph::removeEdge(const std::string& label1, const std::string& label2) {
    if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
        std::cerr << "Error: Vertex not found." << std::endl;
        return;
    }

    Vertex* v1 = vertices[label1];
    Vertex* v2 = vertices[label2];

    if (v1 == v2) {
        std::cerr << "Error: Attempted to remove a self-loop edge." << std::endl;
        return;
    }

    // Check if an edge exists between the two vertices
    Edge* edgeToRemove = nullptr;
    for (const auto& neighborEntry : v1->getEdges()) {
        if (neighborEntry.first == v2) {
            edgeToRemove = neighborEntry.second;
            break;
        }
    }

    if (!edgeToRemove) {
        std::cerr << "Error: Edge not found between " << label1 << " and " << label2 << "." << std::endl;
        return;
    }

    // Remove the edge from both vertices
    v1->removeEdge(v2);
    v2->removeEdge(v1);
    delete edgeToRemove;
}



Graph::~Graph() {
    for (const auto& entry : vertices) {
        delete entry.second; // Clean up memory
    }
}

unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) {
    // A map to store minimum distance to each vertex.
    std::unordered_map<Vertex*, unsigned long> distances;
    // A map to store "previous" relation to rebuild the shortest path.
    std::unordered_map<Vertex*, Vertex*> previous;
    // Custom comparator for the priority queue.
    auto cmp = [&distances](Vertex* left, Vertex* right) { return distances[left] > distances[right]; };
    // Priority queue to select the vertex with the smallest distance.
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(cmp)> queue(cmp);

    // Initialize distances and queue.
    for (const auto& pair : vertices) {
        distances[pair.second] = (pair.first == startLabel) ? 0 : std::numeric_limits<unsigned long>::max();
        queue.push(pair.second);
    }

    Vertex* startVertex = vertices[startLabel];
    Vertex* endVertex = vertices[endLabel];

    // Algorithm execution
    while (!queue.empty()) {
        Vertex* current = queue.top();
        queue.pop();

        if (current == endVertex) {
            break; // If we reached the destination.
        }

        // Visit each edge exiting current
        for (const auto& edgePair : current->getEdges()) {
            Vertex* neighbor = edgePair.first;
            unsigned long weight = edgePair.second->getWeight();
            unsigned long distanceThroughU = distances[current] + weight;

            // If shorter path from start to neighbor found,
            // update neighbor's distance and previous Vertex
            if (distanceThroughU < distances[neighbor]) {
                distances[neighbor] = distanceThroughU;
                previous[neighbor] = current;
                // Since we cannot update the priority of an element inside the heap,
                // we are just pushing the updated vertex again. The vertex with the longer
                // distance will be ignored later on.
                queue.push(neighbor);
            }
        }
    }

    // Reconstruct the shortest path
    if (distances[endVertex] != std::numeric_limits<unsigned long>::max()) {
        for (Vertex* at = endVertex; at != nullptr; at = previous[at]) {
            path.insert(path.begin(), at->getLabel());
        }
    }

    return distances[endVertex]; // Return the shortest distance
}