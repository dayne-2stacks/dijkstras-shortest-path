#ifndef VERTEX_H
#define VERTEX_H
#include <list>
#include <string>
#include <unordered_map>
#include "Edge.hpp"

class Vertex {
private:
    std::string label;
    std::unordered_map<Vertex*, Edge*> edges;

public:
    Vertex(const std::string& label) : label(label) {}

    const std::string& getLabel() const {
        return label;
    }

    void addEdge(Vertex* neighbor, Edge* edge) {
        edges[neighbor] = edge;
    }

    void removeEdge(Vertex* neighbor) {
        edges.erase(neighbor);
    }

    const std::unordered_map<Vertex*, Edge*>& getEdges() const {
        return edges;
    }
};

#endif // VERTEX_H