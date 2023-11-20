#ifndef GRAPH_H
#define GRAPH_H

#include "Edge.hpp"
#include "Vertex.hpp"
#include "GraphBase.hpp"

#include <queue>
#include <vector>
#include <functional>
#include <unordered_map>
#include <limits>

#include <iostream>


class Graph {
private:
    std::unordered_map<std::string, Vertex*> vertices;

public:
    void addVertex(const std::string& label);
    void addEdge(const std::string& label1, const std::string& label2, unsigned long weight);
    void printAdjacencyList() const;
    void removeVertex(const std::string& label);
    void removeEdge(const std::string& label1, const std::string& label2);
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path);
    ~Graph();
};
#endif