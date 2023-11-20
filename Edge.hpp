#ifndef EDGE_H
#define EDGE_H
#include <string>

class Vertex; // Forward declaration

class Edge {
private:
    Vertex* source;
    Vertex* target;
    unsigned long weight;

public:
    Edge(Vertex* source, Vertex* target, unsigned long weight);
    Vertex* getSource() const;
    Vertex* getTarget() const;
    unsigned long getWeight() const;
};

#endif // EDGE_H
