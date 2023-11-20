#include "Edge.hpp"
#include "Vertex.hpp"

Edge::Edge(Vertex* source, Vertex* target, unsigned long weight)
    : source(source), target(target), weight(weight) {}

Vertex* Edge::getSource() const {
    return source;
}

Vertex* Edge::getTarget() const {
    return target;
}

unsigned long Edge::getWeight() const {
    return weight;
}
