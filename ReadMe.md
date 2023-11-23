# Graph Data Structure and Dijkstra's Algorithm Implementation

This repository contains a C++ implementation of a graph data structure using adjacency lists, along with an implementation of Dijkstra's algorithm for finding the shortest path between nodes in a graph.

## Overview

The graph data structure is a fundamental construct used extensively in computer science. It is useful for representing and navigating complex networks. Dijkstra's algorithm is an efficient algorithm for finding the shortest path from one node to all other nodes in a weighted graph.

## Files

- `Vertex.hpp`: Header file for the Vertex class representing nodes in the graph.
- `Edge.hpp`: Header file for the Edge class representing weighted edges between nodes.
-  `Edge.cpp`: Implemntation File for Edge class.
- `Graph.hpp`: Header file for the Graph class that encapsulates the graph logic.
- `Graph.cpp`: Implementation file for the Graph class.
- `GraphBase.hpp`: Abstract base class that defines the graph interface.

## Usage

To use this code, include the header files in your project and compile the `.cpp` files with your C++ compiler.

## Implementation Details

### Classes

- `Vertex`: Nodes in the graph with labels and a list of edges.
- `Edge`: Connections between vertices with weights.
- `Graph`: The graph itself, containing a map of vertices and methods for manipulating the graph.
- `GraphBase`: An interface for the graph operations.

### Graph Operations

- `addVertex`: Add a vertex to the graph.
- `addEdge`: Add an edge to the graph.
- `removeVertex`: Remove a vertex from the graph.
- `removeEdge`: Remove an edge from the graph.
- `printAdjacencyList`: Print the adjacency list of each vertex in the graph.
- `shortestPath`: Implement Dijkstra's algorithm to find the shortest path from one node to another.

### Dijkstra's Algorithm

The `shortestPath` method in the `Graph` class implements Dijkstra's algorithm. It uses a priority queue to select the next closest vertex and updates the distances to each vertex's neighbors.

## Compilation

Compile the code with a C++ compiler supporting C++11 or later. The following is an example of how you might compile the code with `g++`:

```sh
g++ -std=c++11 -Wall *.cpp
