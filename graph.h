// graph.h
//
// <Mira Sweis>
//
// Basic graph class using adjacency matrix representation.
//
// University of Illinois at Chicago
// CS 251: Spring 2022
// Project #7 - Openstreet Maps
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <algorithm>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
     map<VertexT, map<VertexT, WeightT>> adjList;
 public:
  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  // NOTE: the graph is implemented using an adjacency matrix.
  // If n exceeds the dimensions of this matrix, an exception
  // will be thrown to let you know that this implementation
  // will not suffice.
  //
  graph() {
      // empty
  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return adjList.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;
    //
    // loop through the adjacency matrix and count how many
    // edges currently exist:
    //
    for (auto& edge : adjList) {
        count += edge.second.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    //
    // is the vertex already in the graph?  If so, we do not
    // insert again otherwise Vertices may fill with duplicates:
    //
    if (adjList.count(v) > 0) {
        return false;
    }
    //
    // if we get here, vertex does not exist so insert.  Where
    // we insert becomes the rows and col position for this
    // vertex in the adjacency matrix.
    //
    map<VertexT, WeightT> newMap;
    this->adjList[v] = newMap; 

    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    //
    // we need to search the Vertices and find the position
    // of each vertex; this will denote the row and col to
    // access in the adjacency matrix:
    //
    if (adjList.count(from) == 0 || adjList.count(to) == 0) {
        return false;
    }
    //
    // the vertices exist and we have the row and col of the
    // adjacency matrix, so insert / update the edge:
    //
    this->adjList[from][to] = weight;

    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // does vertex to and from even exist?
    // if no - return false
    if (adjList.count(from) == 0 || adjList.at(from).count(to) == 0) {
        return false;
    }
    //
    // Okay, the edge exists, return the weight via the
    // reference parameter:
    //
    weight = this->adjList.at(from).at(to);

    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;

    // if the vertex doesn't exist; return the empty set
    if (adjList.count(v) == 0) {
        return S;
    }
    // insert the neighboring verticies in a set
    for (auto& ver : adjList.at(v)) {
        S.insert(ver.first);
    }
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
      vector<VertexT> verticies;
      // pushes all verticies in a vector
      for (auto& ver : this->adjList) {
          verticies.push_back(ver.first);
      }
      return verticies;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
     output << "***************************************************" << endl;
     output << "********************* GRAPH ***********************" << endl;
     output << "**Num vertices: " << this->NumVertices() << endl;
     output << "**Num edges: " << this->NumEdges() << endl;

     output << endl;
     output << "**Vertices:" << endl;
     int i = 0;
     for (auto& e : adjList) {
       output << " " << i << ". " << e.first << endl;
       i++;
     }

     output << endl;
     output << "**Edges:" << endl;
     for (auto &e : adjList) {
       output << e.first << ": ";
       for (auto &x : e.second) {
           output << "(" << e.first << "," << x.first << "," << x.second << ")" << endl;
       }
       output << endl;
     }
     output << "**************************************************" << endl;
  }
};
