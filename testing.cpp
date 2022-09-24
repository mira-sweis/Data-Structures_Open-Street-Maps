// testing.cpp <Starter Code>
// <Your name>
//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreet Maps
// This file is used for testing graph.h.  We encourage you to use Google'
// test framework for this project, but it is not required (because we will
// not be grading it).  
//

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <string>
#include <fstream>

#include "graph.h"

using namespace std;


int main() {

	graph<int, int> g;
	g.addVertex(1);
	g.addVertex(2);
	g.addEdge(1, 2, 3);
	g.dump(cout);

	return 0;
}