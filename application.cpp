// application.cpp
// <Mira Sweis>
//
// University of Illinois at Chicago
// CS 251: Spring 2022
// Project #7 - Openstreet Maps
//
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

/*

Creative Component:
    My creative component, was inspired by the ideas given on the Project 7
    jumpstart. I decided to write a program that takes in a group of peoples
    locations then finds a suitable meeting place for everyone.
    It first takes in the number of people. Then, one by one, takes
    in their locations. FInally, using functions we wrote, we find the
    center building between everyone.
    To run the program: enter "c" whatever map you want, then a valid number
    of people and their locations.

*/

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <limits>
#include <stack>
#include "tinyxml2.h"
#include "dist.h"
#include "osm.h"
#include "graph.h"

using namespace std;
using namespace tinyxml2;
// Infinity value for Dijkstra
const int INF = numeric_limits<int>::max();

// copied from lab 11 work
// prioritize is orginally from project 6
class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
    const pair<long long, double>& p2) const {
        if (p1.second == p2.second) {
            return p1.first > p2.first;
        }
        return p1.second > p2.second;
      }
};

// Dijkstra's algorithm from lab #11
// added perameters for predessesors and destination point
vector<long long> Dijkstra(graph<long long, double>& G, long long& startV,
    map<long long, double>& distances, map<long long, long long>& pred) {
    // start
    vector<long long>  visited;
    priority_queue <pair<long long, double>,
        vector<pair<long long, double>>, prioritize> pq;

    for (long long &vertex : G.getVertices()) {
        distances[vertex] = INF;
        pred[vertex] = -1;
        pq.push(make_pair(vertex, INF));
    }
    distances[startV] = 0;
    pq.push(make_pair(startV, 0));

    while (!pq.empty()) {
        pair <long long, double> curr = pq.top();
        pq.pop();

        if (distances[curr.first] == INF) {
            break;
        } else if (curr.second >= INF) {
            break;
        } else if (count(visited.begin(), visited.end(), curr.first) != 0) {
            continue;
        } else {
            visited.push_back(curr.first);
        }
        for (long long s : G.neighbors(curr.first)) {
            double edgeWeight;
            G.getWeight(curr.first, s, edgeWeight);
            double pathDistance = curr.second + edgeWeight;
            if (pathDistance < distances[s]) {
                distances[s] = pathDistance;
                pq.push(make_pair(s, pathDistance));
                pred[s] = curr.first;
            }
        }
    }
    return visited;
}


// search for persons building
// for milestone 7
bool searchBuilding(string& query, vector<BuildingInfo>& buildings,
    BuildingInfo& result) {
    for (auto const& e : buildings) {
        // search building abbreviations
        if (e.Abbrev == query) {
            result = e;
            return true;
        }
        // search building names
        if (e.Fullname.find(query) != string::npos) {
            result = e;
            return true;
        }
    }
    return false;
}

// finds nearest building
// for milestone 8
BuildingInfo nearestBuilding(Coordinates& midpoint,
    vector<BuildingInfo>& buildings, set<string>& unreachable) {
    BuildingInfo result;
    double min = INF;
    double distance;
    // loop through the buildings to find the closest to the midpoint
    for (auto& e : buildings) {
        if (unreachable.count(e.Fullname) != 0) {
            continue;
        }
        distance = distBetween2Points(midpoint.Lat, midpoint.Lon, e.Coords.Lat, e.Coords.Lon);
        if (distance < min) {
            min = distance;
            result = e;
        }
    }
    return result;
}

// finds nearest nodes
// for milestone #9
long long nearestNode(BuildingInfo& building, map<long long,
    Coordinates>& Nodes, vector<FootwayInfo>& Footways) {
    // Visual Studio made me initialize the result
    long long result = Footways[0].Nodes[0];
    double min = INF;
    double distance;
    // loop throught the footways an nodes
    for (auto& e : Footways) {
        for (auto& x : e.Nodes) {
            distance = distBetween2Points(building.Coords.Lat,
                building.Coords.Lon, Nodes[x].Lat, Nodes[x].Lon);
            // find minimum distance
            if (distance < min) {
                min = distance;
                result = x;
            }
        }
    }
    return result;
}

//
// gets the path for each person
// for milestone 11
void getPath(map<long long, long long>& pred, long long& start,
    long long& dest, map<long long, double>& distance) {
    // was suggested to check the edge case one more time
    if (distance[dest] >= INF) {
        cout << "sorry, destination unreachable." << endl << endl;
        return;
    }
    // creates a stack of path
    stack<long long> path;
    long long cur = dest;
    while (cur != start) {
        path.push(cur);
        cur = pred[cur];
    }
    // outputs path, poping every current node in stack
    cout << "Path: " << start;
    while (!path.empty()) {
        cur = path.top();
        cout << "->" << cur;
        path.pop();
    }
    cout << endl;
}

//
// Implement your creative component application here
//
void creative(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double> G) {
    // creative application
    int numberOfPeople;
    string location;
    BuildingInfo building;
    set<string> invalid;
    stack<double> lat;
    stack<double> lon;
    bool found = false;
    int i = 0;

    cout << "Enter the number of people: ";
    cin >> numberOfPeople;
    if (numberOfPeople < 0 || numberOfPeople > 1000) {
        cout << "Enter a valid number or meet somewhere else: ";
        cin >> numberOfPeople;
    }
    cout << endl;
    getline(cin, location);
    // loops to get the locations of everyone
    // whille doing so, we can get the latitudes and longitudes of each person
    // then find the midpoint
    while (location != "#") {
        cout << "input location of person " << i + 1 << ": ";
        getline(cin, location);
        found = searchBuilding(location, Buildings, building);
        if (!found) {
            cout << "Please enter a correct building name" << endl;
            getline(cin, location);
            continue;
        }
        lat.push(building.Coords.Lat);
        lon.push(building.Coords.Lon);
        i++;
        if (i == numberOfPeople) {
            break;
        }
    }
    // to find the midpoint
    // go through every two points of in the stack, calculate the midpoint
    // then remove from the stack
    // lat and lon should be the same size stack
    for (int i = 0; i < lon.size(); ++i) {
        double lat1 = lat.top();
        lat.pop();
        double lat2 = lat.top();
        lat.pop();
        double lon1 = lon.top();
        lon.pop();
        double lon2 = lon.top();
        lon.pop();
        Coordinates point = centerBetween2Points(lat1, lon1, lat2, lon2);
        lat.push(point.Lat);
        lon.push(point.Lon);
    }
    Coordinates mid;
    mid.Lat = lat.top();
    mid.Lon = lon.top();
    // find nearest midpoint building
    BuildingInfo midBuild = nearestBuilding(mid, Buildings, invalid);
    cout << "Best meeting place: " << midBuild.Fullname << endl;
}

//
// Implement your standard application here
// Added perameter for our graph
//
void application(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double> G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // MILESTONE 7: Search Buildings 1 and 2
    bool found1 = false;
    bool found2 = false;
    BuildingInfo building1;
    BuildingInfo building2;
    found1 = searchBuilding(person1Building, Buildings, building1);
    found2 = searchBuilding(person2Building, Buildings, building2);

    // if one of the inputs are false inputs then ask for another input
    if (!found1) {
        cout << "Person 1's building not found" << endl;
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
        continue;
    }
    if (!found2) {
        cout << "Person 2's building not found" << endl;
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
        continue;
    }
    cout << "Person 1's point:" << endl;
    cout << " " << building1.Fullname << endl;
    cout << " (" << building1.Coords.Lat << ", ";
    cout << building1.Coords.Lon << ")" << endl;
    cout << "Person 2's point:" << endl;
    cout << " " << building2.Fullname << endl;
    cout << " (" << building2.Coords.Lat << ", ";
    cout << building2.Coords.Lon << ")" << endl;
    // MILESTONE 8: Locate Center Building
    Coordinates midpoint;
    midpoint = centerBetween2Points(building1.Coords.Lat,
        building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);

    // bool for inner while loop
    bool path = false;

    // for milestone 9
    long long dist1;
    long long dist2;
    long long mid;
    // to compare building names to unreachable building names
    set<string> invalidDest;
    // for milestone 10 / 11
    map<long long, long long> pred0;
    map<long long, long long> pred1;
    map<long long, long long> pred2;
    map<long long, double> centerDist;
    map<long long, double> distance1;
    map<long long, double> distance2;
    vector<long long> visited;

    // while the path is not found
    while (path == false) {
        BuildingInfo centerBuilding;
        centerBuilding = nearestBuilding(midpoint, Buildings, invalidDest);
        // MILESTONE 9: Find Nearest Nodes from buildings 1, 2 & Center
        dist1 = nearestNode(building1, Nodes, Footways);
        dist2 = nearestNode(building2, Nodes, Footways);
        mid = nearestNode(centerBuilding, Nodes, Footways);
        // if their is no new new destination found inplace of a invalid one
        if (invalidDest.size() == 0) {
            cout << "Destination Building:" << endl;
            cout << " " << centerBuilding.Fullname << endl;
            cout << " (" << centerBuilding.Coords.Lat << ", ";
            cout << centerBuilding.Coords.Lon << ")" << endl << endl;
            // print nearest node for each building
            // for P1
            cout << "Nearest P1 node:" << endl;
            cout << " " << dist1 << endl;
            cout << " (" << Nodes[dist1].Lat << ", ";
            cout << Nodes[dist1].Lon << ")" << endl;
            // for P2
            cout << "Nearest P2 node:" << endl;
            cout << " " << dist2 << endl;
            cout << " (" << Nodes[dist2].Lat << ", ";
            cout << Nodes[dist2].Lon << ")" << endl;
            // for destination
            cout << "Nearest destination node:" << endl;
            cout << " " << mid << endl;
            cout << " (" << Nodes[mid].Lat << ", ";
            cout << Nodes[mid].Lon << ")" << endl;
        } else {
            // output the new desitantion
            cout << "New destination building:" << endl;
            cout << " " << centerBuilding.Fullname << endl;
            cout << " (" << centerBuilding.Coords.Lat << ", ";
            cout << centerBuilding.Coords.Lon << ")" << endl;
            // output the new nearest node
            cout << "Nearest destination node:" << endl;
            cout << " " << mid << endl;
            cout << " (" << Nodes[mid].Lat << ", ";
            cout << Nodes[mid].Lon << ")" << endl;
        }
        // MILESTONE 10: Run Dijkstra’s Algorithm
        // Dijkstra's algorith is added above from lab 11

        // runs dijkstra on the center building
        visited = Dijkstra(G, mid, centerDist, pred0);
        visited = Dijkstra(G, dist1, distance1, pred1);
        // if person cannot reach person 2
        if (distance1[dist2] >= INF) {
            cout << "Sorry, destination unreachable." << endl << endl;
            // break from loop and repeat process
            break;
        }
        // run Dijkstra's on person 1 and 2 buildings
        visited = Dijkstra(G, dist2, distance2, pred2);
        // if the peaople cannot reach the destination
        if (centerDist[mid] >= INF) {
            cout << "Sorry, destination unreachable." << endl << endl;
            break;
        }
        // MILESTONE 11: Find Second Nearest Destination (loop again)
        // if both people cant reach the center building
        if (distance1[mid] >= INF || distance2[mid] >= INF) {
            cout << endl;
            cout << "At least one person was unable to reach the destination building.";
            cout << " Finding next closest building..." << endl;
            cout << endl;
            // adds unreachable building to the set
            invalidDest.insert(centerBuilding.Fullname);
        } else {
            // path found!
            path = true;
            // MILESTONE 11: Print path (path found! break)
            cout << "Person 1's distance to dest: ";
            cout << distance1[mid] << " miles" << endl;
            getPath(pred1, dist1, mid, distance1);
            cout << endl;
            cout << "Person 2's distance to dest: ";
            cout << distance2[mid] << " miles" << endl;
            getPath(pred2, dist2, mid, distance2);
        }
    }
    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  graph<long long, double> G;

  // MILESTONE 5: add vertices
  for (auto const& e : Nodes) {
      G.addVertex(e.first);
  }
  // MILESTONE 6: add edges
  for (auto e : Footways) {
      for (int i = 0; i < e.Nodes.size() - 1; ++i) {
          // find coordinates
          Coordinates c1 = Nodes[e.Nodes[i]];
          Coordinates c2 = Nodes[e.Nodes[i + 1]];
          // calulate the distance
          double distance = distBetween2Points(c1.Lat, c1.Lon, c2.Lat, c2.Lon);
          G.addEdge(c1.ID, c2.ID, distance);
          G.addEdge(c2.ID, c1.ID, distance);
      }
  }
  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Menu
  //
  string userInput;
  cout << "Enter \"a\" for the standard application or " 
        << "\"c\" for the creative component application> ";
  getline(cin, userInput);
  if (userInput == "a") {
    application(Nodes, Footways, Buildings, G);
  } else if (userInput == "c") {
    // add arguments
    creative(Nodes, Footways, Buildings, G);
  }
  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
