/*dist.h*/

//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreen Maps
// 

#include <iostream>
#include <cmath>
#include "osm.h"

using namespace std;

double distBetween2Points(double lat1, double long1, double lat2, double long2);
Coordinates centerBetween2Points(double lat1, double long1, double lat2, double long2);
