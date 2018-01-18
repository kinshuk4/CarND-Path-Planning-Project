//
// Created by Kinshuk Chandra on 23.01.18.
//
#include "way_points_map.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

WayPointsMap::WayPointsMap() {}

WayPointsMap::WayPointsMap(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                           const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_dx,
                           const vector<double> &map_waypoints_dy) : _x(map_waypoints_x),
                                                                     _y(map_waypoints_y),
                                                                     _s(map_waypoints_s),
                                                                     _dx(map_waypoints_dx),
                                                                     _dy(map_waypoints_dy) {}

void WayPointsMap::init(const string map_file_) {
    // The max s value before wrapping around the track back to 0

    ifstream in_map_(map_file_.c_str(), ifstream::in);


    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        _x.push_back(x);
        _y.push_back(y);
        _s.push_back(s);
        _dx.push_back(d_x);
        _dy.push_back(d_y);
    }
}


