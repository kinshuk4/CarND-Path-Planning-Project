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
                           const vector<double> &map_waypoints_dy) : map_waypoints_x(map_waypoints_x),
                                                                     map_waypoints_y(map_waypoints_y),
                                                                     map_waypoints_s(map_waypoints_s),
                                                                     map_waypoints_dx(map_waypoints_dx),
                                                                     map_waypoints_dy(map_waypoints_dy) {}

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
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
}


