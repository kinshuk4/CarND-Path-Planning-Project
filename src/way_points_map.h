//
// Created by Kinshuk Chandra on 23.01.18.
//
#include <vector>
#include <string>

using namespace std;
#ifndef PATH_PLANNING_WAY_POINTS_MAP_H
#define PATH_PLANNING_WAY_POINTS_MAP_H

class WayPointsMap{
public:
    WayPointsMap();
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    //Constructors

    WayPointsMap(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                 const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_dx,
                 const vector<double> &map_waypoints_dy);

    void init(const string map_file_);
};
#endif //PATH_PLANNING_WAY_POINTS_MAP_H
