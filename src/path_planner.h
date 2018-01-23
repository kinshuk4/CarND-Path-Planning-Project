//
// Created by Kinshuk Chandra on 24.01.18.
//
#include "spline.h"
#include "way_points_map.h"
#include <vector>

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H


class PathPlanner {
public:
    vector<vector<double>> getNextXYVals(
            WayPointsMap map, int lane, double car_x, double car_y, double car_yaw, double car_s, double ref_vel,
            vector<vector<double>> previous_path) const;

};


#endif //PATH_PLANNING_PATH_PLANNER_H
