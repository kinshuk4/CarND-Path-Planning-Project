//
// Created by Kinshuk Chandra on 23.01.18.
//
#include "way_points_map.h"
#include "vehicle.h"
#include "VehicleAction.h"
#ifndef PATH_PLANNING_VEHICLECONTROLLER_H
#define PATH_PLANNING_VEHICLECONTROLLER_H


class VehicleController {
    vector<vector<double>> sensor_fusion;
    int lane;
public:
    VehicleController(vector<vector<double>> sensor_fusion, int lane);

    bool is_too_close(int prev_size, double car_s, double end_path_s) const;

    bool is_lane_available(int prev_size, double car_s, int lane) const;

    VehicleAction take_vehicle_action(double ref_vel, bool too_close, int prev_size, double car_s) const;
};




#endif //PATH_PLANNING_VEHICLECONTROLLER_H
