//
// Created by Kinshuk Chandra on 23.01.18.
//
#include "vehicle.h"
#include "constants.h"

bool VehicleFusionData::is_in_lane(int lane) const {
    double d = this->d;
    return (d < (2 + LANE_WIDTH * lane + 2) && d > (2 + LANE_WIDTH * lane - 2));
}

double VehicleFusionData::check_car_s_projection(int prev_size) const {
    //project s value in time using prev path points assuming constant speed
    double check_car_s = this->check_car_s + double(prev_size) * 0.02 * this->check_speed;
    return check_car_s;
}

bool VehicleFusionData::is_too_close(int prev_size, double car_s, double end_path_s) const {
    bool too_close = false;
    double check_car_s_proj = check_car_s_projection(prev_size);
    // Check if this car is in front and too close to us using previous values
    if ((check_car_s_proj > car_s) && ((check_car_s_proj - end_path_s) < SAFETY_MARGIN)) {
        too_close = true; // The car in front is dangerously close
    }

    return too_close;
}
