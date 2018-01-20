//
// Created by Kinshuk Chandra on 23.01.18.
//
#include "constants.h"

#ifndef PATH_PLANNING_VEHICLEACTION_H
#define PATH_PLANNING_VEHICLEACTION_H


struct VehicleAction {
    double ref_vel;
    int lane;

    VehicleAction(double ref_vel, int lane);
};


#endif //PATH_PLANNING_VEHICLEACTION_H
