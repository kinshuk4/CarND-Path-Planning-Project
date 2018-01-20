//
// Created by Kinshuk Chandra on 23.01.18.
//

#include "VehicleController.h"
#include "constants.h"
#include <math.h>

bool VehicleController::is_too_close(int prev_size, double car_s, double end_path_s) const {
    for (int i = 0; i < sensor_fusion.size(); ++i) {
        vector<double> sensor_fusion_data = sensor_fusion[i];
        VehicleFusionData vehicleFusionData(sensor_fusion_data);

        if (vehicleFusionData.is_in_lane(lane)) {
            double check_car_s_proj = vehicleFusionData.check_car_s_projection(prev_size);
            // Check if this car is in front and too close to us using previous values
            if ((check_car_s_proj > car_s) && ((check_car_s_proj - end_path_s) < SAFETY_MARGIN)) {
                return true;
            }
        }
    }
    return false;
}

VehicleController::VehicleController(vector<vector<double>> sensor_fusion, WayPointsMap wayPointsMap, int lane) {
    sensor_fusion = sensor_fusion;
    wayPointsMap = wayPointsMap;
    lane = lane;
}

//almost same as is_to_close but here we are using lane from method argument
bool VehicleController::is_lane_available(int prev_size, double car_s, int lane) const {
    for (int i = 0; i < sensor_fusion.size(); ++i) {
        vector<double> sensor_fusion_data = sensor_fusion[i];
        VehicleFusionData vehicleFusionData(sensor_fusion_data);

        if (vehicleFusionData.is_in_lane(lane)) {
            double check_car_s_proj = vehicleFusionData.check_car_s_projection(prev_size);
            // Check if this car is in front and too close to us using previous values
            //using fabs to take care of cars from behind
            if (fabs(check_car_s_proj - car_s) < SAFETY_MARGIN) {
                return false;
            }
        }
    }
    return true;


}

VehicleAction VehicleController::take_vehicle_action(double ref_vel, bool too_close, int prev_size, double car_s) const {
    LaneDecision laneDecision = KEEP_LANE;

    if (too_close) {

        //only when too close we have to change the lane
        //first try to see if can go right
        int curr_lane = lane;
        //prefer to take left
        //if current lane is already not left and
        if (curr_lane != MIN_LANE && is_lane_available(prev_size, car_s, curr_lane - 1)) {
            return VehicleAction(ref_vel, curr_lane-1);
        } else if (curr_lane != MAX_LANE && is_lane_available(prev_size, car_s, curr_lane + 1)){
            return VehicleAction(ref_vel, curr_lane+1);
        }else{
            ref_vel -= ACCELERATION;
            return VehicleAction(ref_vel, curr_lane);
        }

    } else if (ref_vel < MAX_VEL) {
        ref_vel += ACCELERATION;
        return VehicleAction(ref_vel, lane);
    }else{
        return VehicleAction(ref_vel, lane);
    }
}
