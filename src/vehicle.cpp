//
// Created by Kinshuk Chandra on 23.01.18.
//
#include <vector>
#include <math.h>

using namespace std;
struct VehicleFusionData
{
    double x, y;
    double d;
    double check_car_s;
    double vx, vy;
    double check_speed;
    double yaw;
    VehicleFusionData(vector<double> vec)
    {
        this -> d = vec[6];

        this -> x = vec[1];
        this -> y = vec[2];

        // Get car velocity
        this -> vx = vec[3];
        this -> vy = vec[4];
        this -> check_speed = sqrt(vx*vx + vy*vy);//get magnitude of velocity i.e. speed

        // Check the s value of the car in the same lane - how close is the car?
        this -> check_car_s = vec[5];


        this -> yaw = atan2(vy, vx);
    }
};
