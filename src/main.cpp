#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "constants.h"
#include "coordinate_utils.h"
#include "way_points_map.h"
#include "vehicle.h"
#include "VehicleController.h"
#include "VehicleAction.h"
using namespace std;

// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}



int main() {
    uWS::Hub h;

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    WayPointsMap wayPointsMap;
    wayPointsMap.init(map_file_, max_s);

    int lane = 1;//lane 0 is far left, lane 1 is middle and we start with lane 1
    double ref_vel = INITIAL_VEL;

    h.onMessage([&ref_vel, &wayPointsMap, &lane](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    if(prev_size > 0) {
                        car_s = end_path_s;
                    }

                    // Check if the car in front is too close
                    bool too_close = false;
                    bool left_gap_check = false;
                    bool right_gap_check = false;
                    VehicleController vehicleController(sensor_fusion, lane);
                    too_close = vehicleController.is_too_close(prev_size, car_s, end_path_s);
                    cout << too_close << " "<< lane << endl;


                    VehicleAction vehicleAction = vehicleController.take_vehicle_action(ref_vel, too_close, prev_size, car_s);

                    ref_vel = vehicleAction.ref_vel;
                    lane = vehicleAction.lane;

                    json msgJson;

                    vector<double> ptsx;
                    vector<double> ptsy;
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    double dist_inc = 0.5;//distance increment
                    //prev path state is almost empty
                    if (prev_size < 2) {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    } else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    double double_d = 2 + 4 * lane;
                    vector<double> next_wp0 = getXY(car_s + 30, double_d, wayPointsMap._s, wayPointsMap._x,
                                                    wayPointsMap._y);
                    vector<double> next_wp1 = getXY(car_s + 60, double_d, wayPointsMap._s, wayPointsMap._x,
                                                    wayPointsMap._y);
                    vector<double> next_wp2 = getXY(car_s + 90, double_d, wayPointsMap._s, wayPointsMap._x,
                                                    wayPointsMap._y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);


                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);
                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    tk::spline s;
//                    for (int i = 0; i < ptsx.size(); ++i)
//                        std::cout << ptsx[i] << ' ';
//                    cout << endl;
//                    for (int i = 0; i < ptsy.size(); ++i)
//                        std::cout << ptsy[i] << ' ';
                    s.set_points(ptsx, ptsy);
                    cout << endl << endl;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back((previous_path_x[i]));
                        next_y_vals.push_back((previous_path_y[i]));
                    }

                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

                    double x_add_on = 0;

                    for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
                        double N = (target_dist / (.02 * ref_vel / 2.24));//2.24 is in m/s
                        double x_point = x_add_on + (target_x) / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        //convert to global coords
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

//                    for (int i = 0; i < 50; i++) {
//                        double next_s = car_s + (i + 1) * dist_inc;
//                        double next_d = 6;
//
//                        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, _x, map_waypoints_y);
//                        next_x_vals.push_back(xy[0]);
//                        next_y_vals.push_back(xy[1]);
//                    }

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
