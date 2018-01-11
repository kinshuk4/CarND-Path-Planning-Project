//
// Created by Kinshuk Chandra on 27.12.17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#define MAX_VEL 49.5
#define ACCELERATION 0.224 //acceleration of 5 m/s^2
#define INITIAL_VEL 0.0
#define MAX_LANE 2 // 3 lane road
#define MIN_LANE 0

#define LANE_WIDTH 4.0 // 4 meters
#define SAFETY_MARGIN 30.0 //30 meters

enum LaneDecision {KEEP_LANE, TAKE_LEFT, TAKE_RIGHT};
#endif //PATH_PLANNING_CONSTANTS_H
