#ifndef DISTSENSORUTIL_HPP
#define DISTSENSORUTIL_HPP

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include <vector>

struct dist_sensor {
    pros::Distance* sensor;
    lemlib::Pose offset;
};

struct DistTaskParams {
    std::vector<dist_sensor> sensors;
    bool use_sensor[3] = {true, true, true};
    lemlib::Chassis* chassis;
};

void correctPoseFromDistSensors(DistTaskParams* params);

extern DistTaskParams wallResetParams;

void resetPoseFromWalls(bool back, bool left, bool right);

//extern bool mclEnabled;
//void mclTaskLoop(void* param);

#endif
