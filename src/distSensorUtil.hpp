#ifndef DISTSENSORUTIL_HPP
#define DISTSENSORUTIL_HPP

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include <vector>

struct dist_sensor {
    pros::Distance &sensor;
    lemlib::Pose offset;
};

struct Particle {
    lemlib::Pose pose;
    double weight;
    Particle() : pose(0, 0, 0), weight(0.0) {}
    Particle(lemlib::Pose p, double w) : pose(p), weight(w) {}
};

bool correct_position(dist_sensor sensor, lemlib::Chassis *chassis, bool is_x_wall, bool forced = false, double correct_rate = 10);
void mcl_init(lemlib::Pose initial_pose, int num_particles = 75);
void mcl_update(double dx, double dy, double dtheta); 
void mcl_sense(std::vector<dist_sensor>& sensors);
lemlib::Pose mcl_get_estimated_pose();
lemlib::Pose mcl_get_fused_pose(lemlib::Pose odomPose, double current_speed);
void mcl_sync_with_chassis(lemlib::Chassis *chassis, double current_speed);

#endif
