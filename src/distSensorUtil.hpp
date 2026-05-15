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
    
    // Convenience constructor
    Particle(lemlib::Pose p, double w) : pose(p), weight(w) {}
};

bool correct_position(dist_sensor sensor, lemlib::Chassis *chassis, bool is_x_wall, bool forced = false, double correct_rate = 10);

void amcl_init(lemlib::Pose initial_pose, int num_particles = 500);
void amcl_update(double dx, double dy, double dtheta); 
void amcl_sense(std::vector<dist_sensor>& sensors);
lemlib::Pose amcl_get_estimated_pose();

#endif