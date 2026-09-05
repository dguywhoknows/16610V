#ifndef DISTSENSORUTIL_HPP
#define DISTSENSORUTIL_HPP

// Helper for nudging the odometry pose using side facing distance sensors that
// range off the perimeter walls of the field.

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include <vector>

// One distance sensor and where it sits on the robot relative to the tracking
// center. offset.theta is the direction the sensor faces, in degrees.
struct dist_sensor {
    pros::Distance* sensor;
    lemlib::Pose offset;
};

// Everything the correction pass needs: the sensors to use, a per sensor enable
// flag, and the chassis whose pose gets corrected.
struct DistTaskParams {
    std::vector<dist_sensor> sensors;
    bool use_sensor[4] = {true, true, true, true};
    lemlib::Chassis* chassis;
};

// Reads the sensors once and, if the walls disagree with odometry by more than a
// couple inches, snaps the pose x and/or y to match. Heading is left alone.
void correctPoseFromDistSensors(DistTaskParams* params);

/*
// Parked alternative: a full Monte Carlo localization implementation that fuses
// the same distance sensors against a wall model. Left here for reference.
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
*/

#endif
