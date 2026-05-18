#include "main.h"
#include <algorithm>
#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "distSensorUtil.hpp"

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	initializeGlobals();
    pros::Task mclTask([]{
        pros::delay(2000);
        std::vector<dist_sensor> mcl_sensors = {
            {distanceSensor1, lemlib::Pose(0, 5, 0)},
            {distanceSensor2, lemlib::Pose(-5, 0, 270)},
            {distanceSensor3, lemlib::Pose(5, 0, 90)}
        };
        lemlib::Pose lastPose = chassis.getPose();
        mcl_init(lastPose);
        const int delay_ms = 20;
        while (true) {
            lemlib::Pose currentPose = chassis.getPose();
            double dx = currentPose.x - lastPose.x;
            double dy = currentPose.y - lastPose.y;
            double dtheta = currentPose.theta - lastPose.theta;
            double distance_moved = sqrt((dx * dx) + (dy * dy));
            double current_speed = distance_moved / (delay_ms / 1000.0);
            mcl_update(dx, dy, dtheta);
            mcl_sense(mcl_sensors);
            lemlib::Pose fusedPose = mcl_get_fused_pose(currentPose, current_speed);
            chassis.setPose(fusedPose.x, fusedPose.y, fusedPose.theta);
            lastPose = chassis.getPose(); 
            pros::delay(delay_ms);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    Paths::runAutonomous();
}

void opcontrol() {
    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X) * 0.7;
        
        if (abs(forward) < 20) forward = 0;
        if (abs(turn) < 20) turn = 0;
 
        driveLeftMotors.move(std::clamp(forward + turn, -127, 127));
        driveRightMotors.move(std::clamp(forward - turn, -127, 127));

        if (master.get_digital_new_press(DIGITAL_Y)) {
            Paths::runAutonomous();
        }
    }
}

