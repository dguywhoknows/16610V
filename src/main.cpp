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
    pros::Task amclTask([]{
        pros::delay(2000); 

        std::vector<dist_sensor> amcl_sensors = {
            {distanceSensor1, lemlib::Pose(0, 5, 0)},
            {distanceSensor2, lemlib::Pose(-5, 0, 270)},
            {distanceSensor3, lemlib::Pose(5, 0, 90)}
        };

        lemlib::Pose lastPose = chassis.getPose();
        amcl_init(lastPose);

        while (true) {
            lemlib::Pose currentPose = chassis.getPose();
            
            amcl_update(currentPose.x - lastPose.x, currentPose.y - lastPose.y, currentPose.theta - lastPose.theta);
            
            amcl_sense(amcl_sensors);

            lastPose = currentPose;
            pros::delay(20);
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