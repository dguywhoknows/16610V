#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include <vector>
#include <string>

// --- Motors ---
extern pros::Controller master;
extern pros::MotorGroup driveLeftMotors;
extern pros::MotorGroup driveRightMotors;

extern lemlib::ControllerSettings lateralSettings(7.0, 0.1, 8.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
extern lemlib::ControllerSettings angularSettings(2.5, 0.0, 17.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);

// --- Sensors ---
extern pros::Imu imu;
extern pros::Rotation verticalRotation;
extern pros::Rotation horizontalRotation;
extern pros::Distance distanceSensor1;
extern pros::Distance distanceSensor2;
extern pros::Distance distanceSensor3;
extern pros::Optical opticalSensor1;

extern pros::adi::DigitalOut something5;
extern pros::adi::DigitalOut something6;
extern pros::adi::DigitalOut something7;

extern lemlib::Chassis chassis;

extern int currentPage;
extern std::string allianceColor;
extern bool controllerEnabled;
extern int currentStartingPos; // 0-7, master selection index
extern std::vector<std::vector<std::vector<double>>> autonPaths;

void initializeGlobals();

#endif