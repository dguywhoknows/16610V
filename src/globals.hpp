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
extern pros::Rotation verticalRotation;
extern pros::Rotation horizontalRotation;
extern pros::Controller master;
extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;
extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;
extern pros::Motor intakeMotor1; 
extern pros::Motor intakeMotor2;
extern pros::Motor liftMotor;
extern pros::MotorGroup driveLeftMotors;
extern pros::MotorGroup driveRightMotors;
extern pros::MotorGroup fullDrive;

extern lemlib::ControllerSettings lateralSettings;
extern lemlib::ControllerSettings angularSettings;

extern pros::Imu imu;
extern pros::Rotation verticalRotation;
extern pros::Rotation horizontalRotation;
extern pros::Distance distanceSensor1;
extern pros::Distance distanceSensor2;
extern pros::Distance distanceSensor3;
extern pros::Distance distanceSensor4;
extern pros::Distance distanceSensor5;
extern pros::Optical opticalSensor1;
extern pros::Optical opticalSensor2;

extern pros::adi::DigitalOut intakeLift1;
extern pros::adi::DigitalOut intakeLift2;
extern pros::adi::DigitalOut liftIntakePTO;
extern pros::adi::DigitalOut endEffectorPiston;
extern pros::adi::DigitalOut colorSorterPiston;
extern pros::adi::DigitalOut scoringPiston;

extern lemlib::Chassis chassis;

extern int currentPage;
extern std::string allianceColor;
extern bool controllerEnabled;
extern int currentStartingPos; // 0-3, auton selection index
extern std::vector<std::vector<std::vector<double>>> autonPaths;

extern void initializeGlobals();

#endif
