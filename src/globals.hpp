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

extern pros::Controller master;

extern pros::MotorGroup driveLeftMotors;
extern pros::MotorGroup driveRightMotors;
extern pros::MotorGroup fullDrive;

extern pros::MotorGroup elevator;

extern pros::Imu imu;
extern pros::Rotation verticalRotation;
extern pros::Rotation horizontalRotation;

extern pros::Distance distBack;
extern pros::Distance distLeft;
extern pros::Distance distRight;
extern pros::Distance autoClampSensor;

extern pros::adi::DigitalOut endEffectorPiston;
extern pros::adi::DigitalOut sideTogglePiston;
extern pros::adi::DigitalOut sideTogglePiston2;

extern lemlib::ControllerSettings lateralSettings;
extern lemlib::ControllerSettings angularSettings;

extern lemlib::Chassis chassis;

extern int currentStartingPos;
extern bool autonRunning;
extern bool fullLiftPowerAuton;
extern bool lcdMotorTestActive;
extern void initializeGlobals();

constexpr double liftSetpointsIn[] = {7.3, 14.3, 19.7, 25.5, 31.5, 37.5, 43.5, 50};
constexpr double liftSetpointsLowIn[] = {7.3, 12.1, 17.85, 23.55, 28.95, 34.95, 40.45, 47.45};
extern bool useLowSetpoints;
inline double liftSetpointIn(int index) { return useLowSetpoints ? liftSetpointsLowIn[index] : liftSetpointsIn[index]; }
constexpr int numLiftSetpoints = 7;

constexpr double minLiftHeightIn = 3.5;
constexpr double maxLiftHeightIn = 50.0;
constexpr double liftInchesPerRotation = 6.6;
constexpr double scoreRaiseDeltaIn = 4.0;
constexpr double forwardClearDistanceIn = 6.0;
constexpr double liftPositionToleranceIn = 0.5;
constexpr double liftFullPower = 127.0;
constexpr double liftDrivePower = 127.0;
constexpr double liftDrivePowerAuton = 100.0;
constexpr double liftResetDownPower = -80.0;
constexpr double liftResetDownPowerAuton = -80.0;
constexpr double liftStallEfficiencyThreshold = 0.03;
constexpr uint32_t clampDelayMs = 100;
constexpr uint32_t clawReleaseDelayMs = 300;
constexpr uint32_t liftStallDebounceMs = 300;

double getLiftHeightInches();
bool liftSettledAt(double inches);
void driveLiftTowards(double inches);
bool liftIsStalled();
void resetLiftToBottom();

#endif