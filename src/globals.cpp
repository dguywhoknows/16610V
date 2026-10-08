#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <string>
#include <cmath>
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

using namespace lemlib;

ControllerSettings lateralSettings(15.0, 0.0, 22.0, 3.0, 0.5, 100.0, 1.50, 500.0, 0.0);
ControllerSettings angularSettings(4.0, 0.0, 26.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup elevator({-5, -8, 19, 13}, pros::MotorGears::blue);

pros::Imu imu(17);
pros::Rotation verticalRotation(-6);
pros::Rotation horizontalRotation(-15);

pros::Distance distBack(20);
pros::Distance distLeft(14);
pros::Distance distRight(9);
pros::Distance autoClampSensor(4);

pros::adi::DigitalOut endEffectorPiston('A');
pros::adi::DigitalOut sideTogglePiston('B');
pros::adi::DigitalOut sideTogglePiston2('H');

pros::MotorGroup driveLeftMotors({-11, -16}, pros::MotorGears::blue);
pros::MotorGroup driveRightMotors({1, 7}, pros::MotorGears::blue);
pros::MotorGroup fullDrive({-11, -16, 1, 7}, pros::MotorGears::blue);

constexpr double driveWheelDiameter = Omniwheel::NEW_275;
constexpr double odomWheelDiameter = Omniwheel::NEW_2;
constexpr double trackingWidth = 11.92;

int currentStartingPos = 4;
bool autonRunning = false;
bool useLowSetpoints = true;
bool fullLiftPowerAuton = false;
bool lcdMotorTestActive = false;

static TrackingWheel verticalWheel(&verticalRotation, odomWheelDiameter, 0);
static TrackingWheel horizontalWheel(&horizontalRotation, odomWheelDiameter, -0.25);

static OdomSensors sensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
static Drivetrain drivetrain(&driveLeftMotors, &driveRightMotors, trackingWidth, driveWheelDiameter, 450.0, 2.0);

lemlib::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors);

void initializeGlobals() {
    chassis.calibrate();
    driveLeftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveRightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveLeftMotors.tare_position();
    driveRightMotors.tare_position();
    imu.tare_heading();
    imu.tare_rotation();
    imu.reset(true);
    verticalRotation.reset();
    horizontalRotation.reset();
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}

double getLiftHeightInches() {
    double rotations = elevator.get_position() / 360.0;
    return minLiftHeightIn + rotations * liftInchesPerRotation;
}

bool liftSettledAt(double inches) {
    return std::fabs(getLiftHeightInches() - inches) <= liftPositionToleranceIn;
}

void driveLiftTowards(double inches) {
    double error = inches - getLiftHeightInches();
    double drivePower = fullLiftPowerAuton ? liftFullPower : (autonRunning ? liftDrivePowerAuton : liftDrivePower);
    double power = 0;
    if (error > liftPositionToleranceIn) {
        power = drivePower;
    } else if (error < -liftPositionToleranceIn) {
        power = fullLiftPowerAuton ? liftResetDownPowerAuton : -drivePower;
    }
    elevator.move(power);
}

static uint32_t liftStallConditionStartTime = 0;

bool liftIsStalled() {
    bool conditionMet = std::fabs(elevator.get_efficiency()) <= liftStallEfficiencyThreshold;

    if (!conditionMet) {
        liftStallConditionStartTime = 0;
        return false;
    }

    if (liftStallConditionStartTime == 0) {
        liftStallConditionStartTime = pros::millis();
        return false;
    }

    return (pros::millis() - liftStallConditionStartTime) >= liftStallDebounceMs;
}

void resetLiftToBottom() {
    liftStallConditionStartTime = 0;
    double downPower = autonRunning ? liftResetDownPowerAuton : liftResetDownPower;
    while (!liftIsStalled()) {
        elevator.move(downPower);
        pros::delay(10);
    }
    elevator.move(0);
    elevator.tare_position_all();
}
