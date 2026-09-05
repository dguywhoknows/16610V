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

// Drive PID gains. Order is kP, kI, kD, anti-windup, small-error range, small-error
// timeout, large-error range, large-error timeout, slew.
ControllerSettings lateralSettings(20.0, 0.0, 17.0, 3.0, 0.75, 100.0, 2.0, 500.0, 0.0);
ControllerSettings angularSettings(6.0, 0.0, 21.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Single intake motor on port 11, reversed.
pros::Motor intake(-11, pros::MotorGears::blue);

// Lift runs on three motors (ports 5, 8, and 19). Ports 5 and 8 are reversed so
// all three pull the lift the same direction.
pros::MotorGroup elevator({-5, -8, 19}, pros::MotorGears::blue);

pros::Imu imu(17);
pros::Rotation verticalRotation(-6);
pros::Rotation horizontalRotation(15);
pros::Distance intakeDetection(14);

pros::adi::DigitalOut intakeLift1('D');
pros::adi::DigitalOut endEffectorPiston('B');
pros::adi::DigitalOut scoringPiston('A');

// Left side on ports 12 and 16 (reversed), right side on ports 1 and 7.
pros::MotorGroup driveLeftMotors({-12, -16}, pros::MotorGears::blue);
pros::MotorGroup driveRightMotors({1, 7}, pros::MotorGears::blue);
pros::MotorGroup fullDrive({-12, -16, 1, 7}, pros::MotorGears::blue);

constexpr double driveWheelDiameter = Omniwheel::NEW_275; // 2.75" drive wheels
constexpr double odomWheelDiameter = Omniwheel::NEW_2;    // 2" tracking wheels
constexpr double trackingWidth = 11.92;                   // distance between the left and right wheels

//int currentPage = 0;
//std::string allianceColor = "RED";
//bool controllerEnabled = true;
int currentStartingPos = 3;

/*
std::vector<std::vector<std::vector<double>>> autonPaths = {
    {{0,0},{20,10},{40,40},{60,20}},
    {{10,10},{30,50},{70,30},{90,10}},
    {{5,60},{25,40},{45,80},{80,60}},
    {{10,20},{20,30},{40,10},{60,50}},
    {{0,80},{30,60},{60,80},{90,40}},
    {{20,20},{40,20},{60,40},{80,60}},
    {{10,90},{30,70},{50,90},{70,70}},
    {{0,40},{20,60},{40,20},{60,40}}
};
*/

// Tracking wheels. The horizontal wheel sits 3.2" behind the tracking center.
static TrackingWheel verticalWheel(&verticalRotation, odomWheelDiameter, 0);
static TrackingWheel horizontalWheel(&horizontalRotation, odomWheelDiameter, -3.2);

// Odometry uses one vertical and one horizontal tracking wheel plus the IMU.
static OdomSensors sensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
static Drivetrain drivetrain(&driveLeftMotors, &driveRightMotors, trackingWidth, driveWheelDiameter, 450.0, 2.0);

lemlib::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors);

void initializeGlobals() {
    chassis.calibrate();
    //driveLeftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    //driveRightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    driveLeftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveRightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveLeftMotors.tare_position();
    driveRightMotors.tare_position();
    imu.tare_heading();
    imu.tare_rotation();
    verticalRotation.reset();
    horizontalRotation.reset();
    // Hold brake keeps the lift from sagging under its own weight.
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}

// Converts the lift encoder reading into a height in inches above the floor.
double getLiftHeightInches() {
    double rotations = elevator.get_position() / 360.0;
    return MIN_LIFT_HEIGHT_IN + rotations * LIFT_INCHES_PER_ROTATION;
}

// True when the lift is within the tolerance band of the requested height.
bool liftSettledAt(double inches) {
    return std::fabs(getLiftHeightInches() - inches) <= LIFT_POSITION_TOLERANCE_IN;
}

// Bang-bang control: full power up or down until we are inside the tolerance band,
// then coast. Call this repeatedly in a loop.
void driveLiftTowards(double inches) {
    double error = inches - getLiftHeightInches();
    double power = 0;
    if (error > LIFT_POSITION_TOLERANCE_IN) {
        power = LIFT_DRIVE_POWER;
    } else if (error < -LIFT_POSITION_TOLERANCE_IN) {
        power = -LIFT_DRIVE_POWER;
    }
    elevator.move(power);
}

// Timestamp of when the lift first looked stalled, or 0 if it currently does not.
static uint32_t liftStallConditionStartTime = 0;

// The lift is "stalled" once its efficiency has sat near zero for long enough,
// which means it is pushing against the bottom hard stop.
bool liftIsStalled() {
    bool conditionMet = std::fabs(elevator.get_efficiency()) <= LIFT_STALL_EFFICIENCY_THRESHOLD;

    if (!conditionMet) {
        liftStallConditionStartTime = 0;
        return false;
    }

    if (liftStallConditionStartTime == 0) {
        liftStallConditionStartTime = pros::millis();
        return false;
    }

    return (pros::millis() - liftStallConditionStartTime) >= LIFT_STALL_DEBOUNCE_MS;
}

// Drives the lift down until it stalls on the hard stop, then zeroes the encoder
// so the next height reading starts from a known point.
void resetLiftToBottom() {
    liftStallConditionStartTime = 0;
    while (!liftIsStalled()) {
        elevator.move(LIFT_RESET_DOWN_POWER);
        pros::delay(10);
    }
    elevator.move(0);
    elevator.tare_position_all();
}
