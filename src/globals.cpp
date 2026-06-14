#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <string>
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

using namespace lemlib;

// --- Controller Settings (Definitions) ---
lemlib::ControllerSettings lateralSettings(7.0, 0.1, 8.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
lemlib::ControllerSettings angularSettings(2.5, 0.0, 17.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);

// --- Motor Definitions ---
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor something1(1); 
pros::Motor something2(1);
pros::Motor something3(1);
pros::Motor something4(1);

// --- Sensor Definitions ---
pros::Imu imu(9);
pros::Rotation verticalRotation(7);
pros::Rotation horizontalRotation(8);
pros::Rotation liftSensor(1);
pros::Distance distanceSensor1(1);
pros::Distance distanceSensor2(1);
pros::Distance distanceSensor3(1);
pros::Distance distanceSensor4(1);
pros::Distance distanceSensor5(1);
pros::Optical opticalSensor1(1);
pros::Optical opticalSensor2(1);

// --- Pneumatic Definitions ---
pros::adi::DigitalOut something5('A');
pros::adi::DigitalOut something6('A');
pros::adi::DigitalOut something7('A');

pros::Motor leftMotor1(-1); pros::Motor leftMotor2(-2); pros::Motor leftMotor3(-3);
pros::Motor rightMotor1(4); pros::Motor rightMotor2(5); pros::Motor rightMotor3(6);

pros::MotorGroup driveLeftMotors({-1, -2, -3});
pros::MotorGroup driveRightMotors({4, 5, 6});
pros::MotorGroup fullDrive({-1, -2, -3, 4, 5, 6});

constexpr double driveWheelDiameter = 1.0;
constexpr double odomWheelDiameter = 1.0;
constexpr double trackingWidth = 1.0;

int currentPage = 0;
std::string allianceColor = "RED";
bool controllerEnabled = true;
int currentStartingPos = 4;

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

static TrackingWheel verticalWheel(&verticalRotation, odomWheelDiameter, 0.5);
static TrackingWheel horizontalWheel(&horizontalRotation, odomWheelDiameter, -4.0);

static OdomSensors sensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
static Drivetrain drivetrain(&driveLeftMotors, &driveRightMotors, trackingWidth, driveWheelDiameter, 360.0, 8.0);

lemlib::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors);

void initializeGlobals() {
    chassis.calibrate();
    driveLeftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    driveLeftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveRightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    driveLeftMotors.tare_position();
    driveRightMotors.tare_position();
    imu.tare_heading();
    imu.tare_rotation();
    verticalRotation.reset();
}

