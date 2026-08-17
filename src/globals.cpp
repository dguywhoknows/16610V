#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <string>
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

using namespace lemlib;

// --- Controller Settings (Definitions) --- TUNE THE PID ITS NOT TUNED YET, https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html
ControllerSettings lateralSettings(7.0, 0.1, 8.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
ControllerSettings angularSettings(2.5, 0.0, 17.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);

// --- Motor Definitions ---
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intakeMotor1(-7, pros::MotorGears::blue);
pros::Motor intakeMotor2(17, pros::MotorGears::blue);
pros::Motor liftMotor(6, pros::MotorGears::blue);

// --- Sensor Definitions ---
pros::Imu imu(9);
pros::Rotation verticalRotation(-4);
pros::Rotation horizontalRotation(15);
pros::Distance intakeDetection(3);
pros::Distance distanceSensor2(8);
pros::Distance distanceSensor3(9);
pros::Distance distanceSensor4(10);
pros::Distance distanceSensor5(13);
pros::Optical opticalSensor1(18);
pros::Optical opticalSensor2(19);

// --- Pneumatic Definitions ---
pros::adi::DigitalOut intakeLift1('D');
pros::adi::DigitalOut liftIntakePTO('E');
pros::adi::DigitalOut endEffectorPiston('B');
pros::adi::DigitalOut scoringPiston('A');

pros::Motor leftMotor1(-11, pros::MotorGears::blue); pros::Motor leftMotor2(-12, pros::MotorGears::blue); pros::Motor leftMotor3(-16, pros::MotorGears::blue);
pros::Motor rightMotor1(1, pros::MotorGears::blue); pros::Motor rightMotor2(2, pros::MotorGears::blue); pros::Motor rightMotor3(5, pros::MotorGears::blue);

pros::MotorGroup driveLeftMotors({-11, -12, -16}, pros::MotorGears::blue);
pros::MotorGroup driveRightMotors({1, 2, 5}, pros::MotorGears::blue);
pros::MotorGroup fullDrive({-11, -12, -16, 1, 2, 5}, pros::MotorGears::blue);
pros::MotorGroup intakeMotors({-7, 17}, pros::MotorGears::blue);

constexpr double driveWheelDiameter = Omniwheel::NEW_275;
constexpr double odomWheelDiameter = Omniwheel::NEW_2;
constexpr double trackingWidth = 11.92;

//int currentPage = 0;
//std::string allianceColor = "RED";
//bool controllerEnabled = true;
int currentStartingPos = 0;

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

static TrackingWheel verticalWheel(&verticalRotation, odomWheelDiameter, 0);
static TrackingWheel horizontalWheel(&horizontalRotation, odomWheelDiameter, -3.2);

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
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
