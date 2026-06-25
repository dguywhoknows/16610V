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
pros::Motor intakeMotor1(1);
pros::Motor intakeMotor2(1);
pros::Motor liftMotor(1);

// --- Sensor Definitions ---
pros::Imu imu(9);
pros::Rotation verticalRotation(7);
pros::Rotation horizontalRotation(8);
pros::Distance intakeDetection(1);
pros::Distance distanceSensor2(1);
pros::Distance distanceSensor3(1);
pros::Distance distanceSensor4(1);
pros::Distance distanceSensor5(1);
pros::Optical opticalSensor1(1);
pros::Optical opticalSensor2(1);

// --- Pneumatic Definitions ---
pros::adi::DigitalOut intakeLift1('A');
pros::adi::DigitalOut intakeLift2('A');
pros::adi::DigitalOut liftIntakePTO('A');
pros::adi::DigitalOut endEffectorPiston('A');
pros::adi::DigitalOut colorSorterPiston('A');
pros::adi::DigitalOut scoringPiston('A');

pros::Motor leftMotor1(-1); pros::Motor leftMotor2(-2); pros::Motor leftMotor3(-3);
pros::Motor rightMotor1(4); pros::Motor rightMotor2(5); pros::Motor rightMotor3(6);

pros::MotorGroup driveLeftMotors({-1, -2, -3});
pros::MotorGroup driveRightMotors({4, 5, 6});
pros::MotorGroup fullDrive({-1, -2, -3, 4, 5, 6});

constexpr double driveWheelDiameter = Omniwheel::NEW_275;
constexpr double odomWheelDiameter = Omniwheel::NEW_2;
constexpr double trackingWidth = 11.9;

int currentPage = 0;
std::string allianceColor = "RED";
bool controllerEnabled = true;
int currentStartingPos = 0;

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
}
