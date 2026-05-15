#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <string>

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
pros::Imu imu(1);
pros::Rotation verticalRotation(1);
pros::Rotation horizontalRotation(1);
pros::Distance distanceSensor1(1);
pros::Distance distanceSensor2(1);
pros::Distance distanceSensor3(1);
pros::Optical opticalSensor1(1);

// --- Pneumatic Definitions ---
pros::adi::DigitalOut something5('A');
pros::adi::DigitalOut something6('A');
pros::adi::DigitalOut something7('A');

constexpr int leftPort1 = 1; constexpr int leftPort2 = 1; constexpr int leftPort3 = 1;
constexpr int rightPort1 = 1; constexpr int rightPort2 = 1; constexpr int rightPort3 = 1;

pros::MotorGroup driveLeftMotors({leftPort1, leftPort2, leftPort3});
pros::MotorGroup driveRightMotors({rightPort1, rightPort2, rightPort3});

constexpr double driveWheelDiameter = 1.0;
constexpr double odomWheelDiameter = 1.0;
constexpr double trackingWidth = 1.0;

int currentPage = 0; // 0 = Home, 1 = Auton
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