#ifndef GLOBALS_HPP
#define GLOBALS_HPP

// Shared hardware objects, tuning constants, and lift helpers.
// Everything the rest of the code touches (chassis, motors, sensors, pistons)
// is declared here and defined once in globals.cpp.

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include <vector>
#include <string>

// Driver controller.
extern pros::Controller master;

// Drivetrain: two motors per side, plus a combined group for testing.
extern pros::MotorGroup driveLeftMotors;
extern pros::MotorGroup driveRightMotors;
extern pros::MotorGroup fullDrive;

// Intake is a single motor now. It used to share power with the lift through a
// pneumatic PTO, but the lift has its own motors so the two are fully separate.
extern pros::Motor intake;

// Lift / elevator: three dedicated motors driven together as one group.
extern pros::MotorGroup elevator;

// Odometry and field sensing.
extern pros::Imu imu;
extern pros::Rotation verticalRotation;
extern pros::Rotation horizontalRotation;
extern pros::Distance intakeDetection; // sees a pin and cup enter the intake during matchloading

// Perimeter distance sensors, one per side, used to reset odometry off the field
// walls. Offsets and enable flags live in distSensorUtil.
extern pros::Distance distFront;
extern pros::Distance distBack;
extern pros::Distance distLeft;
extern pros::Distance distRight;

// Pneumatics.
extern pros::adi::DigitalOut intakeLift1;       // raises and drops the intake / matchload arm
extern pros::adi::DigitalOut endEffectorPiston; // claw open / closed
extern pros::adi::DigitalOut scoringPiston;     // wrist extend / retract

// Drive PID gains, tuned on the brain screen and read back here.
extern lemlib::ControllerSettings lateralSettings;
extern lemlib::ControllerSettings angularSettings;

extern lemlib::Chassis chassis;

// Which starting position / auton routine is selected (set from the controller).
//extern int currentPage;
//extern std::string allianceColor;
//extern bool controllerEnabled;
extern int currentStartingPos;
//extern std::vector<std::vector<std::vector<double>>> autonPaths;

// Calibrates the IMU and zeroes every encoder. Call once at startup.
extern void initializeGlobals();

// Lift heights in inches for each number of pins stacked on the goal.
constexpr double LIFT_SETPOINTS_IN[] = {11.9, 18.0, 24.69, 31.38, 38.07, 44.76, 50};
constexpr int NUM_LIFT_SETPOINTS = 7;

constexpr double MIN_LIFT_HEIGHT_IN = 11.8;         // height when the lift is resting at the bottom
constexpr double MAX_LIFT_HEIGHT_IN = 50.0;         // don't command the lift past this
constexpr double LIFT_INCHES_PER_ROTATION = 6.6;    // travel per full turn of the lift motor
constexpr double WRIST_CLEAR_HEIGHT_IN = 25.0;      // lift must be at least this high before the wrist can swing out
constexpr double SCORE_RAISE_DELTA_IN = 4.0;        // how far to lift after releasing so the claw clears the stack
constexpr double FORWARD_CLEAR_DISTANCE_IN = 6.0;   // if already near max height, back away this far instead of lifting
constexpr double LIFT_POSITION_TOLERANCE_IN = 0.5;  // "close enough" band for the lift being at a target
constexpr double LIFT_DRIVE_POWER = 127.0;          // voltage used to drive the lift toward a setpoint
constexpr int LIFT_RESET_DOWN_POWER = -127;         // voltage used to push the lift down against its hard stop
constexpr double LIFT_STALL_EFFICIENCY_THRESHOLD = 0.03; // efficiency at or below this means the lift is pushing against the stop
constexpr uint32_t CLAMP_DELAY_MS = 100;            // wait after clamping the claw before moving the lift
constexpr uint32_t WRIST_EXTEND_DELAY_MS = 300;     // wait for the wrist to finish extending
constexpr uint32_t CLAW_RELEASE_DELAY_MS = 300;     // wait for the claw to finish opening
constexpr uint32_t WRIST_RETRACT_DELAY_MS = 300;    // wait for the wrist to finish retracting
constexpr uint32_t LIFT_STALL_DEBOUNCE_MS = 300;    // stall has to hold this long before we trust it

double getLiftHeightInches();          // current lift height, derived from the motor encoder
bool liftSettledAt(double inches);     // true when the lift is within tolerance of the given height
void driveLiftTowards(double inches);  // one step of bang-bang control toward a height
bool liftIsStalled();                  // true once the lift has been stalled long enough to count
void resetLiftToBottom();              // drive the lift down until it stalls, then zero the encoder

#endif
