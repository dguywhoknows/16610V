#include "main.h"          // pulls in all core PROS APIs (motors, sensors, tasks, etc.)
#include <algorithm>        // gives us std::clamp so we can cap motor values to [-127, 127]
#include "globals.hpp"      // our own file, declares every motor, sensor, and chassis object
#include "paths.hpp"        // our own file, declares Paths::runAutonomous()
#include "lemlib/chassis/chassis.hpp" // LemLib library, chassis class for movement and odometry
#include "pros/adi.hpp"     // PROS API for 3-wire (ADI) ports, used for pneumatics
#include "pros/motors.hpp"  // PROS API for V5 motors, move(), get_position(), etc.
#include "distSensorUtil.hpp" // our own file, MCL particle filter functions (mcl_init, mcl_update, etc.)

void on_center_button() {} // called when the center LCD button is pressed — left empty, no action needed

void initialize() {                         // PROS calls this once at startup before anything else runs
    pros::lcd::initialize();                // turns on the V5 brain's LCD screen so we can print debug info
    initializeGlobals();                    // runs our setup code: calibrates chassis, zeros encoders, etc.

    pros::Task mclTask([]{                  // creates a new background RTOS task that runs the MCL localization loop
        pros::delay(2000);                  // waits 2 seconds before starting MCL — gives the IMU time to calibrate

        std::vector<dist_sensor> mcl_sensors = { // creates a list of the distance sensors MCL will use
            {distanceSensor1, lemlib::Pose(0, 5, 0)},    // front sensor: 5 inches forward of center, pointing forward (0°)
            {distanceSensor2, lemlib::Pose(-5, 0, 270)}, // left sensor: 5 inches left of center, pointing left (270°)
            {distanceSensor3, lemlib::Pose(5, 0, 90)}    // right sensor: 5 inches right of center, pointing right (90°)
        };

        lemlib::Pose lastPose = chassis.getPose(); // saves the robot's starting pose so we can compute how far it moves each loop
        mcl_init(lastPose);                        // initializes the particle filter with N particles spread around the starting pose

        const int delay_ms = 20; // MCL loop runs every 20 milliseconds = 50 times per second

        while (true) {                             // infinite loop - keeps running for the entire match
            lemlib::Pose currentPose = chassis.getPose(); // reads the current odometry pose (x, y, heading) from LemLib

            double dx = currentPose.x - lastPose.x;            // how far the robot moved in X since the last loop
            double dy = currentPose.y - lastPose.y;            // how far the robot moved in Y since the last loop
            double dtheta = currentPose.theta - lastPose.theta; // how much the robot turned since the last loop

            double distance_moved = sqrt((dx * dx) + (dy * dy)); // Pythagorean theorem - total distance traveled this cycle (inches)
            double current_speed = distance_moved / (delay_ms / 1000.0); // speed = distance / time - inches per second

            mcl_update(dx, dy, dtheta);            // PREDICT step: moves all particles using the odometry delta + random noise
            mcl_sense(mcl_sensors);                // SENSE + RESAMPLE step: weights particles by sensor readings, resamples the set

            lemlib::Pose fusedPose = mcl_get_fused_pose(currentPose, current_speed); // blends odometry + MCL estimate into one pose
            chassis.setPose(fusedPose.x, fusedPose.y, fusedPose.theta); // pushes the blended pose back into LemLib's odometry system

            lastPose = chassis.getPose(); // updates lastPose for next iteration (re-reads from chassis to stay in sync)
            pros::delay(delay_ms);        // yields the task for 20 ms so other tasks (drive, auton) can run
        }
    });
}

void disabled() {}               // called when the robot is disabled by field control, left empty, PROS stops motors automatically
void competition_initialize() {} // called right before autonomous at competitions, left empty, nothing extra needed

void autonomous() {         // called at the start of the 15-second autonomous period
    Paths::runAutonomous(); // hands off to our autonomous routine defined in paths.cpp
}

void opcontrol() {   // called at the start of driver control or by default when the code is normally run in non-competition mode, loops forever
    while (true) {   // infinite loop, keeps reading controller and driving until the match ends

        int forward = master.get_analog(ANALOG_LEFT_Y);      // reads left stick Y axis: positive = forward, negative = backward, range -127 to 127
        int turn = master.get_analog(ANALOG_RIGHT_X) * 0.7;  // reads right stick X axis scaled to 70%, reduces spin sensitivity

        if (abs(forward) < 20) forward = 0; // deadband: stick within 20 of center = treat as 0, prevents drift from resting thumb
        if (abs(turn) < 20) turn = 0;       // same deadband for the turn axis

        driveLeftMotors.move(std::clamp(forward + turn, -127, 127));  // arcade mix: left motors = forward + turn
        driveRightMotors.move(std::clamp(forward - turn, -127, 127)); // arcade mix: right motors = forward - turn
        // std::clamp keeps value inside [-127, 127] so we never send an out-of-range command

        if (master.get_digital_new_press(DIGITAL_Y)) { // checks if Y was JUST pressed this frame (not held, fires once per press)
            Paths::runAutonomous();                     // runs autonomous, useful for testing
        }
    }
}