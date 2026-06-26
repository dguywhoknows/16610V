#include "main.h"
#include <algorithm>
#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "distSensorUtil.hpp"

void on_center_button() {}

void initialize() {
    pros::lcd::initialize();
    initializeGlobals();
    intakeLift1.set_value(true);
    intakeLift2.set_value(true);
    liftIntakePTO.set_value(false);
    endEffectorPiston.set_value(false);
    colorSorterPiston.set_value(false);
    scoringPiston.set_value(false);
/*
    pros::Task mclTask([]{
        pros::delay(2000);

        std::vector<dist_sensor> mcl_sensors = {
            {distanceSensor1, lemlib::Pose(0, 5, 0)},
            {distanceSensor2, lemlib::Pose(-5, 0, 270)},
            {distanceSensor3, lemlib::Pose(5, 0, 90)}
        };

        lemlib::Pose lastPose = chassis.getPose();
        mcl_init(lastPose);

        const int delay_ms = 20;

        while (true) {
            lemlib::Pose currentPose = chassis.getPose();

            double dx = currentPose.x - lastPose.x;
            double dy = currentPose.y - lastPose.y;
            double dtheta = currentPose.theta - lastPose.theta;

            double distance_moved = sqrt((dx * dx) + (dy * dy));
            double current_speed = distance_moved / (delay_ms / 1000.0);

            mcl_update(dx, dy, dtheta);
            mcl_sense(mcl_sensors);

            lemlib::Pose fusedPose = mcl_get_fused_pose(currentPose, current_speed);
            chassis.setPose(fusedPose.x, fusedPose.y, fusedPose.theta);

            lastPose = chassis.getPose();
            pros::delay(delay_ms);
        }
    });
    */
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    Paths::runAutonomous();
}

void opcontrol() {
    bool intakeLiftState = true;
    bool liftIntakePTOState = false;
    bool endEffectorState = false;
    bool colorSorterPistonState = false;
    bool runningIntake = false;
    bool scoringPistonState = false;
    float intakePower = 0.0;
    float liftPower = 0.0;

    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X) * 0.85;

        if (abs(forward) < 20) forward = 0;
        if (abs(turn) < 20) turn = 0;

        if(master.get_digital_new_press(DIGITAL_R2)) {
            intakeLiftState = false;
        }
        if(master.get_digital_new_release(DIGITAL_R2)) {
            intakeLiftState = true;
        }

        if(master.get_digital_new_press(DIGITAL_R1)) {
            runningIntake = !runningIntake;
        }

        if(runningIntake) {
            intakePower = 127;
        } else {
            intakePower = 0;
        }

        if(master.get_digital_new_press(DIGITAL_L1)) {
            liftIntakePTOState = true;
            liftPower = 127;
            intakePower = 127;
        }

        if(master.get_digital_new_release(DIGITAL_L1)) {
            liftIntakePTOState = false;
        }

        bool wasRunningIntake = runningIntake;

        if(master.get_digital_new_press(DIGITAL_L2)) {
            liftMotor.move(-127);
            if(!runningIntake) {
                liftIntakePTOState = true;
                intakePower = -127;
            }
            if(master.get_digital_new_press(DIGITAL_R1)) {
                liftIntakePTOState = false;
                runningIntake = !runningIntake;
                intakePower = 127;
            }
        }

        if(master.get_digital_new_release(DIGITAL_L2) && !wasRunningIntake) {
            liftIntakePTOState = false;
        }

        if(master.get_digital_new_press(DIGITAL_B)) {
            endEffectorState = true;
        }
        
        if(master.get_digital_new_press(DIGITAL_DOWN)) {
            endEffectorState = false;
        }

        if(master.get_digital_new_press(DIGITAL_UP)) {
            endEffectorPiston.set_value(true);
            intakeMotor1.move(127);
            intakeMotor2.move(127);
            liftIntakePTO.set_value(true);
            liftMotor.move(127);
            scoringPiston.set_value(true);
            pros::delay(500);
            liftIntakePTO.set_value(false);
            liftMotor.move(-127);
            pros::delay(800);
            liftMotor.move(0);
            if(!runningIntake) {
                intakeMotor1.move(0);
                intakeMotor2.move(0);
            }
        }

        if(master.get_digital_new_press(DIGITAL_X)) {
            scoringPistonState = !scoringPistonState;
        }

        if(master.get_digital_new_press(DIGITAL_LEFT) && currentStartingPos > 0) {
            currentStartingPos -= 1;
        }

        if(master.get_digital_new_press(DIGITAL_RIGHT) && currentStartingPos < 3) {
            currentStartingPos += 1;
        }

        if (master.get_digital_new_press(DIGITAL_Y)) {
            Paths::runAutonomous();
        }

        driveLeftMotors.move(std::clamp(forward + turn, -127, 127));
        driveRightMotors.move(std::clamp(forward - turn, -127, 127));

        intakeMotor1.move(intakePower);
        intakeMotor2.move(intakePower);
        liftMotor.move(liftPower);
        intakeLift1.set_value(intakeLiftState);
        intakeLift2.set_value(intakeLiftState);
        liftIntakePTO.set_value(liftIntakePTOState);
        endEffectorPiston.set_value(endEffectorState);
        colorSorterPiston.set_value(colorSorterPistonState);
        scoringPiston.set_value(scoringPistonState);
    }
}