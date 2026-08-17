#include "main.h"
#include <algorithm>
#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "distSensorUtil.hpp"

void on_center_button() {}

void updateLCD(void* param) {
    while(true) {
        int vertRaw = verticalRotation.get_position();
        int horzRaw = horizontalRotation.get_position();
        double imuDeg = imu.get_heading();
        lemlib::Pose pose = chassis.getPose(false);
        pros::lcd::print(0, "Vert: %d  Horz: %d", vertRaw, horzRaw);
        pros::lcd::print(1, "IMU: %.1f deg", imuDeg);
        pros::lcd::print(2, "X: %.2f  Y: %.2f", pose.x, pose.y);
        pros::lcd::print(3, "Theta: %.1f deg", pose.theta);
        pros::lcd::print(4, "Starting Pos: %d", currentStartingPos);
        pros::lcd::print(5, "Autons: 0: Normal top/bottom");
        pros::lcd::print(6, "1: Normal left/right (clr -> right)");
        pros::lcd::print(7, "2: SAWP top/bottom");
        pros::lcd::print(8, "3: SAWP left/right, 4: skills");
        pros::delay(20);
    }
}

void initialize() {
    pros::lcd::initialize();
    pros::Task LCD_update_task(updateLCD, nullptr, "LCD Update Task");
    initializeGlobals();
    intakeLift1.set_value(true);
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
    bool runningIntakeForLift = false;
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

        if(runningIntake || runningIntakeForLift) {
            intakePower = 127;
        } else {
            intakePower = 0;
        }

        if(master.get_digital_new_press(DIGITAL_L1)) {
            liftIntakePTOState = false;
            liftPower = 127;
            intakePower = 127;
            runningIntakeForLift = true;
        }

        if(master.get_digital_new_release(DIGITAL_L1)) {
            liftIntakePTOState = true;
            pros::delay(50);
            runningIntakeForLift = false;
        }

        bool wasRunningIntake = runningIntake;

        if(master.get_digital_new_press(DIGITAL_L2)) {
            liftMotor.move(-127);
            if(!runningIntake) {
                liftIntakePTOState = false;
                intakePower = -127;
            }
            if(master.get_digital_new_press(DIGITAL_R1)) {
                liftIntakePTOState = true;
                pros::delay(50);
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
            intakeMotors.move(127);
            liftIntakePTO.set_value(true);
            liftMotor.move(127);
            scoringPiston.set_value(true);
            pros::delay(500);
            liftIntakePTO.set_value(false);
            liftMotor.move(-127);
            pros::delay(800);
            liftMotor.move(0);
            if(!runningIntake) {
                intakeMotors.move(0);
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

        intakeMotors.move(intakePower);
        liftMotor.move(liftPower);
        intakeLift1.set_value(intakeLiftState);
        liftIntakePTO.set_value(liftIntakePTOState);
        endEffectorPiston.set_value(endEffectorState);
        colorSorterPiston.set_value(colorSorterPistonState);
        scoringPiston.set_value(scoringPistonState);

        pros::delay(20);
    }
}