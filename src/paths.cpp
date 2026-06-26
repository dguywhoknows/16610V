#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <math.h>

namespace Paths {
    /*
    void redLeftQuadLeft() {
        chassis.cancelMotion();
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        chassis.setPose(-63, 10, 90, false);
        pros::delay(20);

        chassis.moveToPoint(-65, 10, 300, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPose(-62, 10, 45, 500, {.forwards = true, .minSpeed = 80, .earlyExitRange = 2}, false);
        chassis.moveToPose(-61, 15, -10, 500, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPose(-58.5, 14.5, 222.14, 700, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2}, false);
        chassis.moveToPoint(-48, 24, 500, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 400, {.minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, 24, 800, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 800, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, 24, 800, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 800, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPose(-55.5, 37, 20, 500, {.forwards = true, .minSpeed = 80, .earlyExitRange = 2}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 800, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPose(-35, 16.5, 70, 500, {.forwards = true, .minSpeed = 80, .earlyExitRange = 2}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -24, 1200, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 400, {.minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, -24, 800, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -24, 800, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, -24, 800, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(0, 0, 1400, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 400, {.minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-13, 0, 400, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.turnToHeading(-45, 400, {.minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-23, 10, 700, {.forwards = true, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(-13, 0, 400, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 400, {.minSpeed = 80}, false);
        pros::delay(20);

        chassis.moveToPoint(0, 0, 600, {.forwards = false, .minSpeed = 80}, false);
        pros::delay(20);
    }
        */
    void path4() { //if you somehow tune the shorter versions and have time for ts make the timeouts shorter
        chassis.cancelMotion();
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        chassis.setPose(-63, 10, 90, false);
        pros::delay(20);
        intakeMotor1.move(127);
        intakeMotor2.move(127);

        chassis.moveToPoint(-65, 10, 700, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(-10, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-63.88, 15, 1000, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(226.97, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-65, 24, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);
        scoringPiston.set_value(false);

        chassis.moveToPoint(-58, 24, 800, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-48, 24, 900, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(153.43, lemlib::DriveSide::LEFT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-24, -24, 3000, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(200);
        scoringPiston.set_value(false);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        intakeMotor1.move(-127);
        intakeMotor2.move(-127);
        pros::delay(500);

        chassis.turnToHeading(90, 400, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -24, 1500, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-65, -24, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);
        scoringPiston.set_value(false);

        chassis.moveToPoint(-58, -24, 800, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-48, 24, 900, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);
    }

    void path3() { //no last pin :( also make timeouts shorter if this is out of time
        chassis.cancelMotion();
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        chassis.setPose(-63, 10, 90, false);
        pros::delay(20);
        intakeMotor1.move(127);
        intakeMotor2.move(127);

        chassis.moveToPoint(-65, 10, 700, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(-10, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-63.88, 15, 1000, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(226.97, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-65, 24, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);
        scoringPiston.set_value(false);

        chassis.moveToPoint(-58, 24, 800, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-48, 24, 900, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(153.43, lemlib::DriveSide::LEFT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-24, -24, 3000, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(200);
        scoringPiston.set_value(false);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        intakeMotor1.move(-127);
        intakeMotor2.move(-127);
        pros::delay(500);

        chassis.turnToHeading(90, 400, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -24, 1500, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);
    }

    void path2() { //no 2nd goal :( start with this after red and if its SOMEHOW out of time then make the timeouts shorter
        chassis.cancelMotion();
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        chassis.setPose(-63, 10, 90, false);
        pros::delay(20);
        intakeMotor1.move(127);
        intakeMotor2.move(127);

        chassis.moveToPoint(-65, 10, 700, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(-10, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-63.88, 15, 1000, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-63, 10, 700, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(226.97, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-65, 24, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);
        scoringPiston.set_value(false);

        chassis.moveToPoint(-58, 24, 800, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-48, 24, 900, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);
    }

    void path1() { //red path, start with this
        chassis.cancelMotion();
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        chassis.setPose(-63, -10, 90, false);
        pros::delay(20);
        intakeMotor1.move(127);
        intakeMotor2.move(127);

        chassis.moveToPoint(-65, -10, 700, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -10, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);

        chassis.turnToHeading(0, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, -24, 1200, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);
        
        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);

        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 700, {.minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        intakeLift2.set_value(false);

        chassis.moveToPoint(-65, -24, 1500, {.forwards = true, .minSpeed = 40}, false);
        pros::delay(20);
        scoringPiston.set_value(false);

        chassis.moveToPoint(-58, -24, 800, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        intakeLift2.set_value(true);

        chassis.moveToPoint(-48, -24, 900, {.forwards = false, .minSpeed = 40}, false);
        pros::delay(20);

        endEffectorPiston.set_value(true);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(500);

        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(700);
        endEffectorPiston.set_value(false);
        liftMotor.move(0);
    }

    void runAutonomous() {
        if (currentStartingPos == 0) {
            path1();
        } else if (currentStartingPos == 1) {
            path2();
        } else if (currentStartingPos == 2) {
            path3();
        } else if (currentStartingPos == 3) {
            path4();
        }
    }
}
