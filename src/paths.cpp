#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <math.h>

namespace Paths {
    void intakeDropTask(void* param) {
        int theDelay = *static_cast<int*>(param);
        pros::delay(theDelay);
        intakeLift1.set_value(true);
        endEffectorPiston.set_value(true);
    }

    struct scoringParams {
        int uptime;
        int downtime;
        int delay;
    };

    void scoringFunction(void* param) {
        scoringParams* params = static_cast<scoringParams*>(param);
        int uptime = params->uptime;
        int downtime = params->downtime;
        int delay = params->delay;
        pros::delay(delay);
        intakeMotors.move(127);
        liftIntakePTO.set_value(true);
        liftMotor.move(127);
        scoringPiston.set_value(true);
        pros::delay(uptime);
        liftIntakePTO.set_value(false);
        liftMotor.move(-127);
        pros::delay(downtime);
        liftMotor.move(0);
        endEffectorPiston.set_value(false);
        intakeMotors.move(127);
    }

    void SAWP1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intakeMotors.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(false);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(true);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        scoringParams params1 = {300, 100, 400};
        pros::Task scoring1(scoringFunction, &params1, "Scoring Function 1");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        scoringParams params2 = {400, 100, 300};
        pros::Task scoring2(scoringFunction, &params2, "Scoring Function 2");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(0, -48, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(112.5, 400, {.earlyExitRange = 10});
        chassis.moveToPoint(24, -60, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(24, -65, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        scoringParams params3 = {400, 100, 300};
        pros::Task scoring3(scoringFunction, &params3, "Scoring Function 3");
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-63.43, lemlib::DriveSide::RIGHT, 800, {.maxSpeed = 127}, false);
        intakeLift1.set_value(false);
        pros::delay(20);

        chassis.moveToPoint(-24, -24, 1000, {.forwards = true});
        endEffectorPiston.set_value(true);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        scoringParams params4 = {500, 100, 200};
        pros::Task scoring4(scoringFunction, &params4, "Scoring Function 4");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        scoringParams params5 = {600, 100, 100};
        pros::Task scoring5(scoringFunction, &params5, "Scoring Function 5");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(-24, -24, 600, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(-45, 300, {.earlyExitRange = 5});
        chassis.moveToPoint(-48, 0, 800, {.forwards = true, .earlyExitRange = 3.5});
        chassis.turnToHeading(-90, 300, {.maxSpeed = 127}, false);
        intakeLift1.set_value(false);
        pros::delay(20);

        chassis.moveToPoint(-63, 0, 400, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-58, 0, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, 0, 300, {.forwards = false}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        pros::delay(20);
    }

    void SAWP2() {

    }

    void shortened1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intakeMotors.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(false);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(true);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        scoringParams params1 = {300, 100, 400};
        pros::Task scoring1(scoringFunction, &params1, "Scoring Function 1");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        scoringParams params2 = {400, 100, 300};
        pros::Task scoring2(scoringFunction, &params2, "Scoring Function 2");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(0, -48, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(112.5, 400, {.earlyExitRange = 10});
        chassis.moveToPoint(24, -60, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(24, -65, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        scoringParams params3 = {400, 100, 300};
        pros::Task scoring3(scoringFunction, &params3, "Scoring Function 3");
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void shortened2() {

    }

    void line1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intakeMotors.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(false);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(true);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        scoringParams params1 = {300, 100, 400};
        pros::Task scoring1(scoringFunction, &params1, "Scoring Function 1");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        scoringParams params2 = {400, 100, 300};
        pros::Task scoring2(scoringFunction, &params2, "Scoring Function 2");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        scoringParams params5 = {600, 100, 100};
        pros::Task scoring5(scoringFunction, &params5, "Scoring Function 5");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);
        
        intakeLift1.set_value(false);
        chassis.moveToPoint(-24, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay4 = 300;
        pros::Task drop4(intakeDropTask, &delay4, "Intake Drop 4");
        scoringParams params6 = {600, 100, 100};
        pros::Task scoring6(scoringFunction, &params6, "Scoring Function 6");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void line2() {

    }

    void skillsPath() {

    }

    void runAutonomous() {
        if(currentStartingPos == 0) {
            shortened1();
        } else if(currentStartingPos == 1) {
            shortened2();
        } else if(currentStartingPos == 2) {
            SAWP1();
        } else if(currentStartingPos == 3) {
            SAWP2();
        } else if(currentStartingPos == 4) {
            line1();
        } else if(currentStartingPos == 5) {
            line2();
        } else if(currentStartingPos == 6) {
            skillsPath();
        }
    }
}