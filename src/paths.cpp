#include "globals.hpp"
#include "paths.hpp"
#include "distSensorUtil.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <math.h>
#include <cmath>

namespace Paths {
    void intakeDropTask(void* param) {
        int theDelay = *static_cast<int*>(param);
        pros::delay(theDelay);
        intakeLift1.set_value(false);
        endEffectorPiston.set_value(true);
    }

    double heightForPins(int pins) {
        int index = pins - 1;
        if (index < 0) index = 0;
        if (index > NUM_LIFT_SETPOINTS - 1) index = NUM_LIFT_SETPOINTS - 1;
        return LIFT_SETPOINTS_IN[index];
    }

    void raiseForPins(int pins) {
        double targetHeight = heightForPins(pins);

        while (!liftSettledAt(WRIST_CLEAR_HEIGHT_IN)) {
            driveLiftTowards(WRIST_CLEAR_HEIGHT_IN);
            pros::delay(10);
        }
        elevator.move(0);

        scoringPiston.set_value(true);
        pros::delay(WRIST_EXTEND_DELAY_MS);

        while (!liftSettledAt(targetHeight)) {
            driveLiftTowards(targetHeight);
            pros::delay(10);
        }
        elevator.move(0);
    }

    void raiseElevatorTask(void* param) {
        int pins = *static_cast<int*>(param);
        raiseForPins(pins);
    }

    void scoreAtGoal() {
        endEffectorPiston.set_value(false);
        pros::delay(CLAW_RELEASE_DELAY_MS);
    }

    struct RetractParams {
        int nextPins;
    };

    void retractAndTransitionTask(void* param) {
        RetractParams* params = static_cast<RetractParams*>(param);

        double currentHeight = getLiftHeightInches();
        if (currentHeight + SCORE_RAISE_DELTA_IN > MAX_LIFT_HEIGHT_IN) {
            double startPos = verticalRotation.get_position();
            double inchesTravelled = 0;
            while (inchesTravelled < FORWARD_CLEAR_DISTANCE_IN) {
                double degreesTravelled = std::fabs(verticalRotation.get_position() - startPos) / 100.0;
                inchesTravelled = (degreesTravelled / 360.0) * (M_PI * 2.0);
                pros::delay(10);
            }
        } else {
            double clearTarget = currentHeight + SCORE_RAISE_DELTA_IN;
            while (!liftSettledAt(clearTarget)) {
                driveLiftTowards(clearTarget);
                pros::delay(10);
            }
            elevator.move(0);
        }

        scoringPiston.set_value(false);
        pros::delay(WRIST_RETRACT_DELAY_MS);

        resetLiftToBottom();
        elevator.move(0);

        if (params->nextPins > 0) {
            raiseForPins(params->nextPins);
        }
    }

    void SAWP1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {2};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->C");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(0, -48, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(112.5, 400, {.earlyExitRange = 4});
        chassis.moveToPoint(24, -60, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(24, -65, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        transitionB.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(true, true, false, true);

        RetractParams retractC = {1};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C->D");
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 800, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(0, -12, 800, {.forwards = true, .earlyExitRange = 4});
        chassis.moveToPoint(-48, -12, 800, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);

        chassis.moveToPoint(-48, -48, 800, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.turnToHeading(90, 400, {.maxSpeed = 127}, false);
        resetPoseFromWalls(false, true, true, true);
        chassis.moveToPoint(-24, -48, 700, {.forwards = true, .earlyExitRange = 2}, false);
        transitionC.join();
        scoreAtGoal();

        RetractParams retractD = {1};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D->F");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.turnToHeading(0, 500, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);
        chassis.moveToPoint(-24, -24, 700, {.forwards = true, .earlyExitRange = 2}, false);
        endEffectorPiston.set_value(true);
        int delay4 = 300;
        pros::Task drop4(intakeDropTask, &delay4, "Intake Drop 4");
        pros::delay(20);
        intakeLift1.set_value(false);

        chassis.turnToHeading(-45, 400, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-48, 0, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(180, 500, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        transitionD.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(true, true, true, false);

        RetractParams retractF = {0};
        pros::Task transitionF(retractAndTransitionTask, &retractF, "Transition F (final)");
        pros::delay(20);

        chassis.moveToPoint(-48, 0, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(90, 500, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-69, 0, 600, {.forwards = false}, false);
        pros::delay(20);
        chassis.moveToPoint(-60, 0, 400, {.forwards = true}, false);
        chassis.moveToPoint(-69, 0, 400, {.forwards = false}, false);
        pros::delay(20);
        resetPoseFromWalls(true, true, true, true);

        intakeLift1.set_value(false);
        pros::delay(20);
    }

    void SAWP2() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-63, -9, 90, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-48, -9, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(0, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {2};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->C");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 0, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(-22.5, 400, {.earlyExitRange = 4});
        chassis.moveToPoint(-60, 24, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, 24, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        transitionB.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(true, true, true, false);

        RetractParams retractC = {1};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C->D");
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 800, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-12, 0, 800, {.forwards = true, .earlyExitRange = 4});
        chassis.moveToPoint(-12, -48, 800, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);

        chassis.moveToPoint(-48, -48, 800, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        resetPoseFromWalls(false, true, true, true);
        chassis.moveToPoint(-48, -24, 700, {.forwards = true, .earlyExitRange = 2}, false);
        transitionC.join();
        scoreAtGoal();

        RetractParams retractD = {1};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D->F");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.turnToHeading(90, 500, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);
        chassis.moveToPoint(-24, -24, 700, {.forwards = true, .earlyExitRange = 2}, false);
        endEffectorPiston.set_value(true);
        int delay4 = 300;
        pros::Task drop4(intakeDropTask, &delay4, "Intake Drop 4");
        pros::delay(20);
        intakeLift1.set_value(false);

        chassis.turnToHeading(135, 400, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(0, -48, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 500, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        transitionD.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(true, true, false, true);

        RetractParams retractF = {0};
        pros::Task transitionF(retractAndTransitionTask, &retractF, "Transition F (final)");
        pros::delay(20);

        chassis.moveToPoint(0, -48, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(90, 500, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-69, -48, 600, {.forwards = false}, false);
        pros::delay(20);
        chassis.moveToPoint(-60, -48, 400, {.forwards = true}, false);
        chassis.moveToPoint(-69, -48, 400, {.forwards = false}, false);
        pros::delay(20);
        resetPoseFromWalls(true, true, true, true);

        intakeLift1.set_value(false);
        pros::delay(20);
    }

    void shortened1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {2};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->C");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(0, -48, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(112.5, 400, {.earlyExitRange = 4});
        chassis.moveToPoint(24, -60, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(24, -65, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        transitionB.join();
        scoreAtGoal();

        RetractParams retractC = {0};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C (final)");
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void shortened2() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-63, -9, 90, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-48, -9, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(0, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {2};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->C");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        chassis.moveToPoint(-48, 0, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(-22.5, 400, {.earlyExitRange = 4});
        chassis.moveToPoint(-60, 24, 700, {.forwards = true, .earlyExitRange = 2});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, 24, 500, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay2 = 300;
        pros::Task drop2(intakeDropTask, &delay2, "Intake Drop 2");
        transitionB.join();
        scoreAtGoal();

        RetractParams retractC = {0};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C (final)");
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void line1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-9, -58, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-9, -63, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-9, -48, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -65, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {1};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->E");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        transitionB.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractE = {1};
        pros::Task transitionE(retractAndTransitionTask, &retractE, "Transition E->D");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay4 = 300;
        pros::Task drop4(intakeDropTask, &delay4, "Intake Drop 4");
        transitionE.join();
        scoreAtGoal();

        RetractParams retractD = {0};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D (final)");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void line2() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-63, -9, 90, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);
        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        intakeLift1.set_value(true);
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        chassis.moveToPoint(-58, -9, 300, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -9, 300, {.forwards = false}, false);
        pros::delay(20);

        int pinsA = 1;
        pros::Task raiseA(raiseElevatorTask, &pinsA, "Raise A");
        chassis.moveToPoint(-48, -9, 500, {.forwards = true, .earlyExitRange = 4});
        intakeLift1.set_value(false);
        chassis.turnToHeading(0, 300, {.earlyExitRange = 10});
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raiseA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractA = {2};
        pros::Task transitionA(retractAndTransitionTask, &retractA, "Transition A->B");
        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 600, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay1 = 300;
        pros::Task drop1(intakeDropTask, &delay1, "Intake Drop 1");
        transitionA.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractB = {1};
        pros::Task transitionB(retractAndTransitionTask, &retractB, "Transition B->E");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        transitionB.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retractE = {1};
        pros::Task transitionE(retractAndTransitionTask, &retractE, "Transition E->D");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay4 = 300;
        pros::Task drop4(intakeDropTask, &delay4, "Intake Drop 4");
        transitionE.join();
        scoreAtGoal();

        RetractParams retractD = {0};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D (final)");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);
    }

    void skillsPath() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-58, 47, 90, false);
        pros::delay(20);

        endEffectorPiston.set_value(false);
        intake.move(127);

        chassis.turnToHeading(-90, 400, {.earlyExitRange = 8});
        chassis.moveToPoint(-63, 40, 600, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-66, 40, 350, {.forwards = true}, false);
        pros::delay(300);
        chassis.moveToPoint(-56, 40, 400, {.forwards = false}, false);
        pros::delay(20);
        resetPoseFromWalls(true, false, true, true);

        int pins2 = 1;
        pros::Task raise2(raiseElevatorTask, &pins2, "Raise 2");
        chassis.moveToPoint(-48, 6, 700, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise2.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract3 = {0};
        pros::Task transition3(retractAndTransitionTask, &retract3, "Transition 2->3");
        chassis.moveToPoint(-48, 10, 500, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, 24, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Drop 3");
        transition3.join();

        int pins3 = 2;
        pros::Task raise3(raiseElevatorTask, &pins3, "Raise 3");
        chassis.moveToPoint(-48, 6, 700, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise3.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(true, true, false, true);

        RetractParams retract4a = {0};
        pros::Task transition4a(retractAndTransitionTask, &retract4a, "Transition 3->4a");
        chassis.moveToPoint(-48, 8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, 12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, 12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, 12, 400, {.forwards = false}, false);
        transition4a.join();

        int pins4a = 3;
        pros::Task raise4a(raiseElevatorTask, &pins4a, "Raise 4a");
        chassis.turnToHeading(20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, 6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise4a.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract4b = {0};
        pros::Task transition4b(retractAndTransitionTask, &retract4b, "Transition 4a->4b");
        chassis.moveToPoint(-48, 8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, 12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, 12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, 12, 400, {.forwards = false}, false);
        transition4b.join();

        int pins4b = 4;
        pros::Task raise4b(raiseElevatorTask, &pins4b, "Raise 4b");
        chassis.turnToHeading(20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, 6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise4b.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract4c = {0};
        pros::Task transition4c(retractAndTransitionTask, &retract4c, "Transition 4b->4c");
        chassis.moveToPoint(-48, 8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, 12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, 12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, 12, 400, {.forwards = false}, false);
        transition4c.join();

        int pins4c = 5;
        pros::Task raise4c(raiseElevatorTask, &pins4c, "Raise 4c");
        chassis.turnToHeading(20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, 6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise4c.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract5 = {0};
        pros::Task transition5(retractAndTransitionTask, &retract5, "Transition 4c->5");
        chassis.moveToPoint(-48, 8, 500, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(-27, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, 48, 900, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay5 = 300;
        pros::Task drop5(intakeDropTask, &delay5, "Drop 5");
        transition5.join();

        int pins5 = 6;
        pros::Task raise5(raiseElevatorTask, &pins5, "Raise 5");
        chassis.moveToPoint(-48, 6, 900, {.forwards = false, .earlyExitRange = 4});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise5.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract6 = {0};
        pros::Task transition6(retractAndTransitionTask, &retract6, "Transition 5->6");
        chassis.moveToPoint(-48, 10, 500, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(70, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, 24, 900, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay6 = 300;
        pros::Task drop6(intakeDropTask, &delay6, "Drop 6");
        transition6.join();

        int pins6 = 7;
        pros::Task raise6(raiseElevatorTask, &pins6, "Raise 6");
        chassis.moveToPoint(-40, 10, 700, {.forwards = false, .earlyExitRange = 4});
        chassis.moveToPoint(-48, 6, 500, {.forwards = false, .earlyExitRange = 3});
        chassis.turnToHeading(180, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        raise6.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract7 = {0};
        pros::Task transition7(retractAndTransitionTask, &retract7, "Transition 6->7");
        chassis.moveToPoint(-48, 6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(135, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-24, -24, 900, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay7 = 300;
        pros::Task drop7(intakeDropTask, &delay7, "Drop 7");
        transition7.join();

        int pins7 = 1;
        pros::Task raise7(raiseElevatorTask, &pins7, "Raise 7");
        chassis.turnToHeading(115, 400, {.earlyExitRange = 6});
        chassis.moveToPoint(-6, -10, 700, {.forwards = false}, false);
        raise7.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract8 = {0};
        pros::Task transition8(retractAndTransitionTask, &retract8, "Transition 7->8");
        chassis.moveToPoint(-30, -14, 700, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-65, -24, 1000, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay8 = 300;
        pros::Task drop8(intakeDropTask, &delay8, "Drop 8");
        transition8.join();

        int pins8 = 1;
        pros::Task raise8(raiseElevatorTask, &pins8, "Raise 8");
        chassis.moveToPoint(-48, -6, 700, {.forwards = false, .earlyExitRange = 3});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise8.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(false, true, true, true);

        RetractParams retract9a = {0};
        pros::Task transition9a(retractAndTransitionTask, &retract9a, "Transition 8->9a");
        chassis.moveToPoint(-48, -8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, -12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, -12, 400, {.forwards = false}, false);
        transition9a.join();

        int pins9a = 2;
        pros::Task raise9a(raiseElevatorTask, &pins9a, "Raise 9a");
        chassis.turnToHeading(-20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, -6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise9a.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract9b = {0};
        pros::Task transition9b(retractAndTransitionTask, &retract9b, "Transition 9a->9b");
        chassis.moveToPoint(-48, -8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, -12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, -12, 400, {.forwards = false}, false);
        transition9b.join();

        int pins9b = 3;
        pros::Task raise9b(raiseElevatorTask, &pins9b, "Raise 9b");
        chassis.turnToHeading(-20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, -6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise9b.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract9c = {0};
        pros::Task transition9c(retractAndTransitionTask, &retract9c, "Transition 9b->9c");
        chassis.moveToPoint(-48, -8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, -12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, -12, 400, {.forwards = false}, false);
        transition9c.join();

        int pins9c = 4;
        pros::Task raise9c(raiseElevatorTask, &pins9c, "Raise 9c");
        chassis.turnToHeading(-20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, -6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise9c.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract9d = {0};
        pros::Task transition9d(retractAndTransitionTask, &retract9d, "Transition 9c->9d");
        chassis.moveToPoint(-48, -8, 600, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 350, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-57, -12, 500, {.forwards = true, .earlyExitRange = 1.5});
        chassis.moveToPoint(-63, -12, 450, {.forwards = true}, false);
        pros::delay(200);
        resetPoseFromWalls(true, false, true, true);
        chassis.moveToPoint(-56, -12, 400, {.forwards = false}, false);
        transition9d.join();

        int pins9d = 5;
        pros::Task raise9d(raiseElevatorTask, &pins9d, "Raise 9d");
        chassis.turnToHeading(-20, 350, {.earlyExitRange = 8});
        chassis.moveToPoint(-48, -6, 600, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise9d.join();
        scoreAtGoal();
        pros::delay(20);

        RetractParams retract10 = {0};
        pros::Task transition10(retractAndTransitionTask, &retract10, "Transition 9d->10");
        chassis.moveToPoint(-48, -10, 500, {.forwards = true, .earlyExitRange = 3});
        chassis.turnToHeading(-135, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, -48, 900, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay10 = 300;
        pros::Task drop10(intakeDropTask, &delay10, "Drop 10");
        transition10.join();

        int pins10 = 6;
        pros::Task raise10(raiseElevatorTask, &pins10, "Raise 10");
        chassis.moveToPoint(-48, -6, 900, {.forwards = false, .earlyExitRange = 4});
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        raise10.join();
        scoreAtGoal();
        pros::delay(20);
        resetPoseFromWalls(false, true, true, true);

        RetractParams retractPark = {0};
        pros::Task transitionPark(retractAndTransitionTask, &retractPark, "Transition 10->park");
        chassis.moveToPoint(-48, -6, 700, {.forwards = true, .earlyExitRange = 4});
        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        chassis.moveToPoint(-63, 0, 900, {.forwards = true}, false);
        resetPoseFromWalls(true, true, true, true);
        transitionPark.join();
        intake.move(0);
        pros::delay(20);
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
