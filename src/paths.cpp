#include "globals.hpp"
#include "paths.hpp"
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

        RetractParams retractC = {1};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C->D");
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-63.43, lemlib::DriveSide::RIGHT, 800, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);

        chassis.moveToPoint(-24, -24, 1000, {.forwards = true});
        endEffectorPiston.set_value(true);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.turnToHeading(0, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        transitionC.join();
        scoreAtGoal();

        RetractParams retractD = {1};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D->E");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        transitionD.join();
        scoreAtGoal();

        RetractParams retractE = {2};
        pros::Task transitionE(retractAndTransitionTask, &retractE, "Transition E->F");
        chassis.moveToPoint(-24, -48, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.turnToHeading(0, 500, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-24, -24, 600, {.forwards = true}, false);
        pros::delay(20);
        chassis.turnToHeading(-45, 400, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-48, 0, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(180, 500, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        transitionE.join();
        scoreAtGoal();
        pros::delay(20);

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

        RetractParams retractC = {1};
        pros::Task transitionC(retractAndTransitionTask, &retractC, "Transition C->D");
        chassis.moveToPoint(-48, 24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(153.43, lemlib::DriveSide::LEFT, 800, {.maxSpeed = 127}, false);
        intakeLift1.set_value(true);
        pros::delay(20);

        chassis.moveToPoint(-24, -24, 1000, {.forwards = true});
        endEffectorPiston.set_value(true);
        pros::delay(20);

        intakeLift1.set_value(false);
        chassis.turnToHeading(90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);

        transitionC.join();
        scoreAtGoal();

        RetractParams retractD = {1};
        pros::Task transitionD(retractAndTransitionTask, &retractD, "Transition D->E");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 127}, false);
        pros::delay(20);

        intakeLift1.set_value(true);
        chassis.moveToPoint(-48, -48, 700, {.forwards = true, .earlyExitRange = 2});
        endEffectorPiston.set_value(true);
        int delay3 = 300;
        pros::Task drop3(intakeDropTask, &delay3, "Intake Drop 3");
        transitionD.join();
        scoreAtGoal();

        RetractParams retractE = {2};
        pros::Task transitionE(retractAndTransitionTask, &retractE, "Transition E->F");
        chassis.moveToPoint(-48, -24, 700, {.forwards = false}, false);
        pros::delay(20);

        chassis.turnToHeading(90, 500, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(-24, -24, 600, {.forwards = true}, false);
        pros::delay(20);
        chassis.turnToHeading(135, 400, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(0, -48, 800, {.forwards = true}, false);
        pros::delay(20);

        chassis.turnToHeading(-90, 400, {.maxSpeed = 127}, false);
        pros::delay(20);
        chassis.moveToPoint(24, -48, 700, {.forwards = false}, false);
        transitionE.join();
        scoreAtGoal();
        pros::delay(20);

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
