#include "main.h"
#include <algorithm>
#include <cmath>
#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "distSensorUtil.hpp"
#include "customLCD.hpp"

bool clawClampState = true;
bool sideToggleState = false;
void on_center_button() {}

enum class ScoreState {
    idle,
    clampDelay,
    moveToSetpoint,
    atSetpoint,
    scoreReleaseDelay,
    scoreRaiseClear,
    scoreWaitForward,
    scoreLowerReset
};

ScoreState scoreState = ScoreState::idle;
int setpointIndex = 0;
double scoreTargetHeight = liftSetpointIn(0);
bool sequenceStarted = false;
bool scorePending = false;
uint32_t scoreStateStartTime = 0;
double forwardStartPos = 0;
ScoreState afterReleaseState = ScoreState::scoreRaiseClear;

bool isScoringModeActive() {
    return sequenceStarted;
}

void handleL1Press() {
    if (!sequenceStarted) {
        sequenceStarted = true;
        setpointIndex = 0;
        scoreStateStartTime = pros::millis();
        scoreState = ScoreState::clampDelay;
    } else if (setpointIndex < numLiftSetpoints - 1) {
        setpointIndex++;
    }
}

void handleL2Press() {
    if (!sequenceStarted) return;
    if (setpointIndex > 0) {
        setpointIndex--;
    }
}

void handleScorePress() {
    if (!sequenceStarted) return;
    scorePending = true;
}

void autoClampTask(void* param) {
    float ddd = autoClampSensor.get();

    if(ddd >= 110) {
        while(true) {
            driveLeftMotors.move(-127);
            driveRightMotors.move(-127);
            float idkfah = autoClampSensor.get();
            if(idkfah <= 100) {
                clawClampState = true;
                driveLeftMotors.move(0);
                driveRightMotors.move(0);
                break;
            }
        }
    } else if(ddd <= 110 && ddd >= 90) {
        while(true) {
            driveLeftMotors.move(-127);
            driveRightMotors.move(-127);
            float idkfah = autoClampSensor.get();
            if(idkfah <= 75) {
                clawClampState = true;
                driveLeftMotors.move(0);
                driveRightMotors.move(0);
                break;
            }
        }
    }else if(ddd <= 90 && ddd >= 70) {
        while(true) {
            driveLeftMotors.move(-127);
            driveRightMotors.move(-127);
            float idkfah = autoClampSensor.get();
            if(idkfah <= 60) {
                clawClampState = true;
                driveLeftMotors.move(0);
                driveRightMotors.move(0);
                break;
            }
        }
    }
}

void scoringTaskLoop(void* param) {
    while (true) {
        if (autonRunning || lcdMotorTestActive) {
            pros::delay(10);
            continue;
        }

        uint32_t now = pros::millis();

        switch (scoreState) {
        case ScoreState::idle:
            elevator.move(0);
            break;

        case ScoreState::clampDelay:
            if (now - scoreStateStartTime >= clampDelayMs) {
                scoreState = ScoreState::moveToSetpoint;
            }
            break;

        case ScoreState::moveToSetpoint:
            scoreTargetHeight = liftSetpointIn(setpointIndex);
            driveLiftTowards(scoreTargetHeight);
            if (liftSettledAt(scoreTargetHeight)) {
                scoreState = ScoreState::atSetpoint;
            }
            break;

        case ScoreState::atSetpoint:
            if (!liftSettledAt(liftSetpointIn(setpointIndex))) {
                scoreState = ScoreState::moveToSetpoint;
                break;
            }
            elevator.move(0);
            if (scorePending) {
                scorePending = false;
                endEffectorPiston.set_value(false);
                scoreStateStartTime = now;
                if (getLiftHeightInches() + scoreRaiseDeltaIn > maxLiftHeightIn) {
                    afterReleaseState = ScoreState::scoreWaitForward;
                } else {
                    afterReleaseState = ScoreState::scoreRaiseClear;
                }
                scoreState = ScoreState::scoreReleaseDelay;
            }
            break;

        case ScoreState::scoreReleaseDelay:
            elevator.move(0);
            if (now - scoreStateStartTime >= clawReleaseDelayMs) {
                if (afterReleaseState == ScoreState::scoreRaiseClear) {
                    scoreTargetHeight = getLiftHeightInches() + scoreRaiseDeltaIn;
                } else {
                    forwardStartPos = verticalRotation.get_position();
                }
                scoreState = afterReleaseState;
            }
            break;

        case ScoreState::scoreRaiseClear:
            driveLiftTowards(scoreTargetHeight);
            if (liftSettledAt(scoreTargetHeight)) {
                scoreState = ScoreState::scoreLowerReset;
            }
            break;

        case ScoreState::scoreWaitForward: {
            double degreesTravelled = std::fabs(verticalRotation.get_position() - forwardStartPos) / 100.0;
            double inchesTravelled = (degreesTravelled / 360.0) * (M_PI * 2.0);
            if (inchesTravelled >= forwardClearDistanceIn) {
                scoreState = ScoreState::scoreLowerReset;
            }
            break;
        }

        case ScoreState::scoreLowerReset:
            resetLiftToBottom();
            elevator.move(0);
            sequenceStarted = false;
            scorePending = false;
            setpointIndex = 0;
            scoreState = ScoreState::idle;
            break;
        }

        pros::delay(10);
    }
}

void lcdTask(void* param) {
    while (true) {
        updateUI();
        pros::delay(150);
    }
}

void initialize() {
    initUI();
    pros::Task lcdTaskHandle(lcdTask, nullptr, "LCD Task");
    initializeGlobals();
    //pros::Task mclTaskHandle(mclTaskLoop, nullptr, "MCL Task");

    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator.tare_position_all();

    endEffectorPiston.set_value(true);
    sideTogglePiston.set_value(false);
    sideTogglePiston2.set_value(false);
    //pros::delay(20);
    //Paths::goToSetpoint(1);
}

void disabled() {
    while (true) {
        endEffectorPiston.set_value(false);
        pros::delay(20);
    }
}
void competition_initialize() {}

void autonomous() {
    Paths::runAutonomous();
}

void opcontrol() {
    pros::Task scoringTask(scoringTaskLoop, nullptr, "Scoring Task");

    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X) * 0.67;

        if (abs(forward) < 20) forward = 0;
        if (abs(turn) < 20) turn = 0;

        if(master.get_digital_new_press(DIGITAL_UP)) {
            useLowSetpoints = !useLowSetpoints;
        }

        if (master.get_digital_new_press(DIGITAL_L1)) {
            handleL1Press();
        }

        if (master.get_digital_new_press(DIGITAL_L2)) {
            handleL2Press();
        }

        if (master.get_digital_new_press(DIGITAL_R1)) {
            clawClampState = !clawClampState;
        }

        if(master.get_digital_new_press(DIGITAL_B)) {
            sideToggleState = true;
        }

        if(master.get_digital_new_release(DIGITAL_B)) {
            sideToggleState = false;
        }

        if (master.get_digital_new_press(DIGITAL_R2)) {
            sequenceStarted = false;
            scorePending = false;
            scoreState = ScoreState::scoreLowerReset;
        }

        if (master.get_digital_new_press(DIGITAL_Y)) {
            Paths::runAutonomous();
        }

        if(master.get_digital_new_press(DIGITAL_DOWN)) {
            pros::Task autoClampTaskDo(autoClampTask);
        }

        if (!lcdMotorTestActive) {
            driveLeftMotors.move(std::clamp(forward + turn, -127, 127));
            driveRightMotors.move(std::clamp(forward - turn, -127, 127));
        }

        endEffectorPiston.set_value(clawClampState);
        sideTogglePiston.set_value(sideToggleState);
        sideTogglePiston2.set_value(sideToggleState);
        pros::delay(20);
    }
}
