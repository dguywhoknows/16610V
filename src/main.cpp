#include "main.h"
#include <algorithm>
#include <cmath>
#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "distSensorUtil.hpp"

// Called when the brain's center button is pressed. Nothing bound to it.
void on_center_button() {}

bool intakeLiftState = true;   // true = intake arm raised, false = dropped
float intakePower = 0.0;       // voltage currently commanded to the intake
bool runningIntake = false;    // whether the driver has the intake toggled on

// The scoring routine is a small state machine. Each press of L1 arms it or bumps
// the target height up a pin; the "score" button drops the stack; the machine then
// clears the wrist, lowers, and re-zeroes the lift before going back to idle.
enum class ScoreState {
    IDLE,
    CLAMP_DELAY,               // claw just clamped, let it settle
    RAISE_TO_WRIST_CLEAR,      // lift up to the height where the wrist can swing out
    WRIST_EXTEND_DELAY,        // wait for the wrist to finish extending
    MOVE_TO_SETPOINT,          // drive the lift to the selected pin height
    AT_SETPOINT,               // holding at height, waiting for the score command
    SCORE_RELEASE_DELAY,       // claw opened, let the stack drop clear
    SCORE_RAISE_CLEAR,         // lift a bit more so the claw clears the stack
    SCORE_WAIT_FORWARD,        // near max height instead: drive forward to clear
    SCORE_RETRACT_WRIST,       // pull the wrist back in
    SCORE_WRIST_RETRACT_DELAY, // wait for the wrist to finish retracting
    SCORE_LOWER_RESET          // drive the lift back down and re-zero it
};

ScoreState scoreState = ScoreState::IDLE;
int setpointIndex = 0;                                  // which entry in LIFT_SETPOINTS_IN we are aiming for
double scoreTargetHeight = LIFT_SETPOINTS_IN[0];        // working target height, in inches
bool sequenceStarted = false;                           // true while the scoring routine owns the lift
bool scorePending = false;                              // set by the score button, consumed at AT_SETPOINT
uint32_t scoreStateStartTime = 0;                       // millis() when the current state began, for timed waits
double forwardStartPos = 0;                             // vertical encoder reading when SCORE_WAIT_FORWARD started
ScoreState afterReleaseState = ScoreState::SCORE_RAISE_CLEAR; // where to go after the claw release delay

// True while the scoring routine is running, so opcontrol leaves the lift and
// intake alone.
bool isScoringModeActive() {
    return sequenceStarted;
}

// L1: arm the routine if it is not running, otherwise step the target up one pin.
void handleL1Press() {
    if (!sequenceStarted) {
        sequenceStarted = true;
        setpointIndex = 0;
        endEffectorPiston.set_value(true);
        scoreStateStartTime = pros::millis();
        scoreState = ScoreState::CLAMP_DELAY;
    } else if (setpointIndex < NUM_LIFT_SETPOINTS - 1) {
        setpointIndex++;
    }
}

// L2: step the target down one pin (only while the routine is running).
void handleL2Press() {
    if (!sequenceStarted) return;
    if (setpointIndex > 0) {
        setpointIndex--;
    }
}

// Score button: ask the state machine to release the stack at the next chance.
void handleScorePress() {
    if (!sequenceStarted) return;
    scorePending = true;
}

// Background task that advances the scoring state machine every 10 ms.
void scoringTaskLoop(void* param) {
    while (true) {
        uint32_t now = pros::millis();

        switch (scoreState) {
        case ScoreState::IDLE:
            elevator.move(0);
            break;

        case ScoreState::CLAMP_DELAY:
            if (now - scoreStateStartTime >= CLAMP_DELAY_MS) {
                scoreState = ScoreState::RAISE_TO_WRIST_CLEAR;
            }
            break;

        case ScoreState::RAISE_TO_WRIST_CLEAR:
            driveLiftTowards(WRIST_CLEAR_HEIGHT_IN);
            if (liftSettledAt(WRIST_CLEAR_HEIGHT_IN)) {
                scoringPiston.set_value(true);
                scoreStateStartTime = now;
                scoreState = ScoreState::WRIST_EXTEND_DELAY;
            }
            break;

        case ScoreState::WRIST_EXTEND_DELAY:
            elevator.move(0);
            if (now - scoreStateStartTime >= WRIST_EXTEND_DELAY_MS) {
                scoreState = ScoreState::MOVE_TO_SETPOINT;
            }
            break;

        case ScoreState::MOVE_TO_SETPOINT:
            scoreTargetHeight = LIFT_SETPOINTS_IN[setpointIndex];
            driveLiftTowards(scoreTargetHeight);
            if (liftSettledAt(scoreTargetHeight)) {
                scoreState = ScoreState::AT_SETPOINT;
            }
            break;

        case ScoreState::AT_SETPOINT:
            // The driver may still be bumping the target up or down, so re-check
            // that we are actually at the current setpoint before holding.
            if (!liftSettledAt(LIFT_SETPOINTS_IN[setpointIndex])) {
                scoreState = ScoreState::MOVE_TO_SETPOINT;
                break;
            }
            elevator.move(0);
            if (scorePending) {
                scorePending = false;
                endEffectorPiston.set_value(false);
                scoreStateStartTime = now;
                // If lifting further would exceed the max height, drive the robot
                // forward to clear the stack instead.
                if (getLiftHeightInches() + SCORE_RAISE_DELTA_IN > MAX_LIFT_HEIGHT_IN) {
                    afterReleaseState = ScoreState::SCORE_WAIT_FORWARD;
                } else {
                    afterReleaseState = ScoreState::SCORE_RAISE_CLEAR;
                }
                scoreState = ScoreState::SCORE_RELEASE_DELAY;
            }
            break;

        case ScoreState::SCORE_RELEASE_DELAY:
            elevator.move(0);
            if (now - scoreStateStartTime >= CLAW_RELEASE_DELAY_MS) {
                if (afterReleaseState == ScoreState::SCORE_RAISE_CLEAR) {
                    scoreTargetHeight = getLiftHeightInches() + SCORE_RAISE_DELTA_IN;
                } else {
                    forwardStartPos = verticalRotation.get_position();
                }
                scoreState = afterReleaseState;
            }
            break;

        case ScoreState::SCORE_RAISE_CLEAR:
            driveLiftTowards(scoreTargetHeight);
            if (liftSettledAt(scoreTargetHeight)) {
                scoreState = ScoreState::SCORE_RETRACT_WRIST;
            }
            break;

        case ScoreState::SCORE_WAIT_FORWARD: {
            // Encoder counts to inches: the Rotation sensor reports centidegrees.
            double degreesTravelled = std::fabs(verticalRotation.get_position() - forwardStartPos) / 100.0;
            double inchesTravelled = (degreesTravelled / 360.0) * (M_PI * 2.0);
            if (inchesTravelled >= FORWARD_CLEAR_DISTANCE_IN) {
                scoreState = ScoreState::SCORE_RETRACT_WRIST;
            }
            break;
        }

        case ScoreState::SCORE_RETRACT_WRIST:
            scoringPiston.set_value(false);
            scoreStateStartTime = now;
            scoreState = ScoreState::SCORE_WRIST_RETRACT_DELAY;
            break;

        case ScoreState::SCORE_WRIST_RETRACT_DELAY:
            elevator.move(0);
            if (now - scoreStateStartTime >= WRIST_RETRACT_DELAY_MS) {
                scoreState = ScoreState::SCORE_LOWER_RESET;
            }
            break;

        case ScoreState::SCORE_LOWER_RESET:
            resetLiftToBottom();
            elevator.move(0);
            // Routine is done, hand the lift and intake back to the driver.
            sequenceStarted = false;
            scorePending = false;
            setpointIndex = 0;
            scoreState = ScoreState::IDLE;
            break;
        }

        pros::delay(10);
    }
}

// Background task that prints odometry and sensor values to the brain screen.
void updateLCD(void* param) {
    while (true) {
        int vertRaw = verticalRotation.get_position();
        int horzRaw = horizontalRotation.get_position();
        double imuDeg = imu.get_heading();
        lemlib::Pose pose = chassis.getPose(false);
        pros::lcd::print(0, "Vert: %d  Horz: %d", vertRaw, horzRaw);
        pros::lcd::print(1, "IMU: %.1f deg", imuDeg);
        pros::lcd::print(2, "X: %.2f  Y: %.2f", pose.x, pose.y);
        pros::lcd::print(3, "Theta: %.1f deg", pose.theta);
        pros::lcd::print(4, "Starting Pos: %d", currentStartingPos);
        pros::lcd::print(5, "0: Normal t/b (clr left), 1: Normal l/r");
        pros::lcd::print(6, "2: SAWP t/b, 3: SAWP l/r");
        pros::lcd::print(7, "4: line t/b, 5: line l/r, 6: skills");
        pros::delay(20);
    }
}

// Runs once when the program starts.
void initialize() {
    pros::lcd::initialize();
    pros::Task LCD_update_task(updateLCD, nullptr, "LCD Update Task");
    initializeGlobals();

    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator.tare_position_all();

    // Start with the intake arm up and both scoring pistons retracted.
    intakeLift1.set_value(true);
    endEffectorPiston.set_value(false);
    scoringPiston.set_value(false);
}

void disabled() {}
void competition_initialize() {}

// Field-control autonomous period: run whichever path is selected.
void autonomous() {
    Paths::runAutonomous();
}

// Driver control period.
void opcontrol() {
    pros::Task scoringTask(scoringTaskLoop, nullptr, "Scoring Task");

    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X) * 0.85; // scale turning down a little

        // Joystick deadband so the robot sits still when the sticks are centered.
        if (abs(forward) < 20) forward = 0;
        if (abs(turn) < 20) turn = 0;

        if (master.get_digital_new_press(DIGITAL_B)) {
            handleScorePress();
        }

        if (master.get_digital_new_press(DIGITAL_L1)) {
            handleL1Press();
        }

        if (master.get_digital_new_press(DIGITAL_L2)) {
            handleL2Press();
        }

        // R2: drop the intake arm and start intaking.
        if (master.get_digital_new_press(DIGITAL_R2) && !isScoringModeActive()) {
            intakeLiftState = false;
            runningIntake = true;
        }

        if (intakeDetection.get() < 290 && intakeLiftState) {
            intakeLiftState = true;
        }

        // R1: toggle the intake on and off.
        if (master.get_digital_new_press(DIGITAL_R1) && !isScoringModeActive()) {
            runningIntake = !runningIntake;
        }

        // D-pad left/right scrolls through the auton starting positions.
        if (master.get_digital_new_press(DIGITAL_LEFT) && currentStartingPos > 0) {
            currentStartingPos -= 1;
        }

        if (master.get_digital_new_press(DIGITAL_RIGHT) && currentStartingPos < 6) {
            currentStartingPos += 1;
        }

        // Y: re-run the selected auton (handy for testing).
        if (master.get_digital_new_press(DIGITAL_Y)) {
            Paths::runAutonomous();
        }

        if (!isScoringModeActive()) {
            intakePower = runningIntake ? 127 : 0;
        }

        // Arcade drive.
        driveLeftMotors.move(std::clamp(forward + turn, -127, 127));
        driveRightMotors.move(std::clamp(forward - turn, -127, 127));

        if (!isScoringModeActive()) {
            intake.move(intakePower);
        }

        intakeLift1.set_value(intakeLiftState);

        pros::delay(20);
    }
}
