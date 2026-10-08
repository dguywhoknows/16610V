#include "globals.hpp"
#include "paths.hpp"
#include "distSensorUtil.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/widgets/label/lv_label.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <math.h>
#include <cmath>

namespace Paths {
    void clampTask(void* param) {
        int theDelay = *static_cast<int*>(param);
        pros::delay(theDelay);
        endEffectorPiston.set_value(true);
    }

    void lowerLiftTask(void* param) {
        int theDelay = *static_cast<int*>(param);
        pros::delay(theDelay);
        resetLiftToBottom();
    }

    double heightForPins(int pins) {
        int index = pins - 1;
        if (index < 0) index = 0;
        if (index > numLiftSetpoints - 1) index = numLiftSetpoints - 1;
        return liftSetpointsIn[index];
    }

    void raiseForPins(int pins) {
        double targetHeight = heightForPins(pins);

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

    void goToSetpoint(int setpoint) {
        double targetHeight = heightForPins(setpoint);

        while (!liftSettledAt(targetHeight)) {
            driveLiftTowards(targetHeight);
            pros::delay(10);
        }
        elevator.move(0);
    }

    struct SetpointTaskParams {
        int setpoint;
        int delay;
    };

    void goToSetpointTask(void* param) {
        SetpointTaskParams* params = static_cast<SetpointTaskParams*>(param);
        pros::delay(params->delay);
        goToSetpoint(params->setpoint);
    }

    void scoreAtGoal() {
        endEffectorPiston.set_value(false);
        pros::delay(clawReleaseDelayMs);
    }

    struct RetractParams {
        int nextPins;
    };

    void retractAndTransitionTask(void* param) {
        RetractParams* params = static_cast<RetractParams*>(param);

        double currentHeight = getLiftHeightInches();
        if (currentHeight + scoreRaiseDeltaIn > maxLiftHeightIn) {
            double startPos = verticalRotation.get_position();
            double inchesTravelled = 0;
            while (inchesTravelled < forwardClearDistanceIn) {
                double degreesTravelled = std::fabs(verticalRotation.get_position() - startPos) / 100.0;
                inchesTravelled = (degreesTravelled / 360.0) * (M_PI * 2.0);
                pros::delay(10);
            }
        } else {
            double clearTarget = currentHeight + scoreRaiseDeltaIn;
            while (!liftSettledAt(clearTarget)) {
                driveLiftTowards(clearTarget);
                pros::delay(10);
            }
            elevator.move(0);
        }

        resetLiftToBottom();
        elevator.move(0);

        if (params->nextPins > 0) {
            raiseForPins(params->nextPins);
        }
    }

    
/*
    struct TrajectoryPoint {
        double x;
        double y;
        double heading;
        double curvature;
        double dist;
        double vel;
        double time;
    };

    constexpr double ramseteB = 2.0 / (39.37 * 39.37);
    constexpr double ramseteZeta = 0.7;
    constexpr double ramseteMaxAccel = 90.0;
    constexpr double ramseteMaxLateralAccel = 70.0;
    constexpr double ramseteTrackWidth = 11.92;
    constexpr double ramseteWheelDiameter = 2.75;
    constexpr double ramseteWheelRpm = 450.0;
    constexpr double ramseteMotorRpm = 600.0;
    constexpr double ramseteMaxVel = ramseteWheelRpm / 60.0 * M_PI * ramseteWheelDiameter;
    constexpr int ramseteSamplesPerSegment = 100;
    constexpr uint32_t ramseteLoopMs = 10;

    struct HermiteSegment {
        double p0x, p0y, t0x, t0y, p1x, p1y, t1x, t1y;
    };

    void sampleHermite(const HermiteSegment& seg, double u, double& x, double& y, double& dx, double& dy, double& ddx, double& ddy) {
        double u2 = u * u;
        double u3 = u2 * u;

        double h00 = 2 * u3 - 3 * u2 + 1;
        double h10 = u3 - 2 * u2 + u;
        double h01 = -2 * u3 + 3 * u2;
        double h11 = u3 - u2;

        double d00 = 6 * u2 - 6 * u;
        double d10 = 3 * u2 - 4 * u + 1;
        double d01 = -6 * u2 + 6 * u;
        double d11 = 3 * u2 - 2 * u;

        double dd00 = 12 * u - 6;
        double dd10 = 6 * u - 4;
        double dd01 = -12 * u + 6;
        double dd11 = 6 * u - 2;

        x = h00 * seg.p0x + h10 * seg.t0x + h01 * seg.p1x + h11 * seg.t1x;
        y = h00 * seg.p0y + h10 * seg.t0y + h01 * seg.p1y + h11 * seg.t1y;
        dx = d00 * seg.p0x + d10 * seg.t0x + d01 * seg.p1x + d11 * seg.t1x;
        dy = d00 * seg.p0y + d10 * seg.t0y + d01 * seg.p1y + d11 * seg.t1y;
        ddx = dd00 * seg.p0x + dd10 * seg.t0x + dd01 * seg.p1x + dd11 * seg.t1x;
        ddy = dd00 * seg.p0y + dd10 * seg.t0y + dd01 * seg.p1y + dd11 * seg.t1y;
    }

    std::vector<TrajectoryPoint> generateCurveTrajectory(double x1, double y1, double x2, double y2, bool forwards, double maxSpeed) {
        lemlib::Pose start = chassis.getPose(true, true);
        double facing = forwards ? start.theta : start.theta + M_PI;

        double len0 = std::hypot(x1 - start.x, y1 - start.y);
        double len1 = std::hypot(x2 - x1, y2 - y1);

        double dir0x = len0 > 1e-6 ? (x1 - start.x) / len0 : cos(facing);
        double dir0y = len0 > 1e-6 ? (y1 - start.y) / len0 : sin(facing);
        double dir1x = len1 > 1e-6 ? (x2 - x1) / len1 : dir0x;
        double dir1y = len1 > 1e-6 ? (y2 - y1) / len1 : dir0y;

        double midx = dir0x + dir1x;
        double midy = dir0y + dir1y;
        double midLen = std::hypot(midx, midy);
        if (midLen < 1e-6) {
            midx = dir1x;
            midy = dir1y;
        } else {
            midx /= midLen;
            midy /= midLen;
        }

        HermiteSegment segments[2] = {
            {start.x, start.y, cos(facing) * len0, sin(facing) * len0, x1, y1, midx * len0, midy * len0},
            {x1, y1, midx * len1, midy * len1, x2, y2, dir1x * len1, dir1y * len1},
        };

        std::vector<TrajectoryPoint> traj;
        traj.reserve(2 * ramseteSamplesPerSegment + 1);
        for (int s = 0; s < 2; s++) {
            for (int i = (s == 0 ? 0 : 1); i <= ramseteSamplesPerSegment; i++) {
                double u = static_cast<double>(i) / ramseteSamplesPerSegment;
                double x, y, dx, dy, ddx, ddy;
                sampleHermite(segments[s], u, x, y, dx, dy, ddx, ddy);

                double speedSq = dx * dx + dy * dy;
                double curvature = speedSq > 1e-9 ? (dx * ddy - dy * ddx) / std::pow(speedSq, 1.5) : 0;
                double heading = speedSq > 1e-9 ? atan2(dy, dx) : (traj.empty() ? facing : traj.back().heading);

                double dist = 0;
                if (!traj.empty()) dist = traj.back().dist + std::hypot(x - traj.back().x, y - traj.back().y);

                traj.push_back({x, y, heading, curvature, dist, 0, 0});
            }
        }

        double maxVel = ramseteMaxVel * std::clamp(maxSpeed, 0.0, 127.0) / 127.0;
        for (auto& p : traj) {
            double k = std::abs(p.curvature);
            double wheelLimit = maxVel / (1.0 + k * ramseteTrackWidth / 2.0);
            double lateralLimit = k > 1e-6 ? std::sqrt(ramseteMaxLateralAccel / k) : maxVel;
            p.vel = std::min({maxVel, wheelLimit, lateralLimit});
        }

        traj.front().vel = 0;
        for (size_t i = 1; i < traj.size(); i++) {
            double ds = traj[i].dist - traj[i - 1].dist;
            traj[i].vel = std::min(traj[i].vel, std::sqrt(traj[i - 1].vel * traj[i - 1].vel + 2 * ramseteMaxAccel * ds));
        }

        traj.back().vel = 0;
        for (int i = static_cast<int>(traj.size()) - 2; i >= 0; i--) {
            double ds = traj[i + 1].dist - traj[i].dist;
            traj[i].vel = std::min(traj[i].vel, std::sqrt(traj[i + 1].vel * traj[i + 1].vel + 2 * ramseteMaxAccel * ds));
        }

        for (size_t i = 1; i < traj.size(); i++) {
            double ds = traj[i].dist - traj[i - 1].dist;
            double avgVel = (traj[i].vel + traj[i - 1].vel) / 2.0;
            traj[i].time = traj[i - 1].time + (avgVel > 1e-6 ? ds / avgVel : 0);
        }

        return traj;
    }

    TrajectoryPoint sampleTrajectory(const std::vector<TrajectoryPoint>& traj, double t) {
        if (t <= traj.front().time) return traj.front();
        if (t >= traj.back().time) return traj.back();

        auto it = std::lower_bound(traj.begin(), traj.end(), t, [](const TrajectoryPoint& p, double time) { return p.time < time; });
        const TrajectoryPoint& b = *it;
        const TrajectoryPoint& a = *(it - 1);
        double span = b.time - a.time;
        double f = span > 1e-9 ? (t - a.time) / span : 0;

        double headingDiff = std::remainder(b.heading - a.heading, 2 * M_PI);
        return {
            a.x + f * (b.x - a.x),
            a.y + f * (b.y - a.y),
            a.heading + f * headingDiff,
            a.curvature + f * (b.curvature - a.curvature),
            a.dist + f * (b.dist - a.dist),
            a.vel + f * (b.vel - a.vel),
            t,
        };
    }

    double inPerSecToMotorRpm(double inPerSec) {
        double wheelRpm = inPerSec / (M_PI * ramseteWheelDiameter) * 60.0;
        return wheelRpm * ramseteMotorRpm / ramseteWheelRpm;
    }

    void followRamsete(const std::vector<TrajectoryPoint>& trajectory, bool forwards, int timeout) {
        if (trajectory.size() < 2) return;
        chassis.waitUntilDone();

        uint32_t startTime = pros::millis();
        double direction = forwards ? 1.0 : -1.0;

        while (true) {
            double elapsed = (pros::millis() - startTime) / 1000.0;
            if (elapsed > trajectory.back().time || pros::millis() - startTime > static_cast<uint32_t>(timeout)) break;

            TrajectoryPoint target = sampleTrajectory(trajectory, elapsed);
            lemlib::Pose pose = chassis.getPose(true, true);

            double targetTheta = forwards ? target.heading : target.heading + M_PI;
            double vd = direction * target.vel;
            double wd = target.vel * target.curvature;

            double dx = target.x - pose.x;
            double dy = target.y - pose.y;
            double ex = cos(pose.theta) * dx + sin(pose.theta) * dy;
            double ey = -sin(pose.theta) * dx + cos(pose.theta) * dy;
            double etheta = std::remainder(targetTheta - pose.theta, 2 * M_PI);

            double k = 2.0 * ramseteZeta * std::sqrt(wd * wd + ramseteB * vd * vd);
            double sinc = std::abs(etheta) < 1e-6 ? 1.0 : sin(etheta) / etheta;

            double v = vd * cos(etheta) + k * ex;
            double w = wd + k * etheta + ramseteB * vd * sinc * ey;

            double left = v - w * ramseteTrackWidth / 2.0;
            double right = v + w * ramseteTrackWidth / 2.0;

            driveLeftMotors.move_velocity(std::clamp(inPerSecToMotorRpm(left), -ramseteMotorRpm, ramseteMotorRpm));
            driveRightMotors.move_velocity(std::clamp(inPerSecToMotorRpm(right), -ramseteMotorRpm, ramseteMotorRpm));

            pros::delay(ramseteLoopMs);
        }

        driveLeftMotors.brake();
        driveRightMotors.brake();
    }

    void curveThrough(double x1, double y1, double x2, double y2, int timeout, bool forwards = true, double maxSpeed = 127) {
        followRamsete(generateCurveTrajectory(x1, y1, x2, y2, forwards, maxSpeed), forwards, timeout);
    }

    constexpr double lineUpToleranceIn = 0.6;
    constexpr double lineUpGoalClearanceIn = 8.0;
    constexpr double lineUpMinLeadIn = 6.0;
    constexpr double lineUpFieldLimitIn = 63.5;
    constexpr double lineUpSearchStepIn = 0.25;
    constexpr double lineUpSearchMaxIn = 144.0;
    constexpr int lineUpMaxAttempts = 4;
    constexpr int lineUpTurnTimeout = 700;
    constexpr int lineUpReturnTurnTimeout = 800;
    constexpr int lineUpMoveTimeout = 900;

    bool isLinedUp(double x, double y, bool forwards) {
        lemlib::Pose pose = chassis.getPose(true);
        double dirX = forwards ? sin(pose.theta) : -sin(pose.theta);
        double dirY = forwards ? cos(pose.theta) : -cos(pose.theta);
        double dx = x - pose.x;
        double dy = y - pose.y;
        if (std::hypot(dx, dy) <= lineUpToleranceIn) return true;

        double along = dx * dirX + dy * dirY;
        double across = dx * dirY - dy * dirX;
        return along > 0 && std::fabs(across) <= lineUpToleranceIn;
    }

    void shiftToLineUp(double x, double y, bool forwards);

    void lineUpToPoint(double x, double y, bool forwards, bool turning) {
        chassis.waitUntilDone();
        if (!turning) {
            shiftToLineUp(x, y, forwards);
            return;
        }

        for (int attempt = 0; attempt < lineUpMaxAttempts && !isLinedUp(x, y, forwards); attempt++) {
            chassis.turnToPoint(x, y, lineUpTurnTimeout, {.forwards = forwards});
            chassis.waitUntilDone();
            pros::delay(20);
        }
    }

    void shiftToLineUp(double x, double y, bool forwards) {
        chassis.waitUntilDone();
        double oldHeading = chassis.getPose().theta;
        double oldHeadingRad = oldHeading * M_PI / 180.0;
        double dirX = forwards ? sin(oldHeadingRad) : -sin(oldHeadingRad);
        double dirY = forwards ? cos(oldHeadingRad) : -cos(oldHeadingRad);

        for (int attempt = 0; attempt < lineUpMaxAttempts && !isLinedUp(x, y, forwards); attempt++) {
            lemlib::Pose pose = chassis.getPose(true);
            bool found = false;
            double bestX = 0;
            double bestY = 0;
            double bestDist = 1e9;

            for (double s = lineUpMinLeadIn; s <= lineUpSearchMaxIn; s += lineUpSearchStepIn) {
                double px = x - s * dirX;
                double py = y - s * dirY;
                if (std::fabs(px) > lineUpFieldLimitIn || std::fabs(py) > lineUpFieldLimitIn) continue;

                bool nearGoal = false;
                for (double gx : {-48.0, -24.0, 24.0, 48.0}) {
                    for (double gy : {-48.0, -24.0, 24.0, 48.0}) {
                        if (std::hypot(px - gx, py - gy) < lineUpGoalClearanceIn) nearGoal = true;
                    }
                }
                if (nearGoal) continue;

                double dist = std::hypot(px - pose.x, py - pose.y);
                if (dist < bestDist) {
                    found = true;
                    bestDist = dist;
                    bestX = px;
                    bestY = py;
                }
            }
            if (!found) break;

            bool driveForwards = (bestX - pose.x) * sin(pose.theta) + (bestY - pose.y) * cos(pose.theta) >= 0;
            chassis.moveToPoint(bestX, bestY, lineUpMoveTimeout, {.forwards = driveForwards});
            chassis.waitUntilDone();
            chassis.turnToHeading(oldHeading, lineUpReturnTurnTimeout);
            chassis.waitUntilDone();
            pros::delay(20);
            lineUpToPoint(x, y, forwards, true);
        }
    }
*/

    void manualDrive(int leftPower, int rightPower) {
        chassis.waitUntilDone();
        driveLeftMotors.move(leftPower);
        driveRightMotors.move(rightPower);
    }

    void firstScoringFunc(int hi) {
        int idkbru = 300;
        pros::Task hello(lowerLiftTask, &idkbru);
        chassis.moveToPoint(48, 60, 1500);
        chassis.waitUntilDone();
        hello.join();
        int bruh1 = 1;
        pros::Task bruh(raiseElevatorTask, &bruh1, "hi");
        pros::delay(20);
        chassis.turnToHeading(-90, 800);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        bruh.join();
        pros::delay(20);
        chassis.moveToPoint(63, 60, 900, {.forwards = false}, false);
        chassis.waitUntilDone();
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, true);
        pros::delay(20);
        SetpointTaskParams myParams = {hi, 300};
        pros::Task myTask(goToSetpointTask, &myParams, "Go To Setpoint");
        chassis.moveToPoint(46, 60, 1500);
        chassis.waitUntilDone();
        chassis.setPose(48, 60, chassis.getPose().theta, false);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.turnToHeading(0, 800);
        chassis.waitUntilDone();
        myTask.join();
        chassis.moveToPoint(48, 24, 900, {.forwards = false}, false);
        pros::delay(20);
        resetLiftToBottom();
        scoreAtGoal();
    }

    void SAWP1() {
        chassis.cancelMotion();
        pros::delay(10);
        //imu.reset();
        pros::delay(20);
        chassis.setPose(64, 9, 180, false);
        pros::delay(20);
/*
        sideTogglePiston2.set_value(true);
        pros::delay(100);
        sideTogglePiston2.set_value(false);
        pros::delay(300);
        sideTogglePiston2.set_value(true);
        pros::delay(100);
        sideTogglePiston2.set_value(false);
*/
        chassis.turnToHeading(140, 500, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        chassis.setPose(61,7, 140, false);
        pros::delay(20);
        pros::delay(20);

        chassis.moveToPoint(51, 20, 600, {.forwards = false}, false);
        chassis.waitUntilDone();
        chassis.setPose(48 + 6.8 * sin(chassis.getPose().theta * M_PI / 180), 24 + 6.8 * cos(chassis.getPose().theta * M_PI / 180), chassis.getPose().theta, false);
        elevator.move(-127);
        pros::delay(100);
        elevator.move(0);
        scoreAtGoal();

        int nn = 400;
        pros::Task nnn(lowerLiftTask, &nn);
        //62.37 10.12
        chassis.moveToPoint(58.64, 14.82, 300, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToHeading(200, 400, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        driveRightMotors.move(-127);
        driveLeftMotors.move(-50);
        pros::delay(485);
        driveLeftMotors.move(0);
        driveRightMotors.move(0);
        endEffectorPiston.set_value(true);
        pros::delay(200);

        chassis.turnToPoint(52.89, 5, 600, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        SetpointTaskParams scoringParams = {2, 100};
        pros::Task(goToSetpointTask, &scoringParams);
        chassis.moveToPoint(52.89, 12.71, 550, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToPoint(48, 24, 500, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(48, 20, 600, {.forwards = false}, false);
        chassis.waitUntilDone();
        elevator.move(-127);
        pros::delay(100);
        elevator.move(0);
        scoreAtGoal();
        chassis.setPose(48 + 6.8 * sin(chassis.getPose().theta * M_PI / 180), 24 + 6.8 * cos(chassis.getPose().theta * M_PI / 180), chassis.getPose().theta, false);
        pros::delay(20);

        pros::Task jfjf(lowerLiftTask, &nn);
        chassis.moveToPoint(61, -5, 700, {.forwards = true}, false);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, true, false);
        pros::delay(20);
        
        chassis.turnToHeading(-20, 600, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        driveLeftMotors.move(-127);
        driveRightMotors.move(-85);
        pros::delay(150);
        driveRightMotors.move(0);
        pros::delay(350);
        driveLeftMotors.move(0);
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, false);
        pros::delay(20);

        SetpointTaskParams scoringParams2 = {2, 300};
        pros::Task hi(goToSetpointTask, &scoringParams2);
        chassis.turnToPoint(52.89, -5, 600, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(52.89, -10, 550, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToPoint(48, -24, 600, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(48, -20, 600, {.forwards = false}, false);
        chassis.waitUntilDone();
        scoreAtGoal();
        chassis.setPose(48 + 6.8 * sin(chassis.getPose().theta * M_PI / 180), -24 + 6.8 * cos(chassis.getPose().theta * M_PI / 180), chassis.getPose().theta, false);
        pros::delay(20);
        
        int jojrweo = 400;
        pros::Task jojr(lowerLiftTask, &jojrweo);
        chassis.moveToPoint(50, 0, 600, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        chassis.setPose(50, 0, chassis.getPose().theta, false);
        pros::delay(20);

        chassis.turnToPoint(24, 24, 600, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        int something = 520;
        pros::Task somethingTask(clampTask, &something);
        chassis.moveToPoint(24, 24, 600, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        SetpointTaskParams scoringParams3 = {3, 300};
        pros::Task scoringTask67(goToSetpointTask, &scoringParams3);
        chassis.turnToPoint(48, 24, 600, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(48, 29, 700, {.forwards = false, .maxSpeed = 127}, false);
        chassis.waitUntilDone();
        scoringTask67.join();
        pros::delay(20);

        elevator.move(-127);
        pros::delay(100);
        elevator.move(0);
        scoreAtGoal();
        pros::delay(20);
/*
        chassis.turnToHeading(198.05, 700, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);
        
        chassis.moveToPoint(63.4, 13.28, 550, {.forwards = false}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        driveRightMotors.move(-127);
        pros::delay(400);
        driveRightMotors.move(0);
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, false);
        pros::delay(20);

        chassis.turnToPoint(52.89, 12.17, 700, {.maxSpeed = 127}, false);
        chassis.waitUntilDone();
        pros::delay(20);

        */
    }

    void SAWP2() {
        /*
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(63.25, 0, -90, false);
        pros::delay(20);
        */
    }

    void wall1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(6.75, -65, 270, false);
        pros::delay(20);

        sideTogglePiston2.set_value(true);
        pros::delay(100);
        sideTogglePiston2.set_value(false);
        pros::delay(300);
        sideTogglePiston2.set_value(true);
        pros::delay(100);
        sideTogglePiston2.set_value(false);

        chassis.turnToHeading(240, 700);
        chassis.setPose(6.75, -60, chassis.getPose().theta, false);

        chassis.moveToPoint(24, -48, 1000, {.forwards = false, .maxSpeed = 60});
        chassis.waitUntilDone();
        elevator.move(-127);
        pros::delay(190);
        elevator.move(0);
        scoreAtGoal();
        pros::delay(50);

        chassis.setPose(16, -56, chassis.getPose().theta, false);
        pros::delay(400);
        chassis.moveToPoint(10, -48, 1000, {.forwards = true, .maxSpeed = 60});

        resetPoseFromWalls(false, true, false);

        chassis.waitUntilDone();

        int lowerDelayW1 = 0;
        pros::Task lowerTaskW1(lowerLiftTask, &lowerDelayW1, "Lower Lift");
        chassis.turnToHeading(323, 500);
        chassis.waitUntilDone();
        lowerTaskW1.join();

        chassis.moveToPoint(21.5, -70, 1500, {.forwards = false, .maxSpeed = 50});
        //chassis.turnToHeading(350, 500, {.maxSpeed = 50});
        //chassis.moveToPoint(26, -72, 800, {.forwards = false, .maxSpeed = 50});
        chassis.waitUntilDone();
        manualDrive(0, -90);
        pros::delay(350);
        manualDrive(0, 0);
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, false);

        chassis.turnToHeading(300, 1000, {.maxSpeed = 40});
        chassis.moveToPoint(-9, -50, 1400, {.maxSpeed = 85});

        int pinsScoreW1 = 3;
        pros::Task raiseScoreW1(raiseElevatorTask, &pinsScoreW1, "Raise 2");
        chassis.turnToHeading(90, 900, {.maxSpeed = 75});
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.moveToPoint(-30, -50, 2500, {.forwards = false, .maxSpeed = 40});
        chassis.waitUntilDone();
        raiseScoreW1.join();

        resetLiftToBottom();
        scoreAtGoal();
        pros::delay(50);

        chassis.moveToPoint(-10, -48, 3000, {.forwards = true, .maxSpeed = 70});
    }

    void wall2() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(-65, 6.75, 180, false);
        pros::delay(20);

        sideTogglePiston.set_value(true);
        pros::delay(100);
        sideTogglePiston.set_value(false);
        pros::delay(300);
        sideTogglePiston.set_value(true);
        pros::delay(100);
        sideTogglePiston.set_value(false);

        chassis.turnToHeading(-150, 700);
        chassis.setPose(-60, 6.75, chassis.getPose().theta, false);

        chassis.moveToPoint(-48, 24, 1000, {.forwards = false, .maxSpeed = 60});
        chassis.waitUntilDone();
        elevator.move(-127);
        pros::delay(190);
        elevator.move(0);
        scoreAtGoal();
        pros::delay(50);
        
        chassis.setPose(-56, 16, chassis.getPose().theta, false);
        pros::delay(400);
        chassis.moveToPoint(-48, 10, 1000, {.forwards = true, .maxSpeed = 60});

        resetPoseFromWalls(false, true, false);

        chassis.waitUntilDone();

        int lowerDelayW2 = 0;
        pros::Task lowerTaskW2(lowerLiftTask, &lowerDelayW2, "Lower Lift");
        chassis.turnToHeading(127, 500);
        chassis.waitUntilDone();
        lowerTaskW2.join();

        chassis.moveToPoint(-70, 22, 1500, {.forwards = false, .maxSpeed = 50});
        //chassis.turnToHeading(100, 500, {.maxSpeed = 50});
        //chassis.moveToPoint(-72, 26, 800, {.forwards = false, .maxSpeed = 50});
        chassis.waitUntilDone();
        manualDrive(-90, 0);
        pros::delay(350);
        manualDrive(0, 0);
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, false);
        pros::delay(50);

        chassis.turnToHeading(150, 1000, {.maxSpeed = 60});
        chassis.moveToPoint(-53, 0, 1000);

        int pinsScoreW2 = 3;
        pros::Task raiseScoreW2(raiseElevatorTask, &pinsScoreW2, "Raise 2");
        chassis.turnToHeading(0, 900, {.maxSpeed = 75});
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.moveToPoint(-53, -30, 2000, {.forwards = false, .maxSpeed = 50});
        chassis.waitUntilDone();
        raiseScoreW2.join();

        resetLiftToBottom();
        scoreAtGoal();
        pros::delay(50);

        chassis.moveToPoint(-48, -10, 3000, {.forwards = true, .maxSpeed = 70});
    }

    void line1() {
        /*
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(63.25, 0, -90, false);
        pros::delay(20);
        */
    }

    void line2() {
        /*
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(63.25, 0, -90, false);
        pros::delay(20);
        */
    }

    void skillsPath() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        horizontalRotation.reset();
        pros::delay(20);
        chassis.setPose(63.25, 0, -90, false);
        pros::delay(20);

        goToSetpoint(2);
        pros::delay(20);
        chassis.moveToPoint(69, 0, 700, {.forwards = false});
        chassis.waitUntilDone();
        pros::delay(50);
        chassis.moveToPoint(63, 0, 700);
        chassis.waitUntilDone();
        pros::delay(50);
        chassis.moveToPoint(72, 0, 700, {.forwards = false});
        
        chassis.setPose(65.25, 0, -90, false);
        pros::delay(20);

        chassis.moveToPoint(36, 0, 900);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToHeading(90, 900);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(0, 0, 900, {.forwards = false});
        chassis.waitUntilDone();
        pros::delay(20);
        resetLiftToBottom();
        scoreAtGoal();
        chassis.setPose(6.25, 0, chassis.getPose().theta, false);
        pros::delay(50);

        chassis.moveToPoint(36, 0, 900);
        chassis.waitUntilDone();
        pros::delay(20);

        int sumDelay = 100;
        pros::Task okson(lowerLiftTask, &sumDelay);
        chassis.turnToHeading(0, 800);
        chassis.waitUntilDone();
        okson.join();
        pros::delay(20);

        chassis.moveToPoint(36, 60, 1200);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToHeading(-90, 700);
        chassis.waitUntilDone();
        resetPoseFromWalls(true, false, true);
        pros::delay(20);

        chassis.moveToPoint(63, 60, 900, {.forwards = false});
        chassis.waitUntilDone();
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, true);
        pros::delay(20);

        SetpointTaskParams sumParams = {2, 300};
        pros::Task whatTask(goToSetpointTask, &sumParams);
        chassis.moveToPoint(54, 60, 1000);
        chassis.waitUntilDone();
        chassis.setPose(60, 60, chassis.getPose().theta, false);
        pros::delay(20);

        chassis.turnToHeading(45, 800);
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(48, 48, 1100, {.forwards = false});
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.turnToHeading(0, 700);
        chassis.waitUntilDone();
        whatTask.join();
        pros::delay(20);

        chassis.moveToPoint(48, 24, 900, {.forwards = false, .maxSpeed = 80});
        chassis.waitUntilDone();
        pros::delay(20);
        resetLiftToBottom();
        scoreAtGoal();
        pros::delay(50);
        firstScoringFunc(2);
        pros::delay(50);
        firstScoringFunc(3);
        pros::delay(50);
        firstScoringFunc(4);
        pros::delay(50);
        firstScoringFunc(5);
        pros::delay(50);

        int idkbru2 = 300;
        pros::Task hello2(lowerLiftTask, &idkbru2);
        chassis.moveToPoint(48, 60, 1500);
        chassis.waitUntilDone();
        hello2.join();
        pros::delay(20);
        chassis.turnToHeading(-90, 800);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.moveToPoint(63, 60, 900, {.forwards = false}, false);
        chassis.waitUntilDone();
        endEffectorPiston.set_value(true);
        resetPoseFromWalls(true, false, true);
        pros::delay(20);
        SetpointTaskParams myParamsa = {6, 300};
        pros::Task myTaska(goToSetpointTask, &myParamsa, "Go To Setpoint");
        chassis.moveToPoint(44, 60, 1500, {.maxSpeed = 80});
        chassis.waitUntilDone();
        chassis.setPose(48, 60, chassis.getPose().theta, false);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.turnToHeading(0, 1100, {.maxSpeed = 80});
        chassis.waitUntilDone();
        myTaska.join();
        chassis.moveToPoint(48, 24, 1200, {.forwards = false, .maxSpeed = 80}, false);
        pros::delay(20);
        resetLiftToBottom();
        scoreAtGoal();
        pros::delay(50);

        int idkbru = 300;
        pros::Task hello(lowerLiftTask, &idkbru);
        chassis.moveToPoint(48, 60, 1500);
        chassis.waitUntilDone();
        hello.join();
        int bruh1 = 1;
        pros::Task bruh(raiseElevatorTask, &bruh1, "hi");
        pros::delay(20);
        chassis.turnToHeading(-90, 800);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        bruh.join();
        pros::delay(20);
        chassis.moveToPoint(63, 60, 900, {.forwards = false}, false);
        pros::delay(20);
        endEffectorPiston.set_value(true);
        SetpointTaskParams myParams = {3, 300};
        pros::Task myTask(goToSetpointTask, &myParams, "Go To Setpoint");
        chassis.moveToPoint(48, 60, 900);
        chassis.waitUntilDone();
        resetPoseFromWalls(false, false, true);
        pros::delay(20);
        chassis.turnToHeading(38.66, 800);
        chassis.waitUntilDone();
        myTask.join();
        pros::delay(20);
        
        chassis.moveToPoint(5, 6.25, 1500, {.forwards = false});
        chassis.moveToPoint(0, 0, 500, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        pros::delay(20);

        chassis.moveToPoint(5, 6.25, 500);
    }

    void runAutonomous() {
        autonRunning = true;
        //mclEnabled = true;
        fullLiftPowerAuton = currentStartingPos == 0 || currentStartingPos == 1 || currentStartingPos == 6;
        if (currentStartingPos == 0) {
            line1();
        }
        else if (currentStartingPos == 1) {
            line2();
        } else if (currentStartingPos == 2) {
            wall1();
        } else if (currentStartingPos == 3) {
            wall2();
        } else if (currentStartingPos == 4) {
            SAWP1();
        } else if (currentStartingPos == 5) {
            SAWP2();
        } else if (currentStartingPos == 6) {
            skillsPath();
        }
        autonRunning = false;
        //mclEnabled = false;
        fullLiftPowerAuton = false;
    }
}