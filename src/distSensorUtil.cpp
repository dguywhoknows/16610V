#include "distSensorUtil.hpp"
#include "globals.hpp"
#include <cmath>
#include <algorithm>
#include <random>

DistTaskParams wallResetParams = {
    {
        {&distBack,  lemlib::Pose(-4.375, -2.875, 180)},
        {&distLeft,  lemlib::Pose(-5.875, -1,     270)},
        {&distRight, lemlib::Pose( 5.875, -1,      90)},
    },
    {true, true, true},
    &chassis,
};

void resetPoseFromWalls(bool back, bool left, bool right) {
    wallResetParams.use_sensor[0] = back;
    wallResetParams.use_sensor[1] = left;
    wallResetParams.use_sensor[2] = right;
    correctPoseFromDistSensors(&wallResetParams);
}

void correctPoseFromDistSensors(DistTaskParams* params) {
    if (params == nullptr || params->chassis == nullptr) return;

    double wall_dist = 72;
    double correct_rate = 10;

    lemlib::Pose currentPos = params->chassis->getPose(true);

    int sensorCount = std::min<int>(3, params->sensors.size());
    for (int i = 0; i < sensorCount; i++) {
        if (!params->use_sensor[i] || params->sensors[i].sensor == nullptr) continue;

        auto& s = params->sensors[i];
        int32_t sensorValue = s.sensor->get();
        if (sensorValue == 9999 || sensorValue <= 0) continue;

        double distanceValue = sensorValue * 0.0393701;

        double offset_y = -s.offset.x * sin(currentPos.theta) + s.offset.y * cos(currentPos.theta);
        double offset_x = s.offset.x * cos(currentPos.theta) + s.offset.y * sin(currentPos.theta);

        double s_rad = currentPos.theta + s.offset.theta * M_PI / 180;
        double x_value = distanceValue * sin(s_rad) + offset_x;
        double y_value = distanceValue * cos(s_rad) + offset_y;

        double ray_x = sin(s_rad);
        double ray_y = cos(s_rad);
        double hit_x_wall = std::abs(ray_x) > 1e-6 ? ((ray_x > 0 ? wall_dist : -wall_dist) - (currentPos.x + offset_x)) / ray_x : 1e9;
        double hit_y_wall = std::abs(ray_y) > 1e-6 ? ((ray_y > 0 ? wall_dist : -wall_dist) - (currentPos.y + offset_y)) / ray_y : 1e9;

        if (hit_x_wall < hit_y_wall) {
            x_value = (ray_x > 0 ? wall_dist : -wall_dist) - x_value;
            if (std::abs(x_value - currentPos.x) < correct_rate) {
                params->chassis->setPose(x_value, currentPos.y, currentPos.theta, true);
                currentPos = params->chassis->getPose(true);
            }
        } else {
            y_value = (ray_y > 0 ? wall_dist : -wall_dist) - y_value;
            if (std::abs(y_value - currentPos.y) < correct_rate) {
                params->chassis->setPose(currentPos.x, y_value, currentPos.theta, true);
                currentPos = params->chassis->getPose(true);
            }
        }
    }
}

/*
bool mclEnabled = false;

struct MCLParticle {
    double x;
    double y;
    double theta;
    double weight;
};

struct MCLReading {
    double inches;
    lemlib::Pose offset;
};

static constexpr int mclNumParticles = 300;
static constexpr double mclWallDist = 72.0;
static constexpr double mclSensorStdev = 3.0;
static constexpr double mclBaseNoise = 0.3;
static constexpr double mclTravelNoise = 0.05;
static constexpr double mclHeadingNoise = 0.03;
static constexpr double mclInitSpread = 3.0;
static constexpr double mclLikelihoodFloor = 0.001;
static constexpr double mclMinMatch = 0.05;
static constexpr double mclGateIn = 10.0;
static constexpr double mclMinIncidence = 0.7;
static constexpr double mclResetJumpIn = 8.0;
static constexpr double mclMinBlend = 0.1;
static constexpr double mclMaxBlend = 0.85;
static constexpr int32_t mclMaxRangeMm = 2000;
static constexpr int32_t mclMinObjectSize = 70;
static constexpr uint32_t mclPeriodMs = 50;
static constexpr double mclStoppedTravelIn = 0.25;
static constexpr double mclMinCorrectionIn = 0.3;
static constexpr double mclMaxCorrectionIn = 2.0;

static std::vector<MCLParticle> mclParticles;
static std::mt19937 mclGen(pros::micros());
static lemlib::Pose mclLastPose(0, 0, 0);
static double mclConfidence = 0;

static double mclExpectedDistance(double x, double y, double theta, const lemlib::Pose& offset, double& incidence) {
    double sx = x + offset.x * cos(theta) + offset.y * sin(theta);
    double sy = y - offset.x * sin(theta) + offset.y * cos(theta);
    double rayRad = theta + offset.theta * M_PI / 180.0;
    double dx = sin(rayRad);
    double dy = cos(rayRad);

    double best = 1e9;
    incidence = 0;
    if (std::abs(dx) > 1e-6) {
        double t = ((dx > 0 ? mclWallDist : -mclWallDist) - sx) / dx;
        if (t > 0 && t < best) {
            best = t;
            incidence = std::abs(dx);
        }
    }
    if (std::abs(dy) > 1e-6) {
        double t = ((dy > 0 ? mclWallDist : -mclWallDist) - sy) / dy;
        if (t > 0 && t < best) {
            best = t;
            incidence = std::abs(dy);
        }
    }
    return best;
}

static void mclResetParticles(const lemlib::Pose& pose) {
    std::uniform_real_distribution<double> spread(-mclInitSpread, mclInitSpread);
    mclParticles.resize(mclNumParticles);
    for (auto& p : mclParticles) {
        p.x = pose.x + spread(mclGen);
        p.y = pose.y + spread(mclGen);
        p.theta = pose.theta;
        p.weight = 1.0 / mclNumParticles;
    }
    mclLastPose = pose;
    mclConfidence = 0;
}

static void mclStep() {
    lemlib::Pose pose = chassis.getPose(true);
    double dx = pose.x - mclLastPose.x;
    double dy = pose.y - mclLastPose.y;
    double travel = std::hypot(dx, dy);

    if (mclParticles.empty() || travel > mclResetJumpIn) {
        mclResetParticles(pose);
        return;
    }
    mclLastPose = pose;

    std::vector<MCLReading> readings;
    for (auto& s : wallResetParams.sensors) {
        if (s.sensor == nullptr) continue;
        int32_t raw = s.sensor->get_distance();
        if (raw <= 0 || raw >= mclMaxRangeMm || s.sensor->get_object_size() <= mclMinObjectSize) continue;

        double inches = raw * 0.0393701;
        double incidence;
        double expected = mclExpectedDistance(pose.x, pose.y, pose.theta, s.offset, incidence);
        if (incidence < mclMinIncidence || std::abs(inches - expected) > mclGateIn) continue;

        readings.push_back({inches, s.offset});
    }

    if (readings.empty()) {
        for (auto& p : mclParticles) {
            p.x += dx;
            p.y += dy;
            p.theta = pose.theta;
        }
        mclConfidence = 0;
        return;
    }

    std::normal_distribution<double> posNoise(0.0, mclBaseNoise + mclTravelNoise * travel);
    std::normal_distribution<double> headingNoise(0.0, mclHeadingNoise);
    for (auto& p : mclParticles) {
        p.x += dx + posNoise(mclGen);
        p.y += dy + posNoise(mclGen);
        p.theta = pose.theta + headingNoise(mclGen);
    }

    double inv2Var = 1.0 / (2.0 * mclSensorStdev * mclSensorStdev);
    double totalWeight = 0;
    double bestWeight = 0;
    for (auto& p : mclParticles) {
        double likelihood = 1.0;
        for (auto& r : readings) {
            double incidence;
            double error = r.inches - mclExpectedDistance(p.x, p.y, p.theta, r.offset, incidence);
            likelihood *= std::max(mclLikelihoodFloor, exp(-error * error * inv2Var));
        }
        p.weight = likelihood;
        totalWeight += likelihood;
        bestWeight = std::max(bestWeight, likelihood);
    }

    if (totalWeight <= 0 || pow(bestWeight, 1.0 / readings.size()) < mclMinMatch) {
        for (auto& p : mclParticles) p.weight = 1.0 / mclNumParticles;
        mclConfidence = 0;
        return;
    }

    double meanX = 0;
    double meanY = 0;
    double sumSquares = 0;
    for (auto& p : mclParticles) {
        p.weight /= totalWeight;
        meanX += p.x * p.weight;
        meanY += p.y * p.weight;
        sumSquares += p.weight * p.weight;
    }

    double essRatio = (1.0 / sumSquares) / mclNumParticles;
    mclConfidence = 0.8 * essRatio + 0.2 * mclConfidence;
    double blend = std::clamp(mclConfidence, mclMinBlend, mclMaxBlend);

    std::vector<MCLParticle> resampled;
    resampled.reserve(mclNumParticles);
    double step = 1.0 / mclNumParticles;
    std::uniform_real_distribution<double> startDist(0.0, step);
    double u = startDist(mclGen);
    double cumulative = mclParticles[0].weight;
    int i = 0;
    for (int m = 0; m < mclNumParticles; m++) {
        while (u > cumulative && i < mclNumParticles - 1) {
            i++;
            cumulative += mclParticles[i].weight;
        }
        MCLParticle p = mclParticles[i];
        p.weight = step;
        resampled.push_back(p);
        u += step;
    }
    mclParticles = std::move(resampled);

    double correctionX = blend * (meanX - pose.x);
    double correctionY = blend * (meanY - pose.y);
    double correction = std::hypot(correctionX, correctionY);
    if (chassis.isInMotion() || travel > mclStoppedTravelIn || correction < mclMinCorrectionIn) return;
    if (correction > mclMaxCorrectionIn) {
        correctionX *= mclMaxCorrectionIn / correction;
        correctionY *= mclMaxCorrectionIn / correction;
    }

    pros::Task self = pros::Task::current();
    uint32_t oldPriority = self.get_priority();
    self.set_priority(TASK_PRIORITY_MAX - 1);
    lemlib::Pose now = chassis.getPose(true);
    if (chassis.isInMotion() || std::hypot(now.x - pose.x, now.y - pose.y) > mclStoppedTravelIn) {
        self.set_priority(oldPriority);
        return;
    }
    chassis.setPose(now.x + correctionX, now.y + correctionY, now.theta, true);
    mclLastPose = chassis.getPose(true);
    self.set_priority(oldPriority);

    double lagX = now.x - pose.x;
    double lagY = now.y - pose.y;
    for (auto& p : mclParticles) {
        p.x += lagX;
        p.y += lagY;
    }
}

void mclTaskLoop(void* param) {
    while (true) {
        if (mclEnabled) {
            mclStep();
        } else {
            mclParticles.clear();
        }
        pros::delay(mclPeriodMs);
    }
}
*/
