#include "distSensorUtil.hpp" // declares Particle struct, dist_sensor struct, and all function signatures
#include "globals.hpp"        // access to chassis, motors, sensors
#include <cmath>              // sin(), cos(), atan2(), sqrt(), exp(), fmod(), abs()
#include <iostream>           // std::cout: for debug prints to serial console
#include <algorithm>          // std::max(), std::clamp(), std::abs()
#include <random>             // mt19937 Mersenne Twister RNG and distributions

static std::vector<Particle> particles; // the particle set: each Particle has a pose (x, y, theta) and a weight

static const double mcl_stdev = 3.0; // Gaussian sensor model standard deviation in inches
// a particle 3" off from the true wall distance gets ~60% weight; 6" off gets ~14%

static std::mt19937 gen(std::random_device{}()); // Mersenne Twister RNG seeded from hardware entropy: used for all random sampling

static double w_slow = 0.0; // slow exponential moving average of particle weights: tracks long-term filter quality
static double w_fast = 0.0; // fast exponential moving average: tracks short-term quality spikes
// when w_fast drops below w_slow, the filter is getting worse → inject random particles to recover

static const double alpha_slow = 0.05; // learning rate for w_slow: ~20 sample memory, reacts slowly
static const double alpha_fast = 0.2;  // learning rate for w_fast: ~5 sample memory, reacts quickly

bool correct_position(dist_sensor sensor, lemlib::Chassis *chassis, bool is_x_wall, bool forced, double correct_rate) {
    // simple single-sensor correction, reads one distance sensor and snaps X or Y to the known field wall

    lemlib::Pose currentPos = chassis->getPose(true); // reads current odometry pose; true = heading in degrees

    int32_t sensorValue = sensor.sensor.get(); // raw reading from the V5 distance sensor in millimeters

    if (sensorValue == 9999 || sensorValue == 0) { // 9999 = out of range/no target; 0 = not yet initialized
        std::cout << "distance value invalid, not correcting position" << std::endl; // warn to serial
        return false; // bail - can't correct with bad data
    }

    double distanceValue = sensorValue * 0.0393701; // convert mm to inches (1 mm = 0.0393701 in)

    double rad = currentPos.theta * M_PI / 180.0; // robot heading in radians for trig

    double offset_x = sensor.offset.x * cos(rad) + sensor.offset.y * sin(rad); // rotate sensor's body-frame X offset into global frame
    double offset_y = -sensor.offset.x * sin(rad) + sensor.offset.y * cos(rad); // rotate sensor's body-frame Y offset into global frame

    double s_rad = (currentPos.theta + sensor.offset.theta) * M_PI / 180.0; // absolute sensor heading = robot heading + sensor mounting angle

    double wall_x = currentPos.x + offset_x + distanceValue * sin(s_rad); // project along ray to find where wall is in global X
    double wall_y = currentPos.y + offset_y + distanceValue * cos(s_rad); // project along ray to find where wall is in global Y

    if (is_x_wall) { // correcting X axis: left/right walls at x = ±72 inches

        double actual_wall = (wall_x > 0) ? 72.0 : -72.0; // real wall is +72 if we projected right, -72 if left

        double corrected_x = actual_wall - (offset_x + distanceValue * sin(s_rad)); // back-calculate where robot center must be

        if (std::abs(corrected_x - currentPos.x) < correct_rate || forced) { // only apply if jump less than or equal to correct_rate, or forced
            std::cout << "Corrected Pose X: " << corrected_x << std::endl;
            chassis->setPose(corrected_x, currentPos.y, currentPos.theta, true); // apply corrected X, keep Y and heading
            return true;
        }
    } else { // correcting Y axis, front/back walls at y = ±72 inches

        double actual_wall = (wall_y > 0) ? 72.0 : -72.0; // real wall is +72 if projected forward, -72 if backward

        double corrected_y = actual_wall - (offset_y + distanceValue * cos(s_rad)); // back-calculate robot center Y

        if (std::abs(corrected_y - currentPos.y) < correct_rate || forced) {
            std::cout << "Corrected Pose Y: " << corrected_y << std::endl;
            chassis->setPose(currentPos.x, corrected_y, currentPos.theta, true); // apply corrected Y, keep X and heading
            return true;
        }
    }

    return false; // correction skipped - jump was too large and not forced
}

double get_expected_distance(lemlib::Pose p, lemlib::Pose offset, double& out_incidence_cos) {
    // ray-casts from the sensor and returns expected distance to the nearest field wall
    // also outputs incidence_cos, low value = glancing ray = unreliable reading

    double rad = p.theta * M_PI / 180.0; // particle heading in radians

    double global_s_x = p.x + offset.x * cos(rad) + offset.y * sin(rad); // sensor's global X position
    double global_s_y = p.y - offset.x * sin(rad) + offset.y * cos(rad); // sensor's global Y position

    double s_rad = (p.theta + offset.theta) * M_PI / 180.0; // absolute sensor heading in radians

    double ray_dx = sin(s_rad); // X component of sensor ray direction (unit vector)
    double ray_dy = cos(s_rad); // Y component of sensor ray direction (unit vector)

    double min_dist = 9999;         // start with max: find the closest wall
    double norm_x = 0, norm_y = 0; // normal of the wall we hit

    if (std::abs(ray_dx) > 0.0001) { // only check X-walls if ray has X component (avoids division by zero)
        double d = ((ray_dx > 0 ? 72.0 : -72.0) - global_s_x) / ray_dx; // parametric intersection: d = (wall_x - sensor_x) / ray_x
        if (d > 0 && d < min_dist) { // positive d = wall is ahead; must be closer than current best
            min_dist = d;
            norm_x = (ray_dx > 0) ? -1.0 : 1.0; // right wall normal points left; left wall normal points right
            norm_y = 0.0;
        }
    }

    if (std::abs(ray_dy) > 0.0001) { // only check Y-walls if ray has Y component
        double d = ((ray_dy > 0 ? 72.0 : -72.0) - global_s_y) / ray_dy; // parametric intersection with front/back wall
        if (d > 0 && d < min_dist) {
            min_dist = d;
            norm_x = 0.0;
            norm_y = (ray_dy > 0) ? -1.0 : 1.0; // front wall normal points backward; back wall points forward
        }
    }

    out_incidence_cos = std::abs(ray_dx * norm_x + ray_dy * norm_y); // dot product of ray and wall normal = cos(incidence angle)
    // 1.0 = perpendicular hit (ideal); 0.0 = grazing hit (unreliable)

    return min_dist; // expected distance in inches to nearest wall
}

double compute_weight(double expected, double measured, double stdev) {
    double error = measured - expected; // difference between measured and expected (inches)
    return exp(-(error * error) / (2.0 * stdev * stdev)); // Gaussian: 1.0 at perfect match, falls off with error
}

void mcl_init(lemlib::Pose initial_pose, int num_particles) {

    particles.clear();
    particles.resize(num_particles); // allocate exactly num_particles slots

    std::uniform_real_distribution<double> dist_xy(-4.0, 4.0);    // scatter X/Y ±4 inches from starting pose
    std::uniform_real_distribution<double> dist_theta(-2.0, 2.0); // scatter heading ±2 degrees

    for (int i = 0; i < num_particles; i++) {
        particles[i].pose = lemlib::Pose(
            initial_pose.x     + dist_xy(gen),   // X = starting X + random ±4"
            initial_pose.y     + dist_xy(gen),   // Y = starting Y + random ±4"
            initial_pose.theta + dist_theta(gen) // heading = starting heading + random ±2°
        );
        particles[i].weight = 1.0 / num_particles; // all particles start with equal weight
    }

    w_slow = 0.0; // reset so stale data from a previous run doesn't carry over
    w_fast = 0.0;
}

void mcl_update(double local_dx, double local_dy, double dtheta) {
    // PREDICT step: moves every particle by the odometry delta + Gaussian noise

    double travel_dist = std::sqrt(local_dx * local_dx + local_dy * local_dy); // total distance moved this cycle

    std::normal_distribution<double> dist_pos(0.0, 0.05 * travel_dist + 0.005); // position noise: 5% of distance + 0.005" floor
    std::normal_distribution<double> dist_rot(0.0, 0.02 * std::abs(dtheta) + 0.001); // rotation noise: 2% of turn + 0.001° floor

    for (auto& p : particles) {

        double rad = p.pose.theta * M_PI / 180.0; // this particle's heading in radians

        // transform local motion delta into global frame using THIS particle's heading
        double global_dx = local_dy * sin(rad) + local_dx * cos(rad); // global X movement
        double global_dy = local_dy * cos(rad) - local_dx * sin(rad); // global Y movement

        p.pose.x += global_dx + dist_pos(gen); // move X by delta + random noise
        p.pose.y += global_dy + dist_pos(gen); // move Y by delta + random noise

        p.pose.theta += dtheta + dist_rot(gen);        // rotate by turn delta + random noise
        p.pose.theta = std::fmod(p.pose.theta, 360.0); // wrap to [0°, 360°)
        if (p.pose.theta < 0) p.pose.theta += 360.0;   // fmod can return negative values
    }
}

void mcl_sense(std::vector<dist_sensor>& sensors) {
    // SENSE + RESAMPLE: weights particles by sensor match, then resamples

    if (particles.empty()) return;

    // Step 1: read all sensors
    std::vector<double> readings; // in inches, or -1 if invalid

    for (auto& s : sensors) {
        int val = s.sensor.get(); // raw mm reading
        if (val == 9999 || val == 0 || s.sensor.get_confidence() < 60) { // out of range, uninitialized, or low confidence
            readings.push_back(-1);
        } else {
            readings.push_back(val * 0.0393701); // convert mm to inches
        }
    }

    // Step 2: compute per-particle per-sensor weights
    std::vector<double> sensor_max_weights(sensors.size(), 0.0); // best weight any particle got per sensor

    std::vector<std::vector<double>> particle_sensor_weights( // [particle][sensor] = weight; -1 = skip
        particles.size(), std::vector<double>(sensors.size(), 1.0)
    );

    int active_sensors = 0;

    for (size_t p_idx = 0; p_idx < particles.size(); p_idx++) {
        for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {

            if (readings[s_idx] < 0) { // invalid reading
                particle_sensor_weights[p_idx][s_idx] = -1.0;
                continue;
            }

            double incidence_cos = 1.0;
            double expected = get_expected_distance(particles[p_idx].pose, sensors[s_idx].offset, incidence_cos);
            // what distance would this particle expect to see?

            if (incidence_cos < 0.707) { // ray hits wall at less than 45deg, too glancing, skip (cos(45°) ≈ 0.707)
                particle_sensor_weights[p_idx][s_idx] = -1.0;
                continue;
            }

            double weight = compute_weight(expected, readings[s_idx], mcl_stdev); // Gaussian match score
            particle_sensor_weights[p_idx][s_idx] = weight;

            if (weight > sensor_max_weights[s_idx]) sensor_max_weights[s_idx] = weight; // track best weight for this sensor
        }
    }

    // Step 3: outlier sensor rejection
    std::vector<bool> use_sensor(sensors.size(), true);
    double outlier_threshold = 0.05; // if best weight < 5%, no particle matched - sensor is an outlier

    for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {
        if (readings[s_idx] >= 0) {
            if (sensor_max_weights[s_idx] < outlier_threshold) {
                use_sensor[s_idx] = false; // probably blocked or looking at wrong wall - discard this cycle
            } else {
                active_sensors++;
            }
        }
    }

    // Step 4: aggregate weights per particle
    double total_weight = 0;

    for (size_t p_idx = 0; p_idx < particles.size(); p_idx++) {
        double p_weight_sum = 0.0;
        int valid_sensor_count = 0;

        for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {
            if (readings[s_idx] >= 0 && use_sensor[s_idx] && particle_sensor_weights[p_idx][s_idx] >= 0) {
                p_weight_sum += particle_sensor_weights[p_idx][s_idx]; // add this sensor's score
                valid_sensor_count++;
            }
        }

        particles[p_idx].weight = (valid_sensor_count > 0)
            ? (p_weight_sum / valid_sensor_count) // average across valid sensors
            : (1.0 / particles.size());           // no valid sensors - give uniform weight so particle isn't killed

        total_weight += particles[p_idx].weight;
    }

    // Step 5: normalize, update adaptive averages, resample
    if (total_weight > 0) {

        double avg_weight = total_weight / particles.size(); // average weight before normalization

        for (auto& p : particles) p.weight /= total_weight; // normalize so weights sum to 1.0

        if (active_sensors > 0) {
            if (w_slow == 0.0) w_slow = avg_weight; else w_slow += alpha_slow * (avg_weight - w_slow); // slow EMA update
            if (w_fast == 0.0) w_fast = avg_weight; else w_fast += alpha_fast * (avg_weight - w_fast); // fast EMA update
        }

        double random_particle_prob = std::max(0.0, 1.0 - (w_fast / w_slow));
        // w_fast << w_slow - filter is degrading - inject random particles to help recover
        // w_fast >= w_slow - filter is healthy - no injection

        std::vector<Particle> new_particles;
        new_particles.reserve(particles.size());

        int N = particles.size();
        double M_inv = 1.0 / N; // step size for systematic resampling

        std::uniform_real_distribution<double> uni_dist(0.0, M_inv);
        std::uniform_real_distribution<double> rand_prob_dist(0.0, 1.0);
        std::uniform_real_distribution<double> field_x(-72.0, 72.0);    // random X anywhere on field
        std::uniform_real_distribution<double> field_y(-72.0, 72.0);    // random Y anywhere on field
        std::uniform_real_distribution<double> field_theta(0.0, 360.0); // random heading [0deg, 360deg)

        double r = uni_dist(gen);       // random start offset for systematic resampling
        double c = particles[0].weight; // running cumulative weight
        int i = 0;                      // index into old particle array

        for (int m = 0; m < N; m++) {

            if (rand_prob_dist(gen) < random_particle_prob && active_sensors > 0) {
                // inject a completely random particle to help filter recover
                Particle rand_p;
                rand_p.pose = lemlib::Pose(field_x(gen), field_y(gen), field_theta(gen));
                rand_p.weight = M_inv;
                new_particles.push_back(rand_p);
            } else {
                // systematic resampling - select particles proportional to weight
                double U = r + m * M_inv; // pointer on the [0,1] cumulative weight axis

                while (U > c && i < N - 1) { // walk cumulative sum until we reach the pointer
                    i++;
                    c += particles[i].weight;
                }
                // particles[i] is selected: high-weight particles cover more of [0,1] → duplicated more often

                Particle sampled = particles[i];
                sampled.weight = M_inv; // reset to uniform weight
                new_particles.push_back(sampled);
            }
        }

        particles = std::move(new_particles); // replace old set with resampled one

    } else {
        // all weights zero: reset to uniform so filter can recover
        double uniform_w = 1.0 / particles.size();
        for (auto& p : particles) p.weight = uniform_w;
    }
}

lemlib::Pose mcl_get_estimated_pose() {
    // returns the weighted average pose of all particles

    if (particles.empty()) return lemlib::Pose(0, 0, 0);

    double sum_x = 0, sum_y = 0;
    double sum_sin = 0, sum_cos = 0; // for circular mean of heading
    double total_w = 0;

    for (auto& p : particles) {
        sum_x   += p.pose.x * p.weight;
        sum_y   += p.pose.y * p.weight;
        sum_sin += sin(p.pose.theta * M_PI / 180.0) * p.weight; // weighted sin(heading)
        sum_cos += cos(p.pose.theta * M_PI / 180.0) * p.weight; // weighted cos(heading)
        total_w += p.weight;
    }

    if (total_w < 0.00001) { // degenerate weights - fall back to unweighted average
        double n = particles.size();
        double sx = 0, sy = 0, s_sin = 0, s_cos = 0;
        for (auto& p : particles) {
            sx += p.pose.x; sy += p.pose.y;
            s_sin += sin(p.pose.theta * M_PI / 180.0);
            s_cos += cos(p.pose.theta * M_PI / 180.0);
        }
        double avg_theta = atan2(s_sin, s_cos) * 180.0 / M_PI;
        if (avg_theta < 0) avg_theta += 360.0;
        return lemlib::Pose(sx / n, sy / n, avg_theta);
    }

    double avg_x = sum_x / total_w;
    double avg_y = sum_y / total_w;
    double avg_theta = atan2(sum_sin, sum_cos) * 180.0 / M_PI; // circular mean - handles 0deg/360deg wrap correctly
    if (avg_theta < 0) avg_theta += 360.0; // normalize to [0deg, 360deg)

    return lemlib::Pose(avg_x, avg_y, avg_theta);
}

void mcl_sync_with_chassis(lemlib::Chassis *chassis, double current_speed) {
    // hard sync: overwrites chassis pose with MCL estimate - only when slow and confident

    if (particles.empty()) return;

    lemlib::Pose mcl_pose = mcl_get_estimated_pose();

    double var_spatial = 0;
    for (const auto& p : particles) {
        double dx = p.pose.x - mcl_pose.x;
        double dy = p.pose.y - mcl_pose.y;
        var_spatial += (dx * dx + dy * dy); // accumulate squared distance from center
    }
    var_spatial /= particles.size(); // mean squared distance = spatial variance (in²)

    if (current_speed < 3.0 && var_spatial < 1.5 && w_fast > 0.1) {
        // only sync if: robot is slow (<3 in/s) AND particles tight (var<1.5 in^2) AND filter healthy (w_fast>0.1)
        chassis->setPose(mcl_pose.x, mcl_pose.y, mcl_pose.theta, true);
    }
}

lemlib::Pose mcl_get_fused_pose(lemlib::Pose odom_pose, double current_speed) {
    // gentle blend: mostly odometry with a small MCL nudge - no sudden jumps

    if (particles.empty()) return odom_pose;

    lemlib::Pose mcl_pose = mcl_get_estimated_pose();

    double var_spatial = 0;
    for (const auto& p : particles) {
        double dx = p.pose.x - mcl_pose.x;
        double dy = p.pose.y - mcl_pose.y;
        var_spatial += (dx * dx + dy * dy);
    }
    var_spatial /= particles.size();

    double blend_factor = 0.0; // how much to pull toward MCL (0 = pure odometry)

    if (w_fast > 0.1) { // only blend if filter has acceptable quality

        const double MAX_BLEND = 0.15; // cap MCL at 15% influence

        double speed_scale = 1.0 - (std::abs(current_speed) / 5.0); // 1.0 at rest, 0.0 at 5+ in/s
        speed_scale = std::clamp(speed_scale, 0.0, 1.0);

        double variance_scale = 1.0 - (var_spatial / 2.0); // 1.0 when perfectly clustered, 0.0 when very spread
        variance_scale = std::clamp(variance_scale, 0.0, 1.0);

        blend_factor = MAX_BLEND * speed_scale * variance_scale; // final blend: both factors must be high for significant correction
    }

    // linear interpolation for X and Y
    double fused_x = odom_pose.x + blend_factor * (mcl_pose.x - odom_pose.x); // nudge X toward MCL
    double fused_y = odom_pose.y + blend_factor * (mcl_pose.y - odom_pose.y); // nudge Y toward MCL

    // spherical interpolation for heading, can't average angles directly due to 0/360 degrees wrap
    double odom_rad = odom_pose.theta * M_PI / 180.0;
    double mcl_rad  = mcl_pose.theta  * M_PI / 180.0;

    double fused_sin = sin(odom_rad) + blend_factor * (sin(mcl_rad) - sin(odom_rad)); // blend sin components
    double fused_cos = cos(odom_rad) + blend_factor * (cos(mcl_rad) - cos(odom_rad)); // blend cos components

    double fused_theta = atan2(fused_sin, fused_cos) * 180.0 / M_PI; // reconstruct heading from blended sin/cos
    if (fused_theta < 0) fused_theta += 360.0; // normalize to [0deg, 360deg)

    return lemlib::Pose(fused_x, fused_y, fused_theta);
}