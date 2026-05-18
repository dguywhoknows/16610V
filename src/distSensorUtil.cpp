#include "distSensorUtil.hpp"
#include "globals.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

static std::vector<Particle> particles;
static const double mcl_stdev = 3.0;
static std::mt19937 gen(std::random_device{}());

static double w_slow = 0.0;
static double w_fast = 0.0;
static const double alpha_slow = 0.05;
static const double alpha_fast = 0.2;

bool correct_position(dist_sensor sensor, lemlib::Chassis *chassis, bool is_x_wall, bool forced, double correct_rate) {
    lemlib::Pose currentPos = chassis->getPose(true);
    int32_t sensorValue = sensor.sensor.get();
    if (sensorValue == 9999 || sensorValue == 0) {
        std::cout << "distance value invalid, not correcting position" << std::endl;
        return false;
    }
    double distanceValue = sensorValue * 0.0393701;
    double rad = currentPos.theta * M_PI / 180.0;
    double offset_x = sensor.offset.x * cos(rad) + sensor.offset.y * sin(rad);
    double offset_y = -sensor.offset.x * sin(rad) + sensor.offset.y * cos(rad);
    double s_rad = (currentPos.theta + sensor.offset.theta) * M_PI / 180.0;
    double wall_x = currentPos.x + offset_x + distanceValue * sin(s_rad);
    double wall_y = currentPos.y + offset_y + distanceValue * cos(s_rad);
    if (is_x_wall) {
        double actual_wall = (wall_x > 0) ? 72.0 : -72.0;
        double corrected_x = actual_wall - (offset_x + distanceValue * sin(s_rad));
        if (std::abs(corrected_x - currentPos.x) < correct_rate || forced) {
            std::cout << "Corrected Pose X: " << corrected_x << std::endl;
            chassis->setPose(corrected_x, currentPos.y, currentPos.theta, true);
            return true;
        }
    } else {
        double actual_wall = (wall_y > 0) ? 72.0 : -72.0;
        double corrected_y = actual_wall - (offset_y + distanceValue * cos(s_rad));
        if (std::abs(corrected_y - currentPos.y) < correct_rate || forced) {
            std::cout << "Corrected Pose Y: " << corrected_y << std::endl;
            chassis->setPose(currentPos.x, corrected_y, currentPos.theta, true);
            return true;
        }
    }
    return false;
}

double get_expected_distance(lemlib::Pose p, lemlib::Pose offset, double& out_incidence_cos) {
    double rad = p.theta * M_PI / 180.0;
    double global_s_x = p.x + offset.x * cos(rad) + offset.y * sin(rad);
    double global_s_y = p.y - offset.x * sin(rad) + offset.y * cos(rad);
    double s_rad = (p.theta + offset.theta) * M_PI / 180.0;
    double ray_dx = sin(s_rad);
    double ray_dy = cos(s_rad);
    
    double min_dist = 9999;
    double norm_x = 0, norm_y = 0;

    if (std::abs(ray_dx) > 0.0001) {
        double d = ((ray_dx > 0 ? 72.0 : -72.0) - global_s_x) / ray_dx;
        if (d > 0 && d < min_dist) {
            min_dist = d;
            norm_x = (ray_dx > 0) ? -1.0 : 1.0; 
            norm_y = 0.0;
        }
    }
    if (std::abs(ray_dy) > 0.0001) {
        double d = ((ray_dy > 0 ? 72.0 : -72.0) - global_s_y) / ray_dy;
        if (d > 0 && d < min_dist) {
            min_dist = d;
            norm_x = 0.0;
            norm_y = (ray_dy > 0) ? -1.0 : 1.0;
        }
    }
    
    out_incidence_cos = std::abs(ray_dx * norm_x + ray_dy * norm_y);
    return min_dist;
}

double compute_weight(double expected, double measured, double stdev) {
    double error = measured - expected;
    return exp(-(error * error) / (2.0 * stdev * stdev));
}

void mcl_init(lemlib::Pose initial_pose, int num_particles) {
    particles.clear();
    particles.resize(num_particles);
    std::uniform_real_distribution<double> dist_xy(-4.0, 4.0);
    std::uniform_real_distribution<double> dist_theta(-2.0, 2.0);
    for (int i = 0; i < num_particles; i++) {
        particles[i].pose = lemlib::Pose(initial_pose.x + dist_xy(gen), initial_pose.y + dist_xy(gen), initial_pose.theta + dist_theta(gen));
        particles[i].weight = 1.0 / num_particles;
    }
    w_slow = 0.0;
    w_fast = 0.0;
}

void mcl_update(double local_dx, double local_dy, double dtheta) {
    double travel_dist = std::sqrt(local_dx * local_dx + local_dy * local_dy);
    std::normal_distribution<double> dist_pos(0.0, 0.05 * travel_dist + 0.005);
    std::normal_distribution<double> dist_rot(0.0, 0.02 * std::abs(dtheta) + 0.001);
    
    for (auto& p : particles) {
        double rad = p.pose.theta * M_PI / 180.0;
        double global_dx = local_dy * sin(rad) + local_dx * cos(rad);
        double global_dy = local_dy * cos(rad) - local_dx * sin(rad);
        p.pose.x += global_dx + dist_pos(gen);
        p.pose.y += global_dy + dist_pos(gen);
        p.pose.theta += dtheta + dist_rot(gen);
        p.pose.theta = std::fmod(p.pose.theta, 360.0);
        if (p.pose.theta < 0) p.pose.theta += 360.0;
    }
}

void mcl_sense(std::vector<dist_sensor>& sensors) {
    if (particles.empty()) return;
    std::vector<double> readings;
    
    for (auto& s : sensors) {
        int val = s.sensor.get();
        if (val == 9999 || val == 0 || s.sensor.get_confidence() < 60) {
            readings.push_back(-1);
        } else { 
            readings.push_back(val * 0.0393701);
        }
    }
    
    std::vector<double> sensor_max_weights(sensors.size(), 0.0);
    std::vector<std::vector<double>> particle_sensor_weights(particles.size(), std::vector<double>(sensors.size(), 1.0));
    
    int active_sensors = 0;
    
    for (size_t p_idx = 0; p_idx < particles.size(); p_idx++) {
        for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {
            if (readings[s_idx] < 0) {
                particle_sensor_weights[p_idx][s_idx] = -1.0;
                continue; 
            }
            
            double incidence_cos = 1.0;
            double expected = get_expected_distance(particles[p_idx].pose, sensors[s_idx].offset, incidence_cos);
            
            if (incidence_cos < 0.707) {
                particle_sensor_weights[p_idx][s_idx] = -1.0;
                continue;
            }
            
            double weight = compute_weight(expected, readings[s_idx], mcl_stdev);
            particle_sensor_weights[p_idx][s_idx] = weight;
            if (weight > sensor_max_weights[s_idx]) {
                sensor_max_weights[s_idx] = weight;
            }
        }
    }
    
    std::vector<bool> use_sensor(sensors.size(), true);
    double outlier_threshold = 0.05;
    for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {
        if (readings[s_idx] >= 0) {
            if (sensor_max_weights[s_idx] < outlier_threshold) {
                use_sensor[s_idx] = false;
            } else {
                active_sensors++;
            }
        }
    }
    
    double total_weight = 0;
    
    for (size_t p_idx = 0; p_idx < particles.size(); p_idx++) {
        double p_weight_sum = 0.0;
        int valid_sensor_count = 0;
        
        for (size_t s_idx = 0; s_idx < sensors.size(); s_idx++) {
            if (readings[s_idx] >= 0 && use_sensor[s_idx] && particle_sensor_weights[p_idx][s_idx] >= 0) {
                p_weight_sum += particle_sensor_weights[p_idx][s_idx];
                valid_sensor_count++;
            }
        }
        
        particles[p_idx].weight = (valid_sensor_count > 0) ? (p_weight_sum / valid_sensor_count) : (1.0 / particles.size());
        total_weight += particles[p_idx].weight;
    }
    
    if (total_weight > 0) {
        double avg_weight = total_weight / particles.size();
        for (auto& p : particles) p.weight /= total_weight;
        
        if (active_sensors > 0) {
            if (w_slow == 0.0) w_slow = avg_weight; else w_slow += alpha_slow * (avg_weight - w_slow);
            if (w_fast == 0.0) w_fast = avg_weight; else w_fast += alpha_fast * (avg_weight - w_fast);
        }
        
        double random_particle_prob = std::max(0.0, 1.0 - (w_fast / w_slow));
        
        std::vector<Particle> new_particles;
        new_particles.reserve(particles.size());
        int N = particles.size();

        double M_inv = 1.0 / N;
        std::uniform_real_distribution<double> uni_dist(0.0, M_inv);
        std::uniform_real_distribution<double> rand_prob_dist(0.0, 1.0);
        std::uniform_real_distribution<double> field_x(-72.0, 72.0);
        std::uniform_real_distribution<double> field_y(-72.0, 72.0);
        std::uniform_real_distribution<double> field_theta(0.0, 360.0);
        
        double r = uni_dist(gen);
        double c = particles[0].weight;
        int i = 0;
        
        for (int m = 0; m < N; m++) {
            if (rand_prob_dist(gen) < random_particle_prob && active_sensors > 0) {
                Particle rand_p;
                rand_p.pose = lemlib::Pose(field_x(gen), field_y(gen), field_theta(gen));
                rand_p.weight = M_inv;
                new_particles.push_back(rand_p);
            } else {
                double U = r + m * M_inv;
                while (U > c && i < N - 1) {
                    i++;
                    c += particles[i].weight;
                }
                Particle sampled = particles[i];
                sampled.weight = M_inv;
                new_particles.push_back(sampled);
            }
        }
        particles = std::move(new_particles);
    } else {
        double uniform_w = 1.0 / particles.size();
        for (auto& p : particles) p.weight = uniform_w;
    }
}

lemlib::Pose mcl_get_estimated_pose() {
    if (particles.empty()) return lemlib::Pose(0,0,0);
    
    double sum_x = 0, sum_y = 0;
    double sum_sin = 0, sum_cos = 0;
    double total_w = 0;

    for (auto& p : particles) {
        sum_x += p.pose.x * p.weight;
        sum_y += p.pose.y * p.weight;
        sum_sin += sin(p.pose.theta * M_PI / 180.0) * p.weight;
        sum_cos += cos(p.pose.theta * M_PI / 180.0) * p.weight;
        total_w += p.weight;
    }

    if (total_w < 0.00001) {
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
    double avg_theta = atan2(sum_sin, sum_cos) * 180.0 / M_PI;
    if (avg_theta < 0) avg_theta += 360.0;
    return lemlib::Pose(avg_x, avg_y, avg_theta);
}

void mcl_sync_with_chassis(lemlib::Chassis *chassis, double current_speed) {
    if (particles.empty()) return;
    lemlib::Pose mcl_pose = mcl_get_estimated_pose();
    double var_spatial = 0;
    for (const auto& p : particles) {
        double dx = p.pose.x - mcl_pose.x;
        double dy = p.pose.y - mcl_pose.y;
        var_spatial += (dx * dx + dy * dy);
    }
    var_spatial /= particles.size();
    
    if (current_speed < 3.0 && var_spatial < 1.5 && w_fast > 0.1) {
        chassis->setPose(mcl_pose.x, mcl_pose.y, mcl_pose.theta, true);
    }
}

lemlib::Pose mcl_get_fused_pose(lemlib::Pose odom_pose, double current_speed) {
    if (particles.empty()) return odom_pose;
    lemlib::Pose mcl_pose = mcl_get_estimated_pose();
    double var_spatial = 0;
    for (const auto& p : particles) {
        double dx = p.pose.x - mcl_pose.x;
        double dy = p.pose.y - mcl_pose.y;
        var_spatial += (dx * dx + dy * dy);
    }
    var_spatial /= particles.size();
    double blend_factor = 0.0;
    if (w_fast > 0.1) {
        const double MAX_BLEND = 0.15;
        double speed_scale = 1.0 - (std::abs(current_speed) / 5.0);
        speed_scale = std::clamp(speed_scale, 0.0, 1.0);
        double variance_scale = 1.0 - (var_spatial / 2.0);
        variance_scale = std::clamp(variance_scale, 0.0, 1.0);
        blend_factor = MAX_BLEND * speed_scale * variance_scale;
    }
    double fused_x = odom_pose.x + blend_factor * (mcl_pose.x - odom_pose.x);
    double fused_y = odom_pose.y + blend_factor * (mcl_pose.y - odom_pose.y);
    double odom_rad = odom_pose.theta * M_PI / 180.0;
    double mcl_rad = mcl_pose.theta * M_PI / 180.0;
    double fused_sin = sin(odom_rad) + blend_factor * (sin(mcl_rad) - sin(odom_rad));
    double fused_cos = cos(odom_rad) + blend_factor * (cos(mcl_rad) - cos(odom_rad));
    double fused_theta = atan2(fused_sin, fused_cos) * 180.0 / M_PI;
    if (fused_theta < 0) fused_theta += 360.0; 
    return lemlib::Pose(fused_x, fused_y, fused_theta);
}
