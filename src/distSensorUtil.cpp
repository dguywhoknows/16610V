#include "distSensorUtil.hpp"
#include "globals.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

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

static std::vector<Particle> particles;
static const double amcl_stdev = 3.0;

double get_expected_distance(lemlib::Pose p, lemlib::Pose offset) {
    double rad = p.theta * M_PI / 180.0;
    
    double global_s_x = p.x + offset.x * cos(rad) + offset.y * sin(rad);
    double global_s_y = p.y - offset.x * sin(rad) + offset.y * cos(rad);
    
    double s_rad = (p.theta + offset.theta) * M_PI / 180.0;
    double ray_dx = sin(s_rad);
    double ray_dy = cos(s_rad);
    
    double min_dist = 9999;
    
    if (std::abs(ray_dx) > 0.0001) {
        double d = ((ray_dx > 0 ? 72.0 : -72.0) - global_s_x) / ray_dx;
        if (d > 0 && d < min_dist) min_dist = d;
    }
    if (std::abs(ray_dy) > 0.0001) {
        double d = ((ray_dy > 0 ? 72.0 : -72.0) - global_s_y) / ray_dy;
        if (d > 0 && d < min_dist) min_dist = d;
    }
    return min_dist;
}

double compute_weight(double expected, double measured, double stdev) {
    double error = measured - expected;
    return exp(-(error * error) / (2.0 * stdev * stdev));
}

void amcl_init(lemlib::Pose initial_pose, int num_particles) {
    particles.clear();
    particles.resize(num_particles);
    for (int i = 0; i < num_particles; i++) {
        double noise_x = ((rand() % 100) / 50.0 - 1.0) * 4.0;
        double noise_y = ((rand() % 100) / 50.0 - 1.0) * 4.0;
        double noise_theta = ((rand() % 100) / 50.0 - 1.0) * 2.0;
        
        particles[i].pose = lemlib::Pose(initial_pose.x + noise_x, initial_pose.y + noise_y, initial_pose.theta + noise_theta);
        particles[i].weight = 1.0 / num_particles;
    }
}

void amcl_update(double dx, double dy, double dtheta) {
    for (auto& p : particles) {
        double noise_x = ((rand() % 100) / 50.0 - 1.0) * 0.1;
        double noise_y = ((rand() % 100) / 50.0 - 1.0) * 0.1;
        double noise_t = ((rand() % 100) / 50.0 - 1.0) * 0.1;
        
        p.pose.x += dx + noise_x;
        p.pose.y += dy + noise_y;
        p.pose.theta += dtheta + noise_t;
    }
}

void amcl_sense(std::vector<dist_sensor>& sensors) {
    if (particles.empty()) return;

    double total_weight = 0;
    std::vector<double> readings;
    
    for (auto& s : sensors) {
        int val = s.sensor.get();
        if (val == 9999 || val == 0 || s.sensor.get_confidence() < 60) {
            readings.push_back(-1);
        } else {
            readings.push_back(val * 0.0393701);
        }
    }
    
    for (auto& p : particles) {
        double p_weight = 1.0;
        for (size_t i = 0; i < sensors.size(); i++) {
            if (readings[i] < 0) continue; 
            
            double expected = get_expected_distance(p.pose, sensors[i].offset);
            double weight = compute_weight(expected, readings[i], amcl_stdev);
            p_weight *= weight;
        }
        p.weight = p_weight;
        total_weight += p.weight;
    }
    
    if (total_weight > 0) {
        for (auto& p : particles) p.weight /= total_weight;
        
        std::vector<Particle> new_particles(particles.size());
        std::vector<double> cumulative(particles.size());
        
        cumulative[0] = particles[0].weight;
        for (size_t i = 1; i < particles.size(); i++) {
            cumulative[i] = cumulative[i-1] + particles[i].weight;
        }
        
        for (size_t i = 0; i < particles.size(); i++) {
            double r = ((double)rand() / RAND_MAX); 
            auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
            int index = std::distance(cumulative.begin(), it);
            if (index >= particles.size()) index = particles.size() - 1;
            
            new_particles[i] = particles[index];
            new_particles[i].weight = 1.0 / particles.size();
        }
        particles = new_particles;
    }
}

lemlib::Pose amcl_get_estimated_pose() {
    if (particles.empty()) return lemlib::Pose(0,0,0);
    double sum_x = 0, sum_y = 0, sum_sin = 0, sum_cos = 0;
    
    for (auto& p : particles) {
        sum_x += p.pose.x;
        sum_y += p.pose.y;
        sum_sin += sin(p.pose.theta * M_PI / 180.0);
        sum_cos += cos(p.pose.theta * M_PI / 180.0);
    }
    
    double n = particles.size();
    double avg_theta = atan2(sum_sin, sum_cos) * 180.0 / M_PI;
    if (avg_theta < 0) avg_theta += 360;
    
    return lemlib::Pose(sum_x / n, sum_y / n, avg_theta);
}