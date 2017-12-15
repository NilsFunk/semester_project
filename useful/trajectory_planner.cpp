#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "filter.h"
#include <cmath>
#include <assert.h>
#include <ctime>
#include <fstream>
#include <iterator>

using namespace std;

inline double gauss(double sigma, double x) {
    double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
    double divider = sqrt(2 * M_PI * pow(sigma, 2));
    return (1 / divider) * exp(expVal);
}

inline std::vector<double> gaussKernel(int samples, double sigma) {
    std::vector<double> v;

    bool doubleCenter = false;
    if (samples % 2 == 0) {
        doubleCenter = true;
        samples--;
    }
    int steps = (samples - 1) / 2;
    double stepSize = (3 * sigma) / steps;

    for (int i = steps; i >= 1; i--) {
        v.push_back(gauss(sigma, i * stepSize * -1));
    }
    std::cout << std::endl;

    v.push_back(gauss(sigma, 0));
    if (doubleCenter) {
        v.push_back(gauss(sigma, 0));
    }

    for (int i = 1; i <= steps; i++) {
        v.push_back(gauss(sigma, i * stepSize));
    }

    float kernel_sum = 0;
    std::cout << "The kernel contains " << v.size() << " entries:";
    for (auto it = v.begin(); it != v.end(); ++it) {
        std::cout << ' ' << *it;
        kernel_sum += *it;
    }

    std::cout << std::endl;


    assert(v.size() == samples);

    return v;
}

inline std::vector<double> gaussSmoothen(std::vector<double> values, double sigma, int samples) {
    std::vector<double> out;
    auto kernel = gaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    int valueIdx = samples / 2 + 1;
    unsigned long ubound = values.size();
    for (unsigned long i = 0; i < ubound; i++) {
        double sample = 0;
        int sampleCtr = 0;
        double totalWeight = 0;
        //std::cout << "Now at value" << i << ": ";
        for (long j = i - sampleSide; j <= i + sampleSide; j++) {
            //std::cout << j << " ";
            if (j > 0 && j < ubound) {
                int sampleWeightIndex = sampleSide + (j - i);
                //std::cout << "(" << sampleWeightIndex << " [" << kernel[sampleWeightIndex] << "]) ";
                sample += kernel[sampleWeightIndex] * values[j];
                totalWeight += kernel[sampleWeightIndex];
                sampleCtr++;
            }
        }
        //double smoothed = sample / (double)sampleCtr;
        double smoothed = sample / totalWeight;
        //std::cout << " S: " << sample << " C: " << sampleCtr << " V: " << values[i] << " SM: " << smoothed << std::endl;
        out.push_back(smoothed);
    }
    return out;
}

int main() {
    time_t tstart, tend;
    tstart = time(0);
    static const float target_radius = 3;       // in
    static const float abs_vel = 2;             // in m/s
    float controller_freq = 50;    // in Hz¡
    static const float super_factor = 4;
    static const float pi = 3.14159;
    double sigma_ = 0.5;
    int kernel_size_ = 51;

    controller_freq = super_factor * controller_freq;

    float target_angle_deg = 20;    // in DEG
    float target_depth = 5;

    float target_x = target_depth;
    float target_angle_rad = 2 * pi / 360 * target_angle_deg;
    float target_y = tan(target_angle_rad) * target_depth;

    // calculate first circle information
    float beta = atan(target_y / target_x);
    float alpha = pi - 2 * (pi / 2 - beta);

    float sample_step_dist = abs_vel / controller_freq;
    float sample_step_rad_diff = sample_step_dist / target_radius;
    float sample_step_rad = sample_step_rad_diff;
    int number_samples_curve = int(alpha / sample_step_rad_diff);

    float abs_acc = abs_vel * abs_vel / target_radius;

    float circle_x = 0;
    float circle_y = target_radius;

    Eigen::Matrix3f CC;
    CC << 1, 0, -circle_x, 0, 1, -circle_y, -circle_x, -circle_y, circle_x * circle_x + circle_y * circle_y -
                                                                  target_radius * target_radius;
    Eigen::Vector3f PP;
    PP << target_x, target_y, 1;
    Eigen::Vector3f LL = CC * PP;


    float tangent_x = -LL(2) / LL(0);
    float p = -2 * circle_y;
    float q = tangent_x * tangent_x - 2 * tangent_x * circle_x + circle_x * circle_x + circle_y * circle_y -
              target_radius * target_radius;
    float tangent_y = -p / 2 - sqrt((p / 2) * (p / 2) - q);

    float tangent_beta = atan(tangent_y / tangent_x);
    float tangent_alpha = pi - 2 * (pi / 2 - tangent_beta);
    float yaw_switch = round(tangent_alpha / sample_step_rad_diff);

    float direction_diff_curve_end = alpha - target_angle_rad;
    float change_curve_angle = alpha - direction_diff_curve_end / 4;

    float m1 = tan(target_angle_rad);
    float m2 = tan(change_curve_angle);

    float p1_x = 0;
    float p1_y = 0;
    float a1 = 1;
    float b1 = -m1;
    float c1 = -p1_y + m1 * p1_x;
    float d1 = sqrt(1 + b1 * b1);

    float p2_x = target_radius * sin(change_curve_angle);
    float p2_y = target_radius * (1 - cos(change_curve_angle));
    float a2 = 1;
    float b2 = -m2;
    float c2 = -p2_y + m2 * p2_x;

    float d2 = sqrt(1 + b2 * b2);

    float A = a1 * d2 - a2 * d1;
    float B = b1 * d2 - b2 * d1;
    float C = c1 * d2 - c2 * d1;

    float E = -B / A;
    float F = -C / A;

    float G = F - p2_y;
    float H = b1 + a1 * E;
    float I = a1 * F + c1;

    float J = (1 + E * E) * d1 * d1 - H * H;

    float P = ((-2 * p2_x + 2 * E * G) * d1 * d1 - 2 * H * I) / J;
    float Q = ((p2_x * p2_x + G * G) * d1 * d1 - I * I) / J;

    float x_c = -P / 2; // + sqrt((P/2)*(P/2) - Q) this part is imaginary but close to zero -> just got rid of it
    float y_c = E * x_c + F;

    number_samples_curve = int(change_curve_angle / sample_step_rad_diff);

    //std::vector<std::vector<double>> path_curve;
    std::vector<double> path_x_pos;
    std::vector<double> path_y_pos;
    std::vector<double> path_x_vel;
    std::vector<double> path_y_vel;
    std::vector<double> path_x_acc;
    std::vector<double> path_y_acc;
    std::vector<double> path_yaw;
    std::vector<double> path_yaw_rate;

    for (int i = 0; i < number_samples_curve; ++i) {
        //std::vector<double> sample;
        float x_pos = target_radius * sin(sample_step_rad);
        float y_pos = target_radius * (1 - cos(sample_step_rad));
        float x_vel = abs_vel * cos(sample_step_rad);
        float y_vel = abs_vel * sin(sample_step_rad);
        float x_acc = -abs_acc * sin(sample_step_rad);
        float y_acc = abs_acc * cos(sample_step_rad);

        path_x_pos.push_back(x_pos);
        path_y_pos.push_back(y_pos);
        path_x_vel.push_back(x_vel);
        path_y_vel.push_back(y_vel);
        path_x_acc.push_back(x_acc);
        path_y_acc.push_back(y_acc);

        if (i <= yaw_switch) {
            float yaw = sample_step_rad;
            path_yaw.push_back(yaw);
            float yaw_rate = sample_step_rad_diff * controller_freq;
            path_yaw_rate.push_back(yaw_rate);
        }

        //path_curve.push_back(sample);

        sample_step_rad = sample_step_rad + sample_step_rad_diff;
    }


    // calculate second circle information

    float alpha_2 = change_curve_angle - target_angle_rad;
    float c2_radius = sqrt((x_c - p2_x) * (x_c - p2_x) + (y_c - p2_y) * (y_c - p2_y));

    float sample_step_rad_diff_2 = sample_step_dist / c2_radius;
    float sample_step_rad_2 = sample_step_rad_diff_2;
    int number_samples_curve_2 = int(alpha_2 / sample_step_rad_diff_2);

    //std::vector<std::vector<double>> path_curve_2;

    for (int i = 0; i < number_samples_curve_2; ++i) {
        //std::vector<double> sample;
        float x_pos_b = c2_radius * sin(sample_step_rad_2);
        float y_pos_b = c2_radius * cos(sample_step_rad_2);
        float x_vel_b = abs_vel * cos(sample_step_rad_2);
        float y_vel_b = -abs_vel * sin(sample_step_rad_2);
        float x_acc_b = -abs_acc * sin(sample_step_rad_2);
        float y_acc_b = -abs_acc * cos(sample_step_rad_2);

        float x_pos = x_pos_b * cos(change_curve_angle) - y_pos_b * sin(change_curve_angle) + x_c;
        float y_pos = x_pos_b * sin(change_curve_angle) + y_pos_b * cos(change_curve_angle) + y_c;
        float x_vel = x_vel_b * cos(change_curve_angle) - y_vel_b * sin(change_curve_angle);
        float y_vel = x_vel_b * sin(change_curve_angle) + y_vel_b * cos(change_curve_angle);
        float x_acc = x_acc_b * cos(change_curve_angle) - y_acc_b * sin(change_curve_angle);
        float y_acc = x_acc_b * sin(change_curve_angle) + y_acc_b * cos(change_curve_angle);

        path_x_pos.push_back(x_pos);
        path_y_pos.push_back(y_pos);
        path_x_vel.push_back(x_vel);
        path_y_vel.push_back(y_vel);
        path_x_acc.push_back(x_acc);
        path_y_acc.push_back(y_acc);

        sample_step_rad_2 = sample_step_rad_2 + sample_step_rad_diff_2;
    }

    // Path straight
    //std::vector<double> straight_pos_sample = path_curve_2.back();
    float straight_pos_x = path_x_pos.back();
    float straight_pos_y = path_y_pos.back();

    float dist_straight_target = sqrt((target_x - straight_pos_x) * (target_x - straight_pos_x) +
                                      (target_y - straight_pos_y) * (target_y - straight_pos_y));
    int number_samples_straight = int(dist_straight_target / sample_step_dist);
    float sample_step_dist_total = sample_step_dist;
    //std::vector<std::vector<double>> path_straight;

    for (int i = 0; i < number_samples_straight; ++i) {
        //std::vector<double> sample;
        float x_pos = straight_pos_x + sample_step_dist_total * cos(target_angle_rad);
        float y_pos = straight_pos_y + sample_step_dist_total * sin(target_angle_rad);
        float x_vel = abs_vel * cos(target_angle_rad);
        float y_vel = abs_vel * sin(target_angle_rad);
        float x_acc = 0;
        float y_acc = 0;
        float yaw = target_angle_rad;
        float yaw_rate = 0;

        path_x_pos.push_back(x_pos);
        path_y_pos.push_back(y_pos);
        path_x_vel.push_back(x_vel);
        path_y_vel.push_back(y_vel);
        path_x_acc.push_back(x_acc);
        path_y_acc.push_back(y_acc);
        path_yaw.push_back(yaw);
        path_yaw_rate.push_back(yaw_rate);

        sample_step_dist_total = sample_step_dist_total + sample_step_dist;
    }

    // yaw rate

    cout << yaw_switch  << endl;

    for (int i = yaw_switch + 1; i < (number_samples_curve + number_samples_curve_2); ++i)
    {
        float yaw = atan((target_y - path_y_pos.at(i)) / (target_x - path_x_pos.at(i)));
        path_yaw.insert(path_yaw.begin()+i, 1, yaw);
        float yaw_rate = (yaw - path_yaw.at(i - 1)) * controller_freq;
        cout << "yaw_rate: " << yaw_rate << endl;
        cout << "yaw:" << yaw << endl;
        cout << "yaw-:" << yaw_rate << endl;
        path_yaw_rate.insert(path_yaw_rate.begin()+i, 1, yaw_rate);
    }

    //std::vector<std::vector<double>> path_;
    //path_.insert( path_.end(), path_curve.begin(), path_curve.end() );
    //path_.insert( path_.end(), path_curve_2.begin(), path_curve_2.end() );
    //path_.insert( path_.end(), path_straight.begin(), path_straight.end() );

    std::vector<double> path_x_pos_smoothed = gaussSmoothen(path_x_pos, sigma_, kernel_size_);
    std::vector<double> path_y_pos_smoothed = gaussSmoothen(path_y_pos, sigma_, kernel_size_);
    std::vector<double> path_x_vel_smoothed = gaussSmoothen(path_x_vel, sigma_, kernel_size_);
    std::vector<double> path_y_vel_smoothed = gaussSmoothen(path_y_vel, sigma_, kernel_size_);
    std::vector<double> path_x_acc_smoothed = gaussSmoothen(path_x_acc, sigma_, kernel_size_);
    std::vector<double> path_y_acc_smoothed = gaussSmoothen(path_y_acc, sigma_, kernel_size_);
    std::vector<double> path_yaw_smoothed = gaussSmoothen(path_yaw, sigma_, kernel_size_);
    std::vector<double> path_yaw_rate_smoothed = gaussSmoothen(path_yaw_rate, sigma_, kernel_size_);

    for (int i = 0; i < kernel_size_/2; ++i)
    {
        path_x_pos_smoothed.at(i) = path_x_pos.at(i);
        path_y_pos_smoothed.at(i) = path_y_pos.at(i);
        path_x_vel_smoothed.at(i) = path_x_vel.at(i);
        path_y_vel_smoothed.at(i) = path_y_vel.at(i);
        path_x_acc_smoothed.at(i) = path_x_acc.at(i);
        path_y_acc_smoothed.at(i) = path_y_acc.at(i);
        path_yaw_smoothed.at(i) = path_yaw.at(i);
        path_yaw_rate_smoothed.at(i) = path_yaw_rate.at(i);
    }

    std::ofstream of_x_pos("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_pos.txt");
    std::ostream_iterator<double> oi_x_pos(of_x_pos, "\n");
    std::copy(path_x_pos.begin(), path_x_pos.end(), oi_x_pos);

    std::ofstream of_y_pos("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_pos.txt");
    std::ostream_iterator<double> oi_y_pos(of_y_pos, "\n");
    std::copy(path_y_pos.begin(), path_y_pos.end(), oi_y_pos);

    std::ofstream of_x_vel("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_vel.txt");
    std::ostream_iterator<double> oi_x_vel(of_x_vel, "\n");
    std::copy(path_x_vel.begin(), path_x_vel.end(), oi_x_vel);

    std::ofstream of_y_vel("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_vel.txt");
    std::ostream_iterator<double> oi_y_vel(of_y_vel, "\n");
    std::copy(path_y_vel.begin(), path_y_vel.end(), oi_y_vel);

    std::ofstream of_x_acc("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_acc.txt");
    std::ostream_iterator<double> oi_x_acc(of_x_acc, "\n");
    std::copy(path_x_acc.begin(), path_x_acc.end(), oi_x_acc);

    std::ofstream of_y_acc("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_acc.txt");
    std::ostream_iterator<double> oi_y_acc(of_y_acc, "\n");
    std::copy(path_y_acc.begin(), path_y_acc.end(), oi_y_acc);

    std::ofstream of_yaw("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/yaw.txt");
    std::ostream_iterator<double> oi_yaw(of_yaw, "\n");
    std::copy(path_yaw.begin(), path_yaw.end(), oi_yaw);

    std::ofstream of_yaw_rate("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/yaw_rate.txt");
    std::ostream_iterator<double> oi_yaw_rate(of_yaw_rate, "\n");
    std::copy(path_yaw_rate.begin(), path_yaw_rate.end(), oi_yaw_rate);

    std::ofstream of_x_poss("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_poss.txt");
    std::ostream_iterator<double> oi_x_poss(of_x_poss, "\n");
    std::copy(path_x_pos_smoothed.begin(), path_x_pos_smoothed.end(), oi_x_poss);

    std::ofstream of_y_poss("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_poss.txt");
    std::ostream_iterator<double> oi_y_poss(of_y_poss, "\n");
    std::copy(path_y_pos_smoothed.begin(), path_y_pos_smoothed.end(), oi_y_poss);

    std::ofstream of_x_vels("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_vels.txt");
    std::ostream_iterator<double> oi_x_vels(of_x_vels, "\n");
    std::copy(path_x_vel_smoothed.begin(), path_x_vel_smoothed.end(), oi_x_vels);

    std::ofstream of_y_vels("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_vels.txt");
    std::ostream_iterator<double> oi_y_vels(of_y_vels, "\n");
    std::copy(path_y_vel_smoothed.begin(), path_y_vel_smoothed.end(), oi_y_vels);

    std::ofstream of_x_accs("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/x_accs.txt");
    std::ostream_iterator<double> oi_x_accs(of_x_accs, "\n");
    std::copy(path_x_acc_smoothed.begin(), path_x_acc_smoothed.end(), oi_x_accs);

    std::ofstream of_y_accs("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/y_accs.txt");
    std::ostream_iterator<double> oi_y_accs(of_y_accs, "\n");
    std::copy(path_y_acc_smoothed.begin(), path_y_acc_smoothed.end(), oi_y_accs);

    std::ofstream of_yaws("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/yaws.txt");
    std::ostream_iterator<double> oi_yaws(of_yaws, "\n");
    std::copy(path_yaw_smoothed.begin(), path_yaw_smoothed.end(), oi_yaws);

    std::ofstream of_yaw_rates("/Users/nilsiism/Workspace/arduino/impactTestBenchAnalogWithSensorsWithoutDelay/MATLABfiles/yaw_rates.txt");
    std::ostream_iterator<double> oi_yaw_rates(of_yaw_rates, "\n");
    std::copy(path_yaw_rate_smoothed.begin(), path_yaw_rate_smoothed.end(), oi_yaw_rates);

    tend = time(0);
    cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< endl;

}