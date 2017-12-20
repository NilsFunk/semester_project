//
// Created by nilsiism on 07.11.17.
//
#include "desired_state_publisher_yaw_adjustment.h"

namespace depth_flight_controller {

    DesiredStatePublisherYawAdjustment::DesiredStatePublisherYawAdjustment()
    {
        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &DesiredStatePublisherYawAdjustment::stateEstimateCallback, this);
        target_sub_ = nh_.subscribe("/hummingbird/target", 1, &DesiredStatePublisherYawAdjustment::pathCallback, this);
        desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("/hummingbird/desired_state", 1);

        abs_vel = 1.5;
        target_radius = 3;
        super_factor_ = 4;
        sample_switch_frequency_ = 50;

        main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / sample_switch_frequency_), &DesiredStatePublisherYawAdjustment::mainloop, this);
        most_recent_path_generation_ = ros::Time::now();

        path_ = generateStarterPath();
    }

    DesiredStatePublisherYawAdjustment::~DesiredStatePublisherYawAdjustment()
    {

    }

    void DesiredStatePublisherYawAdjustment::mainloop(const ros::TimerEvent& time)
    {
        std::cout << is_new_path << std::endl;

        if (is_new_path == true) {
            path_ = path_new_;
            is_new_path = false;
        }

        if (path_.size() > 4) {
            quad_msgs::QuadDesiredState desired_state;
            desired_state = path_.front();
            desired_state.header.stamp = ros::Time::now();
            desired_state_pub_.publish(desired_state);
            path_.erase(path_.begin(), path_.begin() + 3);
        }
    }

    void DesiredStatePublisherYawAdjustment::pathCallback(const depth_flight_controller_msgs::Target &msg)
    {

        if (msg.valid == true && ros::Time::now() - most_recent_path_generation_ > ros::Duration(1.0))
        {
            is_trajectory_valid_ = true;
            double current_x_pos = state_estimate_.position(0);
            double current_y_pos = state_estimate_.position(1);
            double original_x_pos = msg.position.x;
            double original_y_pos = msg.position.y;
            double original_yaw = msg.yaw;
            double original_depth = msg.depth;
            double original_Y = msg.Y;
            double current_side;
            double current_angle;

            double current_yaw = QuaterniondToYaw(state_estimate_.orientation);

            double target_x_pos = original_x_pos + original_depth * cos(original_yaw) - original_Y * sin(original_yaw);
            double target_y_pos = original_y_pos + original_depth * sin(original_yaw) + original_Y * cos(original_yaw);

            double current_depth    = (target_x_pos - current_x_pos) * cos(current_yaw) + (target_y_pos - current_y_pos) * sin(current_yaw);
            double current_Y        = -1*(target_x_pos - current_x_pos) * sin(current_yaw) + (target_y_pos - current_y_pos) * cos(current_yaw);

            current_angle    = fabs(atan(current_Y/current_depth));

            if (current_angle > 0.01)
            {
                current_side     = current_Y/fabs(current_Y);
                current_Y        = current_Y/(double)current_side;
            }
            else
            {
                current_side    = 0;
                current_angle   = 0;
            }

            /*
            std::cout << "max depth : " << current_depth << std::endl;
            std::cout << "max depth original: " << original_depth << std::endl;
            std::cout << "current Y: " << current_Y << std::endl;
            std::cout << "original Y: " << original_Y << std::endl;
            std::cout << "current_x_pos: " << current_x_pos << std::endl;
            std::cout << "current_y_pos: " << current_y_pos << std::endl;
            std::cout << "original_x_pos: " << original_x_pos << std::endl;
            std::cout << "original_y_pos: " << original_y_pos << std::endl;
            std::cout << "target_x_pos: " << target_x_pos << std::endl;
            std::cout << "target_y_pos: " << target_y_pos << std::endl;
            std::cout << "current_side " << current_side << std::endl;
            std::cout << "curent_angle " << current_angle << std::endl;
            */

            if (current_side != 0 && current_angle > 0.06)
            {
                path_new_ = generatePath(current_angle, current_depth, current_side, state_estimate_);

            } else
            {
                path_new_ = generateStraightPath(current_depth, state_estimate_);
            }

            if (is_trajectory_valid_ == true)
            {
                most_recent_path_generation_ = ros::Time::now();
                is_new_path = true;
            }
        }
    }


    void DesiredStatePublisherYawAdjustment::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
    }

    void DesiredStatePublisherYawAdjustment::stateEstimateOriginalCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_original_ = QuadState(*msg);
    }

    double DesiredStatePublisherYawAdjustment::QuaterniondToYaw(const Eigen::Quaterniond& q)
    {
        // yaw (z-axis rotation)
        double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        double yaw = atan2(siny, cosy);

        return yaw;
    }


    inline double DesiredStatePublisherYawAdjustment::gauss(double sigma, double x)
    {
        double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
        double divider = sqrt(2 * M_PI * pow(sigma, 2));
        return (1 / divider) * exp(expVal);
    }

    inline std::vector<double> DesiredStatePublisherYawAdjustment::gaussKernel(int samples, double sigma)
    {
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

        v.push_back(gauss(sigma, 0));
        if (doubleCenter) {
            v.push_back(gauss(sigma, 0));
        }

        for (int i = 1; i <= steps; i++) {
            v.push_back(gauss(sigma, i * stepSize));
        }

        float kernel_sum = 0;
        for (std::vector<int>::size_type i = 0; i != v.size(); i++) {
            kernel_sum += i;
        }

        assert(v.size() == samples);

        return v;
    }

    inline std::vector<double> DesiredStatePublisherYawAdjustment::gaussSmoothen(std::vector<double> values, double sigma, int samples)
    {
        std::vector<double> out;
        std::vector<double> kernel = gaussKernel(samples, sigma);
        int sampleSide = samples / 2;
        int valueIdx = samples / 2 + 1;
        unsigned long ubound = values.size();
        for (unsigned long i = 0; i < ubound; i++) {
            double sample = 0;
            int sampleCtr = 0;
            double totalWeight = 0;
            for (long j = i - sampleSide; j <= i + sampleSide; j++) {
                if (j > 0 && j < ubound) {
                    int sampleWeightIndex = sampleSide + (j - i);
                    sample += kernel[sampleWeightIndex] * values[j];
                    totalWeight += kernel[sampleWeightIndex];
                    sampleCtr++;
                }
            }
            double smoothed = sample / totalWeight;
            out.push_back(smoothed);
        }
        return out;
    }

    std::vector<quad_msgs::QuadDesiredState> DesiredStatePublisherYawAdjustment::generatePath(double target_angle_rad, double target_depth, int target_side, QuadState state_estimate) {

        std::vector <quad_msgs::QuadDesiredState> path_smoothed;

        if (target_depth < 0.7) {
            target_depth = 0.7;
        }

        float controller_freq = 50;    // in Hz¡
        static const float pi = 3.14159;
        double sigma_ = 0.5;
        int kernel_size_ = 51;

        controller_freq = super_factor_ * controller_freq;

        float target_x = target_depth;
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

        // Prepare second curve
        float x_c;
        float y_c;
        float p2_x;
        float p2_y;
        float change_curve_angle;

        if (target_angle_rad > 0.06)
        {
            float direction_diff_curve_end = alpha - target_angle_rad;
            change_curve_angle = alpha - direction_diff_curve_end / 4;

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

            x_c = -P / 2; // + sqrt((P/2)*(P/2) - Q) this part is imaginary but close to zero -> just got rid of it
            y_c = E * x_c + F;

            number_samples_curve = int(change_curve_angle / sample_step_rad_diff);
        }

        std::vector<double> path_x_pos;
        std::vector<double> path_y_pos;
        std::vector<double> path_x_vel;
        std::vector<double> path_y_vel;
        std::vector<double> path_x_acc;
        std::vector<double> path_y_acc;
        std::vector<double> path_yaw;
        std::vector<double> path_yaw_rate;

        for (int i = 0; i < number_samples_curve; ++i) {
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

            float yaw = sample_step_rad;
            path_yaw.push_back(yaw);
            float yaw_rate = sample_step_rad_diff * controller_freq;
            path_yaw_rate.push_back(yaw_rate);

            sample_step_rad = sample_step_rad + sample_step_rad_diff;
        }

        if (target_angle_rad > 0.06) {
            // calculate second circle information
            float alpha_2 = change_curve_angle - target_angle_rad;
            float yaw2 = path_yaw.back();
            float yaw_rate2;

            float c2_radius = sqrt((x_c - p2_x) * (x_c - p2_x) + (y_c - p2_y) * (y_c - p2_y));
            if (c2_radius > 10000000) {
                //is_trajectory_valid_ = false;

                // Fix broken trajectory with radis set to 2
                c2_radius = 2;
                is_trajectory_valid_ = true;
            }

            if (is_trajectory_valid_ == true) {
                float sample_step_rad_diff_2 = sample_step_dist / c2_radius;

                float sample_step_rad_2 = sample_step_rad_diff_2;
                std::cout << alpha_2 << std::endl;
                std::cout << sample_step_rad_diff_2 << std::endl;
                int number_samples_curve_2 = int(alpha_2 / sample_step_rad_diff_2);
                std::cout << number_samples_curve_2 << std::endl;
                number_samples_curve = number_samples_curve + number_samples_curve_2;
                yaw_rate2 = sample_step_rad_diff_2 * controller_freq;

                for (int i = 0; i < number_samples_curve_2; ++i) {
                    yaw2 = yaw2 - sample_step_rad_diff_2;

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
                    path_yaw.push_back(yaw2);
                    path_yaw_rate.push_back(yaw_rate2);

                    sample_step_rad_2 = sample_step_rad_2 + sample_step_rad_diff_2;
                }
            }

            // Path straight
            float straight_pos_x = path_x_pos.back();
            float straight_pos_y = path_y_pos.back();

            float dist_straight_target = sqrt((target_x - straight_pos_x) * (target_x - straight_pos_x) +
                                              (target_y - straight_pos_y) * (target_y - straight_pos_y));
            int number_samples_straight = int(dist_straight_target / sample_step_dist);
            float sample_step_dist_total = sample_step_dist;

            for (int i = 0; i < number_samples_straight; ++i) {
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

            std::vector<double> path_x_pos_smoothed = gaussSmoothen(path_x_pos, sigma_, kernel_size_);
            std::vector<double> path_y_pos_smoothed = gaussSmoothen(path_y_pos, sigma_, kernel_size_);
            std::vector<double> path_x_vel_smoothed = gaussSmoothen(path_x_vel, sigma_, kernel_size_);
            std::vector<double> path_y_vel_smoothed = gaussSmoothen(path_y_vel, sigma_, kernel_size_);
            std::vector<double> path_x_acc_smoothed = gaussSmoothen(path_x_acc, sigma_, kernel_size_);
            std::vector<double> path_y_acc_smoothed = gaussSmoothen(path_y_acc, sigma_, kernel_size_);
            std::vector<double> path_yaw_smoothed = gaussSmoothen(path_yaw, sigma_, kernel_size_);
            std::vector<double> path_yaw_rate_smoothed = gaussSmoothen(path_yaw_rate, sigma_, kernel_size_);

            for (int i = 0; i < kernel_size_ / 2; ++i) {
                path_x_pos_smoothed.at(i) = path_x_pos.at(i);
                path_y_pos_smoothed.at(i) = target_side * path_y_pos.at(i);
                path_x_vel_smoothed.at(i) = path_x_vel.at(i);
                path_y_vel_smoothed.at(i) = target_side * path_y_vel.at(i);
                path_x_acc_smoothed.at(i) = path_x_acc.at(i);
                path_y_acc_smoothed.at(i) = target_side * path_y_acc.at(i);
                path_yaw_smoothed.at(i) = target_side * path_yaw.at(i);
                path_yaw_rate_smoothed.at(i) = target_side * path_yaw_rate.at(i);
            }

            int number_samples = path_x_pos_smoothed.size();
            double state_yaw = QuaterniondToYaw(state_estimate.orientation);
            double state_x = state_estimate.position(0);
            double state_y = state_estimate.position(1);

            for (int i = 0; i < number_samples; ++i) {
                quad_msgs::QuadDesiredState desired_state;
                desired_state.position.x = cos(state_yaw) * path_x_pos_smoothed.at(0) -
                                           sin(state_yaw) * target_side * (path_y_pos_smoothed.at(0)) + state_x;
                desired_state.position.y = sin(state_yaw) * path_x_pos_smoothed.at(0) +
                                           cos(state_yaw) * target_side * (path_y_pos_smoothed.at(0)) + state_y;
                desired_state.position.z = 1.6;

                desired_state.velocity.x = cos(state_yaw) * path_x_vel_smoothed.at(0) -
                                           sin(state_yaw) * target_side * (path_y_vel_smoothed.at(0));
                desired_state.velocity.y = sin(state_yaw) * path_x_vel_smoothed.at(0) +
                                           cos(state_yaw) * target_side * (path_y_vel_smoothed.at(0));
                desired_state.velocity.z = 0;

                desired_state.acceleration.x = cos(state_yaw) * path_x_acc_smoothed.at(0) -
                                               sin(state_yaw) * target_side * (path_y_acc_smoothed.at(0));
                desired_state.acceleration.y = sin(state_yaw) * path_x_acc_smoothed.at(0) +
                                               cos(state_yaw) * target_side * (path_y_acc_smoothed.at(0));
                desired_state.acceleration.z = 0;

                double desired_yaw = target_side * (path_yaw_smoothed.at(0)) + state_yaw;

                if (desired_yaw > 3.14159) {
                    desired_yaw = -3.14159 + (desired_yaw - 3.14159);
                }

                desired_state.yaw = desired_yaw;
                desired_state.yaw_rate = target_side * (path_yaw_rate_smoothed.at(0));
                desired_state.yaw_acceleration = 0;

                path_x_pos_smoothed.erase(path_x_pos_smoothed.begin());
                path_y_pos_smoothed.erase(path_y_pos_smoothed.begin());
                path_x_vel_smoothed.erase(path_x_vel_smoothed.begin());
                path_y_vel_smoothed.erase(path_y_vel_smoothed.begin());
                path_x_acc_smoothed.erase(path_x_acc_smoothed.begin());
                path_y_acc_smoothed.erase(path_y_acc_smoothed.begin());
                path_yaw_smoothed.erase(path_yaw_smoothed.begin());
                path_yaw_rate_smoothed.erase(path_yaw_rate_smoothed.begin());

                path_smoothed.push_back(desired_state);
            }
        }
        return path_smoothed;
    }

    std::vector<quad_msgs::QuadDesiredState> DesiredStatePublisherYawAdjustment::generateStraightPath(double target_depth, QuadState state_estimate)
    {
        float controller_freq = 50;    // in Hz¡

        controller_freq = super_factor_ * controller_freq;

        // Path straight
        float sample_step_dist = abs_vel / controller_freq;
        float dist_straight_target = target_depth;
        int number_samples_straight = int(dist_straight_target / sample_step_dist);
        float sample_step_dist_total = sample_step_dist;

        std::vector<double> path_x_pos;
        std::vector<double> path_y_pos;
        std::vector<double> path_x_vel;
        std::vector<double> path_y_vel;
        std::vector<double> path_x_acc;
        std::vector<double> path_y_acc;
        std::vector<double> path_yaw;
        std::vector<double> path_yaw_rate;

        for (int i = 0; i < number_samples_straight; ++i)
        {
            float x_pos = sample_step_dist_total;
            float y_pos = 0;
            float x_vel = abs_vel;
            float y_vel = 0;
            float x_acc = 0;
            float y_acc = 0;
            float yaw = 0;
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

        std::vector<quad_msgs::QuadDesiredState> path_smoothed;

        int number_samples = path_x_pos.size();
        quad_msgs::QuadDesiredState desired_state;
        double state_yaw = QuaterniondToYaw(state_estimate.orientation);
        double state_x = state_estimate.position(0);
        double state_y = state_estimate.position(1);

        for (int i = 0; i < number_samples; ++i)
        {
            desired_state.position.x = cos(state_yaw)*path_x_pos.at(0) + state_x;
            desired_state.position.y = sin(state_yaw)*path_x_pos.at(0) + state_y;
            desired_state.position.z = 1.6;

            desired_state.velocity.x = cos(state_yaw)*path_x_vel.at(0);
            desired_state.velocity.y = sin(state_yaw)*path_x_vel.at(0);
            desired_state.velocity.z = 0;

            desired_state.acceleration.x = cos(state_yaw)*path_x_acc.at(0);
            desired_state.acceleration.y = sin(state_yaw)*path_x_acc.at(0);
            desired_state.acceleration.z = 0;

            desired_state.yaw = state_yaw;
            desired_state.yaw_rate = 0;
            desired_state.yaw_acceleration = 0;

            path_x_pos.erase(path_x_pos.begin());
            path_x_vel.erase(path_x_vel.begin());
            path_x_acc.erase(path_x_acc.begin());

            path_smoothed.push_back(desired_state);
        }
        return path_smoothed;
    }

    std::vector<quad_msgs::QuadDesiredState> DesiredStatePublisherYawAdjustment::generateStarterPath()
    {
        float controller_freq = 50;    // in Hz¡

        controller_freq = super_factor_ * controller_freq;

        int number_samples_start = 25*super_factor_;
        float total_vel = 0;
        float vel_diff = abs_vel / number_samples_start;
        float total_dist = 0;

        std::vector<double> path_x_pos;
        std::vector<double> path_y_pos;
        std::vector<double> path_x_vel;
        std::vector<double> path_y_vel;
        std::vector<double> path_x_acc;
        std::vector<double> path_y_acc;
        std::vector<double> path_yaw;
        std::vector<double> path_yaw_rate;

        for (int i = 0; i < number_samples_start; ++i)
        {
            float x_pos = total_dist + total_vel / controller_freq;
            float y_pos = 0;
            float x_vel = total_vel;
            float y_vel = 0;
            float x_acc = 0;
            float y_acc = 0;
            float yaw = 0;
            float yaw_rate = 0;

            path_x_pos.push_back(x_pos);
            path_y_pos.push_back(y_pos);
            path_x_vel.push_back(x_vel);
            path_y_vel.push_back(y_vel);
            path_x_acc.push_back(x_acc);
            path_y_acc.push_back(y_acc);
            path_yaw.push_back(yaw);
            path_yaw_rate.push_back(yaw_rate);

            total_vel = total_vel + vel_diff;
        }

        std::vector<quad_msgs::QuadDesiredState> path_smoothed;

        int number_samples = path_x_pos.size();
        quad_msgs::QuadDesiredState desired_state;

        for (int i = 0; i < number_samples; ++i)
        {
            desired_state.position.x = path_x_pos.at(0);
            desired_state.position.y = 0;
            desired_state.position.z = 1.6;

            desired_state.velocity.x = path_x_vel.at(0);
            desired_state.velocity.y = 0;
            desired_state.velocity.z = 0;

            desired_state.acceleration.x = path_x_acc.at(0);
            desired_state.acceleration.y = 0;
            desired_state.acceleration.z = 0;

            desired_state.yaw = 0;
            desired_state.yaw_rate = 0;
            desired_state.yaw_acceleration = 0;

            path_x_pos.erase(path_x_pos.begin());
            path_x_vel.erase(path_x_vel.begin());
            path_x_acc.erase(path_x_acc.begin());

            path_smoothed.push_back(desired_state);
        }
        return path_smoothed;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "desired_state_publisher");

    depth_flight_controller::DesiredStatePublisherYawAdjustment dsp;

    ros::spin();

    return 0;
}