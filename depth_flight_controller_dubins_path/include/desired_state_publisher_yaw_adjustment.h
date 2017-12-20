//
// Created by nilsiism on 11.12.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_DESIRED_STATE_PUBLISHER_YAW_ADJUSTMENT_H
#define DEPTH_FLIGHT_CONTROLLER_DESIRED_STATE_PUBLISHER_YAW_ADJUSTMENT_H

#pragma once
#include "ros/ros.h"
#include "quad_msgs/QuadDesiredState.h"
#include <fstream>
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_common/quad_state.h"
#include "depth_flight_controller_msgs/Target.h"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <assert.h>
#include <ctime>
#include <iterator>

namespace depth_flight_controller
{
    using namespace quad_common;

    class DesiredStatePublisherYawAdjustment
    {
    public:
        DesiredStatePublisherYawAdjustment();
        ~DesiredStatePublisherYawAdjustment();

        void setDesiredState(const QuadState &state_estimate);

        void pathCallback(const depth_flight_controller_msgs::Target &target);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        void stateEstimateOriginalCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        void mainloop(const ros::TimerEvent& time);

        std::vector<quad_msgs::QuadDesiredState> generatePath(double target_angle_rad, double target_depth, int target_side, QuadState state_estimate);
        std::vector<quad_msgs::QuadDesiredState> generateStraightPath(double target_depth, QuadState state_estimate);
        std::vector<quad_msgs::QuadDesiredState> generateStarterPath();
        inline std::vector<double> gaussSmoothen(std::vector<double> values, double sigma, int samples);
        inline std::vector<double> gaussKernel(int samples, double sigma);
        inline double gauss(double sigma, double x);

        double QuaterniondToYaw(const Eigen::Quaterniond& q);

    protected:
        ros::NodeHandle nh_;

        ros::Subscriber state_estimate_sub_;
        ros::Subscriber state_estimate_original_sub_;
        ros::Subscriber target_sub_;
        ros::Publisher desired_state_pub_;

        ros::Timer main_loop_timer_;

    private:
        QuadState state_estimate_;
        QuadState state_estimate_original_;
        double controller_frequency_;
        double sample_switch_frequency_;

        bool is_new_path;
        bool is_trajectory_valid_;

        std::vector<quad_msgs::QuadDesiredState> path_;
        std::vector<quad_msgs::QuadDesiredState> path_new_;

        double abs_vel;
        double target_radius;
        double super_factor_;

        time_t tstart, tend;

        ros::Time most_recent_path_generation_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_DESIRED_STATE_PUBLISHER_YAW_ADJUSTMENT_H
