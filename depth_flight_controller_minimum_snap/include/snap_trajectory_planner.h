//
// Created by nilsiism on 14.12.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_SNAP_TRAJECTORY_PLANNER_H
#define DEPTH_FLIGHT_CONTROLLER_SNAP_TRAJECTORY_PLANNER_H

#pragma once
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include "ros/ros.h"
#include "quad_msgs/QuadDesiredState.h"
#include <fstream>
#include "quad_msgs/QuadStateEstimate.h"
#include "depth_flight_controller_msgs/PathPosition.h"
#include "depth_flight_controller_msgs/PathPositions.h"
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

    class SnapTrajectoryPlanner
    {
    public:
        SnapTrajectoryPlanner();
        ~SnapTrajectoryPlanner();

        void setDesiredState(const QuadState &state_estimate);

        void pathCallback(const depth_flight_controller_msgs::Target &target);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        void stateEstimateOriginalCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        void mainloop(const ros::TimerEvent& time);
        

        double QuaterniondToYaw(const Eigen::Quaterniond& q);

    protected:
        ros::NodeHandle nh_;

        ros::Subscriber state_estimate_sub_;
        ros::Subscriber state_estimate_original_sub_;
        ros::Subscriber target_sub_;
        ros::Publisher desired_state_pub_;
        ros::Publisher path_pub_;

        ros::Timer main_loop_timer_;

    private:
        QuadState state_estimate_;
        QuadState state_estimate_original_;

        bool generate_new_path_;
        bool is_new_path_;
        bool is_state_estimate_init_;
        bool is_trajectory_valid_;
        bool do_initialize_;

        std::vector<quad_msgs::QuadDesiredState> path_;
        std::vector<quad_msgs::QuadDesiredState> curr_path_;
        quad_msgs::QuadDesiredState curr_state_;

        double abs_vel_;

        ros::Time most_recent_path_generation_;
    };
}



#endif //DEPTH_FLIGHT_CONTROLLER_SNAP_TRAJECTORY_PLANNER_H
