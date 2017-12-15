//
// Created by nilsiism on 14.12.17.
//

//
// Created by nilsiism on 07.11.17.
//
#include "snap_trajectory_planner.h"

namespace depth_flight_controller {

    SnapTrajectoryPlanner::SnapTrajectoryPlanner()
    {
        //is_starting_ = true;
        generate_new_path_ = true;
        do_initialize_ = true;
        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &SnapTrajectoryPlanner::stateEstimateCallback, this);
        //state_estimate_original_sub_ = nh_.subscribe("/hummingbird/state_estimate_original_img", 1, &SnapTrajectoryPlanner::stateEstimateOriginalCallback, this);
        target_sub_ = nh_.subscribe("/hummingbird/target", 1, &SnapTrajectoryPlanner::pathCallback, this);
        desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("/hummingbird/desired_state", 1);

        abs_vel_ = 1.5;
        target_radius = 1;
        super_factor_ = 4;
        sample_switch_frequency_ = 50;

        main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / sample_switch_frequency_), &SnapTrajectoryPlanner::mainloop, this);
        most_recent_path_generation_ = ros::Time::now();

    }

    SnapTrajectoryPlanner::~SnapTrajectoryPlanner()
    {

    }

    void SnapTrajectoryPlanner::mainloop(const ros::TimerEvent& time)
    {
        if (path_.size() > 0)
        {
            //std::cout << path_.size() << std::endl;
            quad_msgs::QuadDesiredState desired_state;
            desired_state = path_.front();
            desired_state.header.stamp = ros::Time::now();
            desired_state_pub_.publish(desired_state);
            path_.erase(path_.begin());
            //curr_state_ = path_.front();
            curr_state_ = desired_state;
        }
    }

    void SnapTrajectoryPlanner::pathCallback(const depth_flight_controller_msgs::Target &msg)
    {
        if (msg.valid == true && ros::Time::now() - most_recent_path_generation_ > ros::Duration(4)) //&& generate_new_path_ == true //
        {
            double current_x_pos = state_estimate_.position(0);
            double current_y_pos = state_estimate_.position(1);
            double original_x_pos = msg.position.x;
            double original_y_pos = msg.position.y;
            double original_yaw = msg.yaw;
            double original_depth = msg.depth;
            double original_Y = msg.Y;
            double current_side;

            double target_x_pos = original_x_pos + original_depth * cos(original_yaw) - original_Y * sin(original_yaw);
            double target_y_pos = original_y_pos + original_depth * sin(original_yaw) + original_Y * cos(original_yaw);

            double current_yaw = QuaterniondToYaw(state_estimate_.orientation);

            double current_depth    = (target_x_pos - current_x_pos) * cos(current_yaw) + (target_y_pos - current_y_pos) * sin(current_yaw);
            double current_Y        = -1*(target_x_pos - current_x_pos) * sin(current_yaw) + (target_y_pos - current_y_pos) * cos(current_yaw);
            double current_angle = atan(current_Y/current_depth);

            if (do_initialize_ == true)
            {
                curr_state_.position.x = state_estimate_.position(0);
                curr_state_.position.y = state_estimate_.position(1);
                curr_state_.position.z = state_estimate_.position(2);
                curr_state_.yaw = current_yaw;
                do_initialize_ = false;
            }

            double target_yaw = current_yaw + current_angle;

            double target_x_vel = abs_vel_*cos(target_yaw);
            double target_y_vel = abs_vel_*sin(target_yaw);

            ros::Time startTime = ros::Time::now();
            most_recent_path_generation_ = ros::Time::now();

            mav_trajectory_generation::Vertex::Vector vertices;
            const int dimension = 4;
            const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
            mav_trajectory_generation::Vertex start(dimension), end(dimension);

            Eigen::Vector4d start_pos(state_estimate_.position(0), state_estimate_.position(1), state_estimate_.position(2), current_yaw);
            Eigen::Vector4d start_vel(state_estimate_.velocity(0), state_estimate_.velocity(1), state_estimate_.velocity(2), curr_state_.yaw_rate);
            //Eigen::Vector4d start_pos(curr_state_.position.x, curr_state_.position.y, 1.6, curr_state_.yaw);
            //Eigen::Vector4d start_vel(curr_state_.velocity.x, curr_state_.velocity.y, curr_state_.velocity.z, curr_state_.yaw_rate);
            Eigen::Vector4d start_acc(curr_state_.acceleration.x, curr_state_.acceleration.y, curr_state_.acceleration.z, curr_state_.yaw_acceleration);
            Eigen::Vector4d start_jerk(curr_state_.jerk.x, curr_state_.jerk.y, curr_state_.jerk.z, 0);
            Eigen::Vector4d start_snap(curr_state_.snap.x, curr_state_.snap.y, curr_state_.snap.z, 0);

            //std::cout << "start pos: " << start_pos << std::endl;
            //std::cout << "start vel: " << start_vel << std::endl;
            //std::cout << "start acc: " << start_acc << std::endl;
            //std::cout << "start jerk: " << start_jerk << std::endl;
            //std::cout << "start snap: " << start_snap << std::endl;

            // Add vertices
            start.addConstraint(mav_trajectory_generation::derivative_order::POSITION, start_pos);
            start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel);
            start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, start_acc);
            start.addConstraint(mav_trajectory_generation::derivative_order::JERK, start_jerk);
            start.addConstraint(mav_trajectory_generation::derivative_order::SNAP, start_snap);
            vertices.push_back(start);

            Eigen::Vector4d target_pos(target_x_pos, target_y_pos, 1.6, target_yaw);
            Eigen::Vector4d target_vel(target_x_vel, target_y_vel, 0, 0);

            //std::cout << "target pos: " << target_pos << std::endl;
            //std::cout << "target vel: " << target_vel << std::endl;

            end.addConstraint(mav_trajectory_generation::derivative_order::POSITION, target_pos);
            end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, target_vel);
            vertices.push_back(end);

            // Determine segments
            std::vector<double> segment_times;
            const double v_max = 2.0;
            const double a_max = 2.0;
            const double magic_fabian_constant = 6.5; // A tuning parameter.
            segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

            // Solve equation
            const int N = 10;
            mav_trajectory_generation::PolynomialOptimization <N> opt(dimension);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            opt.solveLinear();

            // Get segments
            //mav_trajectory_generation::Segment::Vector segments;
            //opt.getSegments(&segments);

            mav_trajectory_generation::Trajectory trajectory;
            opt.getTrajectory(&trajectory);

            mav_msgs::EigenTrajectoryPoint state;
            mav_msgs::EigenTrajectoryPoint::Vector states;

            // Whole trajectory:
            //double sampling_interval = 0.02;
            //bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

            // Sample range:
            int int_pos = mav_trajectory_generation::derivative_order::POSITION;
            int int_vel = mav_trajectory_generation::derivative_order::VELOCITY;
            int int_acc = mav_trajectory_generation::derivative_order::ACCELERATION;
            int int_jerk = mav_trajectory_generation::derivative_order::JERK;
            int int_snap = mav_trajectory_generation::derivative_order::SNAP;
            double t_start = 0;
            double t_end = 10.0;
            double dt = 0.02;
            std::vector <Eigen::VectorXd> position;
            std::vector <Eigen::VectorXd> velocity;
            std::vector <Eigen::VectorXd> acceleration;
            std::vector <Eigen::VectorXd> jerk;
            std::vector <Eigen::VectorXd> snap;
            std::vector<double> sampling_times; // Optional.

            trajectory.evaluateRange(t_start, t_end, dt, int_pos, &position, &sampling_times);

            trajectory.evaluateRange(t_start, t_end, dt, int_vel, &velocity, &sampling_times);

            trajectory.evaluateRange(t_start, t_end, dt, int_acc, &acceleration, &sampling_times);

            trajectory.evaluateRange(t_start, t_end, dt, int_jerk, &jerk, &sampling_times);

            trajectory.evaluateRange(t_start, t_end, dt, int_snap, &snap, &sampling_times);

            std::vector <quad_msgs::QuadDesiredState> path;
            int number_samples = position.size();

            for (int i = 0; i < number_samples; ++i) {

                quad_msgs::QuadDesiredState desired_state;

                Eigen::VectorXd curr_pos = position.at(0);
                Eigen::VectorXd curr_vel = velocity.at(0);
                Eigen::VectorXd curr_acc = acceleration.at(0);
                Eigen::VectorXd curr_jerk = jerk.at(0);
                Eigen::VectorXd curr_snap = snap.at(0);

                desired_state.position.x = curr_pos(0);
                desired_state.position.y = curr_pos(1);
                desired_state.position.z = curr_pos(2);
                desired_state.yaw = curr_pos(3);

                desired_state.velocity.x = curr_vel(0);
                desired_state.velocity.y = curr_vel(1);
                desired_state.velocity.z = curr_vel(2);
                desired_state.yaw_rate = curr_vel(3);

                desired_state.acceleration.x = curr_acc(0);
                desired_state.acceleration.y = curr_acc(1);
                desired_state.acceleration.z = curr_acc(2);
                desired_state.yaw_acceleration = curr_acc(3);

                desired_state.jerk.x = curr_jerk(0);
                desired_state.jerk.y = curr_jerk(1);
                desired_state.jerk.z = curr_jerk(2);

                desired_state.snap.x = curr_snap(0);
                desired_state.snap.y = curr_snap(1);
                desired_state.snap.z = curr_snap(2);

                position.erase(position.begin());
                velocity.erase(velocity.begin());
                acceleration.erase(acceleration.begin());
                jerk.erase(jerk.begin());
                snap.erase(snap.begin());

                path.push_back(desired_state);
            }

            ros::Time endTime = ros::Time::now();

            double end_Time = endTime.toSec();
            double start_Time = startTime.toSec();

            int number_delted_samples = int((end_Time-start_Time)/0.02);

            //std::cout << number_delted_samples << std::endl;

            path.erase(path.begin(),path.begin()+number_delted_samples);

            path_ = path;
        }
        //}

        /*
        if (msg.valid == true && ros::Time::now() - most_recent_path_generation_ > ros::Duration(2)) //&& generate_new_path_ == true
        {
            is_trajectory_valid_ = true;
            //generate_new_path_ = false;
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

            std::cout << "current Y sign: " << current_Y << std::endl;

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

            if (current_side != 0)
            {
                //std::cout << msg.depth << std::endl;
                //ros::Time start = ros::Time::now();
                std::cout << "generate curve path" << std::endl;
                path_new_ = generatePath(current_angle, current_depth, current_side, state_estimate_);
                std::cout << "generated curve path" << std::endl;
                //ros::Time end = ros::Time::now();

            } else
            {
                std::cout << "generate straight path" << std::endl;
                path_new_ = generateStraightPath(current_depth, state_estimate_);
                std::cout << "generated straight path" << std::endl;
            }

            if (is_trajectory_valid_ == true)
            {
                most_recent_path_generation_ = ros::Time::now();
                is_new_path = true;
            }
        }
         */
    }


    void SnapTrajectoryPlanner::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
    }

    void SnapTrajectoryPlanner::stateEstimateOriginalCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_original_ = QuadState(*msg);
    }

    double SnapTrajectoryPlanner::QuaterniondToYaw(const Eigen::Quaterniond& q)
    {
        // yaw (z-axis rotation)
        double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        double yaw = atan2(siny, cosy);

        return yaw;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_trajectory_planner");

    depth_flight_controller::SnapTrajectoryPlanner dsp;

    ros::spin();

    return 0;
}