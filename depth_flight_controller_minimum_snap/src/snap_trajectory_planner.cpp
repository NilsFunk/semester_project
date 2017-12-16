#include "snap_trajectory_planner.h"

namespace depth_flight_controller {

    SnapTrajectoryPlanner::SnapTrajectoryPlanner()
    {
        is_state_estimate_init_ = false;
        do_initialize_ = true;

        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &SnapTrajectoryPlanner::stateEstimateCallback, this);
        target_sub_ = nh_.subscribe("/hummingbird/target", 1, &SnapTrajectoryPlanner::pathCallback, this);

        desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("/hummingbird/desired_state", 1);
        path_pub_ = nh_.advertise<depth_flight_controller_msgs::PathPositions>("/hummingbird/path", 1);

        abs_vel_ = 2.0;

        main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / 50), &SnapTrajectoryPlanner::mainloop, this);

        most_recent_path_generation_ = ros::Time::now();
    }

    SnapTrajectoryPlanner::~SnapTrajectoryPlanner()
    {

    }

    void SnapTrajectoryPlanner::mainloop(const ros::TimerEvent& time)
    {
        if (path_.size() > 0)
        {
            quad_msgs::QuadDesiredState desired_state;
            desired_state = path_.front();
            if (is_new_path_ == true)
            {
                std::cout << "Distance x-direction while publishing - state - desired: " << (state_estimate_.position(0)-desired_state.position.x) << std::endl;
                std::cout << "Distance y-direction while publishing - state - desired: " << (state_estimate_.position(1)-desired_state.position.y) << std::endl;
                is_new_path_ = false;
            }
            desired_state.header.stamp = ros::Time::now();
            desired_state_pub_.publish(desired_state);
            path_.erase(path_.begin());
            //curr_path_ = path_;
            curr_state_ = desired_state;
        }
    }

    void SnapTrajectoryPlanner::pathCallback(const depth_flight_controller_msgs::Target &msg)
    {
        if (msg.valid == true && ros::Time::now() - most_recent_path_generation_ > ros::Duration(2) && is_state_estimate_init_ == true) {
            double state_x_pos = state_estimate_.position(0);
            double state_y_pos = state_estimate_.position(1);

            double img_state_x_pos = msg.position.x;
            double img_state_y_pos = msg.position.y;
            double img_state_yaw = msg.yaw;
            double img_target_depth = msg.depth;
            double img_target_Y = msg.Y;
            double img_obstacle_depth = msg.obstacle_depth;
            double img_obstacle_Y = msg.obstacle_Y;
            double state_side;

            double target_x_pos =
                    img_state_x_pos + img_target_depth * cos(img_state_yaw) - img_target_Y * sin(img_state_yaw);
            double target_y_pos =
                    img_state_y_pos + img_target_depth * sin(img_state_yaw) + img_target_Y * cos(img_state_yaw);

            double obstacle_x_pos =
                    img_state_x_pos + img_obstacle_depth * cos(img_state_yaw) - img_obstacle_Y * sin(img_state_yaw);
            double obstacle_y_pos =
                    img_state_y_pos + img_obstacle_depth * sin(img_state_yaw) + img_obstacle_Y * cos(img_state_yaw);

            double state_yaw = QuaterniondToYaw(state_estimate_.orientation);

            double state_target_depth =
                    (target_x_pos - state_x_pos) * cos(state_yaw) + (target_y_pos - state_y_pos) * sin(state_yaw);
            double state_target_Y =
                    -1 * (target_x_pos - state_x_pos) * sin(state_yaw) + (target_y_pos - state_y_pos) * cos(state_yaw);
            double state_target_yaw = atan(state_target_Y / state_target_depth);

            double target_yaw = state_yaw + state_target_yaw;

            double target_x_vel = abs_vel_ * cos(target_yaw);
            double target_y_vel = abs_vel_ * sin(target_yaw);

            ros::Time startTime = ros::Time::now();
            most_recent_path_generation_ = ros::Time::now();

            mav_trajectory_generation::Vertex::Vector vertices;
            const int dimension = 4;
            const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
            mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

            /*
            if (curr_path_.size() > 2)
            {
                curr_path_.erase(curr_path_.begin(), curr_path_.begin()+1);
                curr_state_ = curr_path_.front();
            }
            */


            if (do_initialize_ == true) {
                curr_state_.position.x = state_estimate_.position(0);
                curr_state_.position.y = state_estimate_.position(1);
                curr_state_.position.z = state_estimate_.position(2);
                curr_state_.yaw = state_yaw;
                do_initialize_ == false;
            }

            std::cout << "Distance x-direction while path planning - state - desired: "
                      << (state_estimate_.position(0) - curr_state_.position.x) << std::endl;
            std::cout << "Distance y-direction while path planning - state - desired: "
                      << (state_estimate_.position(1) - curr_state_.position.y) << std::endl;


            // Optional define start of curve as current position
            /*
            Eigen::Vector4d start_pos(state_estimate_.position(0), state_estimate_.position(1), state_estimate_.position(2), state_yaw);
            Eigen::Vector4d start_vel(state_estimate_.velocity(0), state_estimate_.velocity(1), state_estimate_.velocity(2), curr_state_.yaw_rate);
            */

            Eigen::Vector4d start_pos(curr_state_.position.x, curr_state_.position.y, curr_state_.position.z,
                                      curr_state_.yaw);
            Eigen::Vector4d start_vel(curr_state_.velocity.x, curr_state_.velocity.y, curr_state_.velocity.z,
                                      curr_state_.yaw_rate);
            Eigen::Vector4d start_acc(curr_state_.acceleration.x, curr_state_.acceleration.y,
                                      curr_state_.acceleration.z, curr_state_.yaw_acceleration);
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


            if (img_obstacle_depth < 2 || img_target_depth / img_obstacle_depth > 1.5) {
                Eigen::Vector4d obstacle_pos(obstacle_x_pos, obstacle_y_pos, 1.6, target_yaw);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, obstacle_pos);
                vertices.push_back(middle);
            }

            Eigen::Vector4d target_pos(target_x_pos, target_y_pos, 1.6, target_yaw);
            Eigen::Vector4d target_vel(target_x_vel, target_y_vel, 0, 0);

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

            mav_trajectory_generation::Trajectory trajectory;
            opt.getTrajectory(&trajectory);

            mav_msgs::EigenTrajectoryPoint state;
            mav_msgs::EigenTrajectoryPoint::Vector states;

            // Different way to get trajectory samples
            /*
            Whole trajectory:
            double sampling_interval = 0.02;
            bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
            */

            // Sample range:
            int int_pos = mav_trajectory_generation::derivative_order::POSITION;
            int int_vel = mav_trajectory_generation::derivative_order::VELOCITY;
            int int_acc = mav_trajectory_generation::derivative_order::ACCELERATION;
            int int_jerk = mav_trajectory_generation::derivative_order::JERK;
            int int_snap = mav_trajectory_generation::derivative_order::SNAP;
            double t_start = 0;
            double t_end = 3.0;
            double dt = 0.02;
            std::vector <Eigen::VectorXd> position;
            std::vector <Eigen::VectorXd> velocity;
            std::vector <Eigen::VectorXd> acceleration;
            std::vector <Eigen::VectorXd> jerk;
            std::vector <Eigen::VectorXd> snap;
            std::vector<double> sampling_times;

            trajectory.evaluateRange(t_start, t_end, dt, int_pos, &position, &sampling_times);
            trajectory.evaluateRange(t_start, t_end, dt, int_vel, &velocity, &sampling_times);
            trajectory.evaluateRange(t_start, t_end, dt, int_acc, &acceleration, &sampling_times);
            trajectory.evaluateRange(t_start, t_end, dt, int_jerk, &jerk, &sampling_times);
            trajectory.evaluateRange(t_start, t_end, dt, int_snap, &snap, &sampling_times);

            std::vector <quad_msgs::QuadDesiredState> path;
            int number_samples = position.size();

            depth_flight_controller_msgs::PathPositions path_positions_msg;
            int every_n_path_position = 0;

            for (int i = 0; i < number_samples; ++i) {

                depth_flight_controller_msgs::PathPosition path_position_msg;

                quad_msgs::QuadDesiredState desired_state;

                Eigen::VectorXd desired_pos = position.at(0);
                Eigen::VectorXd desired_vel = velocity.at(0);
                Eigen::VectorXd desired_acc = acceleration.at(0);
                Eigen::VectorXd desired_jerk = jerk.at(0);
                Eigen::VectorXd desired_snap = snap.at(0);

                desired_state.position.x = desired_pos(0);
                desired_state.position.y = desired_pos(1);
                desired_state.position.z = desired_pos(2);
                desired_state.yaw = desired_pos(3);

                desired_state.velocity.x = desired_vel(0);
                desired_state.velocity.y = desired_vel(1);
                desired_state.velocity.z = desired_vel(2);
                desired_state.yaw_rate = desired_vel(3);

                desired_state.acceleration.x = desired_acc(0);
                desired_state.acceleration.y = desired_acc(1);
                desired_state.acceleration.z = desired_acc(2);
                desired_state.yaw_acceleration = desired_acc(3);

                desired_state.jerk.x = desired_jerk(0);
                desired_state.jerk.y = desired_jerk(1);
                desired_state.jerk.z = desired_jerk(2);

                desired_state.snap.x = desired_snap(0);
                desired_state.snap.y = desired_snap(1);
                desired_state.snap.z = desired_snap(2);

                position.erase(position.begin());
                velocity.erase(velocity.begin());
                acceleration.erase(acceleration.begin());
                jerk.erase(jerk.begin());
                snap.erase(snap.begin());

                if (every_n_path_position == 0)
                {
                    path_position_msg.x_pos = (desired_pos(0)-curr_state_.position.x)*cos(state_yaw) + (desired_pos(1)-curr_state_.position.y) * sin(state_yaw);
                    path_position_msg.y_pos = (desired_pos(0)-curr_state_.position.x)*(-1)*sin(state_yaw) + (desired_pos(1)-curr_state_.position.y) * cos(state_yaw);
                    path_positions_msg.path_positions.push_back(path_position_msg);
                }

                ++every_n_path_position;

                if (every_n_path_position == 1)
                {
                    every_n_path_position = 0;
                }

                path.push_back(desired_state);
            }

            path_pub_.publish(path_positions_msg);

            ros::Time endTime = ros::Time::now();

            double end_Time = endTime.toSec();
            double start_Time = startTime.toSec();

            int number_delted_samples = int((end_Time-start_Time)/0.02) + 2;

            path.erase(path.begin(),path.begin()+number_delted_samples);
            path_.clear();
            path_ = path;

            is_new_path_ = true;
        }
    }


    void SnapTrajectoryPlanner::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
        is_state_estimate_init_ = true;
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