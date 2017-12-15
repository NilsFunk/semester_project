


//
// Created by nilsiism on 07.11.17.
//
#include "desired_state_publisher.h"

namespace depth_flight_controller {

    DesiredStatePublisher::DesiredStatePublisher()
    {
        //state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &DesiredStatePublisher::stateEstimateCallback, this);
        desired_state_pub_ = nh_.advertise<quad_msgs::QuadDesiredState>("/hummingbird/desired_state", 1);

        sample_switch_frequency_ = 49;
        controller_frequency_ = 50;

        pathGenerator();
        main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / sample_switch_frequency_), &DesiredStatePublisher::mainloop, this);


        //Create a dynamic array to hold the values
        std::vector<int> numbers;

        //Create an input file stream
        std::ifstream in("sample_path.txt",std::ios::in);

        /*
                As long as we haven't reached the end of the file, keep reading entries.
        */

        float number;  //Variable to hold each number as it is read

        //Read number using the extraction (>>) operator
        while (in >> number) {
            //Add the number to the end of the array
            numbers.push_back(number);
        }

        //Close the file stream
        in.close();

        /*
            Now, the vector<int> object "numbers" contains both the array of numbers,
                and its length (the number count from the file).
        */

        //Display the numbers
        std::cout << "Numbers:\n";
        for (int i=0; i<numbers.size(); i++) {
            std::cout << numbers[i] << '\n';
        }
    }

    DesiredStatePublisher::~DesiredStatePublisher()
    {

    }

    void DesiredStatePublisher::mainloop(const ros::TimerEvent& time)
    {
        if (path_.size() > 0)
        {
            quad_msgs::QuadDesiredState desired_state;
            desired_state.header.stamp = ros::Time::now();

            std::vector<double> sample = path_.front();

            desired_state.position.x = sample.at(0);
            desired_state.position.y = sample.at(1);
            desired_state.position.z = 0;

            desired_state.velocity.x = sample.at(2);
            desired_state.velocity.y = sample.at(3);
            desired_state.velocity.z = 0;

            desired_state.acceleration.x = sample.at(4);
            desired_state.acceleration.y = sample.at(5);
            desired_state.acceleration.z = 0;

            desired_state.yaw = sample.at(6);
            desired_state.yaw_rate = sample.at(7);
            desired_state.yaw_acceleration = 0;

            path_.erase(path_.begin());

            desired_state_pub_.publish(desired_state);
        }
    }

    void DesiredStatePublisher::pathGenerator()
    {
        //std::vector<std::vector<double> > path;

        abs_vel_ = 1;
        turning_radius_ = 1;
        abs_acc_ = abs_vel_*abs_vel_/turning_radius_;

        step_dist_ = abs_vel_ / controller_frequency_;

        step_time_ = 0.02;
        sample_step_rad_ = step_dist_ / turning_radius_;

        yaw_rate_ = sample_step_rad_ / step_time_;


        sample_step_total_rad_ = sample_step_rad_;

        double arc_length = 2*3.14;
        int steps = int(arc_length / sample_step_rad_);

        for (int i = 0; i < steps; ++i)
        {
            std::vector<double> sample;
            double x_body = turning_radius_ * sin(sample_step_total_rad_);
            double y_body = turning_radius_ * (1 - cos(sample_step_total_rad_));
            double x_vel = abs_vel_ * cos(sample_step_total_rad_);
            double y_vel = abs_vel_ * sin(sample_step_total_rad_);
            double x_acc = -abs_acc_ * sin(sample_step_total_rad_);
            double y_acc = abs_acc_ * cos(sample_step_total_rad_);

            sample_step_total_rad_ = sample_step_total_rad_ + sample_step_rad_;
            double yaw = sample_step_total_rad_;
            double yaw_rate = yaw_rate_;

            if (sample_step_total_rad_ > 3.14159)
            {
                sample_step_total_rad_ = -3.14159 + (sample_step_total_rad_ - 3.14159);
            }

            sample.push_back(x_body);
            sample.push_back(y_body);
            sample.push_back(x_vel);
            sample.push_back(y_vel);
            sample.push_back(x_acc);
            sample.push_back(y_acc);
            sample.push_back(yaw);
            sample.push_back(yaw_rate);
            path_.push_back(sample);
        }

    }


    void DesiredStatePublisher::setDesiredState(const QuadState &state_estimate)
    {

    }

    /*void DesiredStatePublisher::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        /*state_estimate_ = QuadState(*msg);
        quad_msgs::QuadDesiredState desired_state;
        desired_state.header.stamp = ros::Time::now();
        double yaw = QuaterniondToYaw(state_estimate_.orientation);

        double yaw_update = sample_step_rad_ + yaw;

        if (yaw_update > 3.14159)
        {
            yaw_update = -3.14159 + (yaw_update - 3.14159);
        }

        desired_state.position.x = state_estimate_.position(0) + x_body_ * cos(yaw) - y_body_ * sin(yaw);
        desired_state.position.y = state_estimate_.position(1) + x_body_ * sin(yaw) + y_body_ * cos(yaw);
        desired_state.position.z = 0;

        //desired_state.velocity.x = abs_vel_ * cos(yaw_update);
        //desired_state.velocity.y = abs_vel_ * sin(yaw_update);
        desired_state.velocity.z = 0;

        //desired_state.acceleration.x = -abs_acc_ * sin(yaw_update);
        //desired_state.acceleration.y = abs_acc_ * cos(yaw_update);
        desired_state.acceleration.z = 0;

        //std::cout << "x_body: " << x_body_ << "; y_body: " << y_body_ << std::endl;
        //std::cout << "yaw: " << yaw_update << "; yaw_rate: " << yaw_rate_ << "; yaw_acceleration: " << 0 << std::endl;

        desired_state.yaw = yaw_update;
        desired_state.yaw_rate = yaw_rate_;
        desired_state.yaw_acceleration = 0;

        desired_state_pub_.publish(desired_state);
    }*/

    double DesiredStatePublisher::QuaterniondToYaw(const Eigen::Quaterniond& q)
    {
        // yaw (z-axis rotation)
        double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        double yaw = atan2(siny, cosy);
        //double yaw = atan2(2*q.w()*q.z() + 2*q.x()*q.y(), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
        //std::cout << (360/2/3.14*yaw) << std::endl;

        return yaw;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "desired_state_publisher");

    depth_flight_controller::DesiredStatePublisher dsp;

    ros::spin();

    return 0;
}