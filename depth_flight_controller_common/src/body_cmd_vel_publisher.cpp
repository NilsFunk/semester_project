//#pragma once
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "body_cmd_velocity_publisher");

    ros::NodeHandle nh_;

    ros::Publisher cmd_vel_base_frame_pub = nh_.advertise<geometry_msgs::TwistStamped>("/hummingbird/base_frame_cmd_vel_fixed", 1);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        geometry_msgs::TwistStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.twist.linear.x = 0.6;
        msg.twist.angular.z = 1;

        cmd_vel_base_frame_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

