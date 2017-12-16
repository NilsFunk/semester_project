#include "cmd_vel_transformer.h"

namespace depth_flight_controller {
    CmdVelTransformer::CmdVelTransformer()
    {
        cmd_vel_world_frame_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/hummingbird/copilot/manual_desired_velocity", 1);
        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &CmdVelTransformer::stateEstimateCallback, this);
        cmd_vel_base_frame_sub_ = nh_.subscribe("hummingbird/base_frame_cmd_vel_fixed", 1, &CmdVelTransformer::cmdVelBaseFrameCallback, this);
        //cmd_vel_base_frame_sub_ = nh_.subscribe("hummingbird/base_frame_cmd_vel", 1, &CmdVelTransformer::cmdVelBaseFrameCallback, this);
    }

    CmdVelTransformer::~CmdVelTransformer()
    {

    }

    void CmdVelTransformer::cmdVelMainloop()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            CmdVelTransformer::setCmdVelWorldFrame(state_estimate_, body_velocities_, body_bodyrates_);
        }
    }

    void CmdVelTransformer::setCmdVelWorldFrame(const QuadState &state_estimate, const Eigen::Vector3d &body_velocities, const Eigen::Vector3d &body_bodyrates)
    {
        Eigen::Matrix3d state_estimate_rot_mat = quaternionToRotationMatrix(state_estimate.orientation);
        Eigen::Vector3d world_velocities = state_estimate_rot_mat * body_velocities;
        Eigen::Vector3d world_bodyrates = state_estimate_rot_mat * body_bodyrates;
        geometry_msgs::TwistStamped cmd_vel_world_frame;
        cmd_vel_world_frame.header.stamp = ros::Time::now();
        if(world_velocities(0)<1 && world_velocities(1)<1 && world_velocities(2)<1 && world_bodyrates(0) < 1 && world_bodyrates(1)<1 && world_bodyrates(2)<1)
        {
            cmd_vel_world_frame.twist.linear = eigenToGeometry(world_velocities);
            cmd_vel_world_frame.twist.angular = eigenToGeometry(world_bodyrates);
        } else
        {
            Eigen::Vector3d world_linear(0.0,0.0,0.0);
            Eigen::Vector3d world_angular(0.0,0.0,0.0);
            cmd_vel_world_frame.twist.linear = eigenToGeometry(world_linear);
            cmd_vel_world_frame.twist.angular = eigenToGeometry(world_angular);
        }

        cmd_vel_world_frame_pub_.publish(cmd_vel_world_frame);
    }

    void CmdVelTransformer::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
    }

    void CmdVelTransformer::cmdVelBaseFrameCallback(const geometry_msgs::TwistStamped::ConstPtr &vel)
    {
        body_velocities_ = geometryToEigen(vel->twist.linear);
        body_bodyrates_ = geometryToEigen(vel->twist.angular);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_transformer");

    depth_flight_controller::CmdVelTransformer cmd_vel_transformer;

    cmd_vel_transformer.cmdVelMainloop();

    return 0;
}