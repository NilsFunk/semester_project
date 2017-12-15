//
// Created by nilsiism on 07.11.17.
//

#ifndef WORLD_BASE_TRANSFORM_CMD_VEL_TRANSFORM_H
#define WORLD_BASE_TRANSFORM_CMD_VEL_TRANSFORM_H

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "quad_msgs/QuadStateEstimate.h"

#include "quad_common/quad_state.h"
#include "quad_common/geometry_eigen_conversions.h"
#include "quad_common/math_common.h"

#include "Eigen/Dense"


namespace depth_flight_controller {

    using namespace quad_common;

    class CmdVelTransformer {
    public:
        CmdVelTransformer();
        ~CmdVelTransformer();

       void setCmdVelWorldFrame(const QuadState &state_estimate, const Eigen::Vector3d &body_velocities, const Eigen::Vector3d &body_bodyrates);

       void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

       void cmdVelBaseFrameCallback(const geometry_msgs::TwistStamped::ConstPtr &vel);

       void cmdVelMainloop();

    protected:
        ros::NodeHandle nh_;

        ros::Subscriber state_estimate_sub_;
        ros::Subscriber cmd_vel_base_frame_sub_;

        ros::Publisher cmd_vel_world_frame_pub_;

    private:
        QuadState state_estimate_;
        Eigen::Vector3d body_bodyrates_;
        Eigen::Vector3d body_velocities_;
        Eigen::Vector3d world_bodyrates_;
        Eigen::Vector3d world_velocities_;
    };
}
#endif //WORLD_BASE_TRANSFORM_CMD_VEL_TRANSFORM_H
