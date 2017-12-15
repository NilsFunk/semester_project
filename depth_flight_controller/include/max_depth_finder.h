//
// Created by nilsiism on 19.11.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_MAX_DEPTH_FINDER_H
#define DEPTH_FLIGHT_CONTROLLER_MAX_DEPTH_FINDER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/TwistStamped.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_common/quad_state.h"

static const std::string OPENCV_WINDOW01 = "Float image window";
static const std::string OPENCV_WINDOW02 = "Mono8 image window";

namespace depth_flight_controller {

    using namespace quad_common;

    class MaxDepthFinder {
    public:
        MaxDepthFinder();
        ~MaxDepthFinder();

        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
        void cmdVelPlotter(const int &image_max, const cv::Point &image_max_pos, const QuadState &state_estimate);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        ros::Publisher cmd_vel_base_frame_pub;
        ros::Subscriber state_estimate_sub_;

        double min_depth_, max_depth_;
        cv::Point min_depth_loc_, max_depth_loc_;

    private:
        QuadState state_estimate_;
        Eigen::Vector3d body_velocities_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_MAX_DEPTH_FINDER_H
