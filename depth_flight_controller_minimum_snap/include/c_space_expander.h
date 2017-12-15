//
// Created by nilsiism on 19.11.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_C_SPACE_EXPANDER_H
#define DEPTH_FLIGHT_CONTROLLER_C_SPACE_EXPANDER_H

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <algorithm>
#include <fstream>
#include "quad_common/geometry_eigen_conversions.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "quad_common/quad_state.h"

namespace depth_flight_controller
{

    using namespace quad_common;

    class CSpaceExpander
    {
    public:
        CSpaceExpander();
        ~CSpaceExpander();

        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);
        float roundToPrecision(float val,int precision);
        void changeImagePrecision(cv::Mat& IO, cv::Mat& IR);
        void expandImage(cv::Mat& IO, cv::Mat& IR);
        void writeMapU(std::ostream& os);
        void writeMapV(std::ostream& os);

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        ros::Subscriber state_estimate_sub_;
        ros::Publisher state_estimate_original_img_pub_;

        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

    private:
        cv::Mat depth_float_img_original_;
        cv::Mat depth_float_img_rounded_;
        cv::Mat depth_float_img_expanded_;
        cv::Mat depth_mono8_img_original_;
        cv::Mat depth_mono8_img_expanded_;
        static const int max_depth_ = 500;
        int utable_[160][max_depth_][2]; // Image width 160, depth 0-300 cm
        int vtable_[120][max_depth_][2];
        float reduced_depth_[max_depth_];
        double focal_length_;
        double drone_radius_;
        static const int precision_ = 100; // 1 := [m]; 1000 := [mm]
        double min_depth_img_, max_depth_img_;
        cv::Point min_depth_loc_, max_depth_loc_;
        QuadState state_estimate_;
        quad_msgs::QuadStateEstimate state_estimate_msg_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_C_SPACE_EXPANDER_H
