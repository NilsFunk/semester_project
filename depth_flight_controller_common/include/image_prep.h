//
// Created by nilsiism on 07.12.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_IMAGE_PREP_H
#define DEPTH_FLIGHT_CONTROLLER_IMAGE_PREP_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "quad_msgs/QuadStateEstimate.h"
#include "depth_flight_controller_msgs/HorizonPoints.h"
#include "quad_common/quad_state.h"
#include <iostream>
#include <string>
#include "quad_common/geometry_eigen_conversions.h"
#include "quad_common/math_common.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <math.h>

namespace depth_flight_controller
{

    using namespace quad_common;

    class ImagePrep
    {
    public:
        ImagePrep();
        ~ImagePrep();

        void originalImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void expandedImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void horizonPointsCallback(const depth_flight_controller_msgs::HorizonPoints& msg);
        float euclideanDist(cv::Point& p, cv::Point& q);

        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber expanded_image_sub_;
        image_transport::Subscriber original_image_sub_;

        image_transport::Publisher horizon_image_pub_;
        image_transport::Publisher original_top_image_pub_;
        image_transport::Publisher expanded_top_image_pub_;

        ros::Subscriber horizon_points_sub_;

    private:
        sensor_msgs::Image depth_rgb_img_ros_;
        sensor_msgs::Image original_depth_img_ros_;

        cv::Mat depth_original_img_;
        cv::Mat depth_horizon_img_;

        cv::Point pt1_;
        cv::Point pt2_;
        cv::Point horizon_center_;
        cv::Point max_depth_pos_;
        cv::Point min_depth_left_pos_;
        cv::Point min_depth_right_pos_;
        cv::Point min_depth_ib_pos_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_IMAGE_PREP_H
