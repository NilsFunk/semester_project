//
// Created by nilsiism on 19.11.17.
//


#ifndef DEPTH_FLIGHT_CONTROLLER_TARGET_FINDER_H
#define DEPTH_FLIGHT_CONTROLLER_TARGET_FINDER_H

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
#include "depth_flight_controller_msgs/Target.h"
#include "quad_common/quad_state.h"
#include <iostream>
#include <string>
#include "quad_common/geometry_eigen_conversions.h"
#include "quad_common/math_common.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <math.h>

//static const std::string OPENCV_WINDOW01 = "window name";


namespace depth_flight_controller
{
    using namespace quad_common;

    class TargetFinder
    {
    public:
        TargetFinder();
        ~TargetFinder();

        void expandedImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        Eigen::Matrix3d tiltCalculator(const QuadState &state_estimate);
        Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q);
        double Slope(int x0, int y0, int x1, int y1);
        std::vector<cv::Point> fullLine(cv::Point a, cv::Point b, cv::Point center_pos);
        float euclideanDistSign(cv::Point& p, cv::Point& q);
        float euclideanDist(cv::Point& p, cv::Point& q);
        int leftOfSecArg(cv::Point& p, cv::Point& q);
        void horizonAnalyze(std::vector<cv::Point> horizon_points, quad_msgs::QuadStateEstimate state_estimate_image_msg);
        std::vector<cv::Point> buildHorizon(const QuadState state_estimate);

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber expanded_image_sub_;

        ros::Subscriber state_estimate_sub_;
        ros::Publisher horizon_points_pub_;
        ros::Publisher target_pub_;

    private:
        // Image information
        cv::Mat depth_expanded_img_;
        quad_msgs::QuadStateEstimate state_estimate_msg_;

        // Camera intrinsic and extrinsic information
        cv::Mat K;
        cv::Mat rvecR;
        cv::Mat T;
        cv::Mat distCoeffs;

        Eigen::Matrix3d body_cam_rot_;
        Eigen::Vector3d horizon_center_point_world_;
        Eigen::Vector3d horizon_left_point_world_;
        Eigen::Vector3d horizon_right_point_world_;
        cv::Mat rvec;

        // State information
        QuadState state_estimate_;
        //Eigen::Vector3d body_velocities_;

        // Horizon build information
        cv::Point pt1_; // Horizon point left edge
        cv::Point pt2_; // Horizon point right edge
        std::vector<cv::Point3f> horizon_points;
        std::vector<cv::Point2f> projected_horizon_points;

        // Horizon analysis information
        bool is_max_valid_;
        double yaw_;


        //cv::Point target_;
        //cv::Point obstacle_;

        /*float min_depth_left_, min_depth_right_, depth_edge_left_, depth_edge_right_, min_depth_ib_, max_depth_;
        float obstacle_depth_;
        double depth_center_;
        float max_dist_center_ib_min_;
        float max_dist_center_left_min_;
        float max_dist_center_right_min_;
        float min_dist_center_max_;
        static const int kernel_size = 3;
        static const int kernel_middle = (kernel_size+1)/2;*/

        //bool close_to_wall_;


        
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_TARGET_FINDER_H
