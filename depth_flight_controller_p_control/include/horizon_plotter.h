//
// Created by nilsiism on 19.11.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_HORIZON_PLOTTER_H
#define DEPTH_FLIGHT_CONTROLLER_HORIZON_PLOTTER_H

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

//static const std::string OPENCV_WINDOW01 = "window name";


namespace depth_flight_controller
{
    using namespace quad_common;

    class HorizonPlotter
    {
    public:
        HorizonPlotter();
        ~HorizonPlotter();

        void expandedImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg);

        Eigen::Matrix3d tiltCalculator(const QuadState &state_estimate);
        void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
        double Slope(int x0, int y0, int x1, int y1);
        void fullLine(cv::Point a, cv::Point b);
        float euclideanDistSign(cv::Point& p, cv::Point& q);
        float euclideanDist(cv::Point& p, cv::Point& q);
        int leftOfSecArg(cv::Point& p, cv::Point& q);
        void cmdVelPlotter();
        void cmdVelPlotter2(const int &max_depth, const cv::Point &max_depth_pos, cv::Point &horizon_center_pos, const QuadState &state_estimate);
        void horizonAnalyze();
        void buildHorizon();
        void plotHorizonPoints();

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber expanded_image_sub_;

        ros::Subscriber state_estimate_sub_;
        ros::Publisher cmd_vel_base_frame_pub_;
        ros::Publisher horizon_points_pub_;

    private:
        // Image information
        cv::Mat depth_expanded_img_;

        // Camera intrinsic and extrinsic information
        cv::Mat K;
        cv::Mat rvecR;
        cv::Mat T;
        cv::Mat distCoeffs;

        // State information
        double roll_, pitch_, yaw_;
        QuadState state_estimate_;
        Eigen::Vector3d body_velocities_;

        // Horizon build information
        cv::Point pt1_; // Horizon point left edge
        cv::Point pt2_; // Horizon point right edge
        std::vector<cv::Point3f> generate3DPoints();
        std::vector<cv::Point3f> horizonPoints;
        std::vector<cv::Point2f> projectedHorizonPoints;

        // Horizon analysis information
        std::vector<cv::Point> free_space_points;
        std::vector<cv::Point> conv_center_point_;

        cv::Point min_depth_left_pos_;
        cv::Point min_depth_right_pos_;
        cv::Point left_edge_pos_;
        cv::Point right_edge_pos_;
        cv::Point min_depth_ib_pos_; // min depth inbetween horizon_center_ and max_depth_pos_
        cv::Point max_depth_pos_;
        cv::Point horizon_center_;

        cv::Point target_;
        cv::Point obstacle_;

        float min_depth_left_, min_depth_right_, depth_edge_left_, depth_edge_right_, min_depth_ib_, max_depth_;
        float obstacle_depth_;
        double depth_center_;
        float max_dist_center_ib_min_;
        float max_dist_center_left_min_;
        float max_dist_center_right_min_;
        float min_dist_center_max_;
        static const int kernel_size = 3;
        static const int kernel_middle = (kernel_size+1)/2;

        bool close_to_wall_;

        depth_flight_controller_msgs::HorizonPoints hps;

        // Control information
        float acc_;

        bool rotate_left_;
        bool rotate_right_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_HORIZON_PLOTTER_H
