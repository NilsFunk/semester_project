//
// Created by nilsiism on 08.12.17.
//

#ifndef DEPTH_FLIGHT_CONTROLLER_IMAGE_CLIPPER_H
#define DEPTH_FLIGHT_CONTROLLER_IMAGE_CLIPPER_H

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

namespace depth_flight_controller
{
    class ImageClipper
    {
    public:
        ImageClipper();
        ~ImageClipper();

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

    private:
        cv::Mat depth_float_img_original_;
        cv::Mat depth_float_img_clipped_;
    };
}

#endif //DEPTH_FLIGHT_CONTROLLER_IMAGE_CLIPPER_H
