//
// Created by nilsiism on 13.11.17.
//

#include "../include/max_depth_finder.h"


namespace depth_flight_controller {

        MaxDepthFinder::MaxDepthFinder()
                : it_(nh_)
        {
            image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/expanded", 1,
                                       &MaxDepthFinder::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);

            cmd_vel_base_frame_pub =  nh_.advertise<geometry_msgs::TwistStamped>("/hummingbird/base_frame_cmd_vel", 1);
            state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &MaxDepthFinder::stateEstimateCallback, this);

            cv::namedWindow(OPENCV_WINDOW01);
            cv::namedWindow(OPENCV_WINDOW02);
        }

        MaxDepthFinder::~MaxDepthFinder()
        {
            cv::destroyWindow(OPENCV_WINDOW01);
            cv::destroyWindow(OPENCV_WINDOW02);
        }

        void MaxDepthFinder::imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv_ptr = cv_bridge::toCvCopy(msg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Update GUI Window

            cv::Mat depth_float_img = cv_ptr->image;
            cv::Mat depth_mono8_img;
            depthToCV8UC1(depth_float_img, depth_mono8_img);

            cv::minMaxLoc(depth_float_img, &min_depth_, &max_depth_, &min_depth_loc_, &max_depth_loc_);

            std::cout << "min :" << min_depth_ << std::endl;
            std::cout << "max :" << max_depth_ << std::endl;


            // Draw an example circle on the video stream
            if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
                cv::circle(depth_mono8_img, cv::Point(max_depth_loc_.x,20), 3, CV_RGB(0,255,0));

            cv::imshow(OPENCV_WINDOW01, depth_float_img);
            cv::imshow(OPENCV_WINDOW02, depth_mono8_img);

            //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);

            MaxDepthFinder::cmdVelPlotter(max_depth_, max_depth_loc_, state_estimate_);

            // Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());
        }

        void MaxDepthFinder::cmdVelPlotter(const int &max_depth, const cv::Point &max_depth_loc, const QuadState &state_estimate)
        {
            int max_depth_loc_x = max_depth_loc.x;
            geometry_msgs::TwistStamped msg_cmd;
            msg_cmd.header.stamp = ros::Time::now();
            if (max_depth <= 0.5) {
                if (state_estimate.bodyrates[2] >= 0)
                {
                    msg_cmd.twist.linear.x = 0.0;
                    msg_cmd.twist.angular.z = 0.3;
                } else
                {
                    msg_cmd.twist.linear.x = 0.0;
                    msg_cmd.twist.angular.z = -0.3;
                }
            } else if (max_depth_loc_x < 78) {
                msg_cmd.twist.linear.x = 0.3;
                msg_cmd.twist.angular.z = 0.3;
            } else if (max_depth_loc_x > 82) {
                msg_cmd.twist.linear.x = 0.3;
                msg_cmd.twist.angular.z = -0.3;
            } else {
                msg_cmd.twist.linear.x = 0.3;
                msg_cmd.twist.angular.z = 0;
            }

            cmd_vel_base_frame_pub.publish(msg_cmd);
        }

        void MaxDepthFinder::depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img)
        {
            //Process images
            if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
                mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
            cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
        }

        void MaxDepthFinder::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg) {
            state_estimate_ = QuadState(*msg); // TODO: Why can the QuadState() method take a pointer as an input?
        }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "max_depth_finder");

    depth_flight_controller::MaxDepthFinder mdf;
    
    ros::spin();

    return 0;
}
