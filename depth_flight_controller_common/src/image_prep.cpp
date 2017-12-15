#include "image_prep.h"

namespace depth_flight_controller
{
    ImagePrep::ImagePrep()
            : it_(nh_)
    {
        expanded_image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/expanded", 1, &ImagePrep::expandedImageCallback, this);
        original_image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/clipped", 1, &ImagePrep::originalImageCallback, this);
        horizon_points_sub_ = nh_.subscribe("/hummingbird/horizon_points" , 1, &ImagePrep::horizonPointsCallback, this);

        horizon_image_pub_ = it_.advertise("/hummingbird/vi_sensor/camera_depth/depth/horizon", 1);
        original_top_image_pub_ = it_.advertise("/hummingbird/vi_sensor/camera_depth/depth/disparity_top_view", 1);
        expanded_top_image_pub_ = it_.advertise("/hummingbird/vi_sensor/camera_depth/depth/expanded_top_view", 1);
    }

    ImagePrep::~ImagePrep()
    {
    }

    void ImagePrep::expandedImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr_expanded;
        cv_bridge::CvImage cv_horizon;
        cv_bridge::CvImage cv_expended_top;
        cv_bridge::CvImage cv_original_top;

        try
        {
            cv_ptr_expanded = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_expanded_img_ = cv_ptr_expanded->image;
        cv::Mat depth_original_img = depth_original_img_;

        cv::Mat depth_mono8_img;
        depthToCV8UC1(depth_expanded_img_, depth_mono8_img);
        cv::cvtColor(depth_mono8_img, depth_horizon_img_, cv::COLOR_GRAY2BGR);

        // Draw an example circle on the video stream
        cv::circle(depth_horizon_img_, horizon_center_, 3, CV_RGB(0,0,255));;
        line(depth_horizon_img_,pt1_,pt2_,CV_RGB(0,0,0),1,8,0);



        cv::LineIterator it(depth_horizon_img_, pt1_, pt2_, 8);
        cv::LineIterator it2 = it;
        std::vector<float> buf(it.count);

        cv::Mat depth_original_top_img(500,530,CV_8UC1,cv::Scalar(255));
        cv::Mat depth_expanded_top_img(500,530,CV_8UC1,cv::Scalar(255));

        for(int i = 0; i < it2.count; i++, ++it2)
        {
            cv::Point current_pos = cv::Point(it2.pos().x, it2.pos().y);
            //std::cout << "current_pos: " << current_pos << std::endl;
            //std::cout << "horizon_center: " << horizon_center_ << std::endl;

            float dist_horizon = euclideanDist(current_pos, horizon_center_);
            //std::cout << "dist_horizon: " << dist_horizon << std::endl;

            float expanded_depth_val = depth_expanded_img_.at<float>(it2.pos());
            float original_depth_val = depth_original_img.at<float>(it2.pos());



            int original_x_pos = int(original_depth_val * dist_horizon / 1.5180765 +265);
            int expanded_x_pos = int(expanded_depth_val * dist_horizon / 1.5180765 +265);

            //std::cout << "expanded_x_pos: " << expanded_x_pos << "; expanded_depth: " << expanded_depth_val << std::endl;
            //std::cout << "original_x_pos: " << original_x_pos << "; original_depth: " << original_depth_val << std::endl << std::endl;

            int expanded_depth_val_cm = 500-int(expanded_depth_val*100);
            int original_depth_val_cm = 500-int(original_depth_val*100);

            //std::cout << "original_x_pos: " << original_x_pos << "; original_depth_val_cm: " << original_depth_val_cm << std::endl << std::endl;

            depth_original_top_img.at<uchar>(cv::Point(original_x_pos, original_depth_val_cm)) = 0;
            depth_expanded_top_img.at<uchar>(cv::Point(expanded_x_pos, expanded_depth_val_cm)) = 0;
        }

        cv::circle(depth_horizon_img_, max_depth_pos_, 3, CV_RGB(255,0,0));
        cv::circle(depth_horizon_img_, min_depth_left_pos_, 3, CV_RGB(100,100,0));
        cv::circle(depth_horizon_img_, min_depth_right_pos_, 3, CV_RGB(0,100,100));
        cv::circle(depth_horizon_img_, min_depth_ib_pos_, 3, CV_RGB(100,0,100));
        //cv::circle(depth_original_top_img, cv::Point(265,500), 30, CV_RGB(255,0,0));
        //cv::circle(depth_expanded_top_img, cv::Point(265,500), 2, CV_RGB(255,0,0));

        int original_fov_right = int(-2*80 +265);
        cv::Point original_fov_right_pt = cv::Point(original_fov_right, 500-2*151.8);
        int original_fov_left  = int(2*80 +265);
        cv::Point original_fov_left_pt = cv::Point(original_fov_left, 500-2*151.8);
        int expanded_fov_right = int(-2*80 +265);
        cv::Point expanded_fov_right_pt = cv::Point(expanded_fov_right, 500-2*151.8);
        int expanded_fov_left  = int(2*80 +265);
        cv::Point expanded_fov_left_pt = cv::Point(expanded_fov_left, 500-2*151.8);

        line(depth_original_top_img,cv::Point(265,500), original_fov_right_pt,100,1,8,0);
        line(depth_original_top_img,cv::Point(265,500),original_fov_left_pt,100,1,8,0);
        line(depth_expanded_top_img,cv::Point(265,500),expanded_fov_right_pt,100,1,8,0);
        line(depth_expanded_top_img,cv::Point(265,500),expanded_fov_left_pt,100,1,8,0);

        cv::circle(depth_expanded_top_img, cv::Point(265+25,500-47), 3, CV_RGB(255,0,0));
        cv::circle(depth_expanded_top_img, cv::Point(265-25,500-47), 3, CV_RGB(255,0,0));

        //ellipse(depth_expanded_top_img,cv::Point(280,400), cv::Size(70,70),0,0,350, 100, 1, 8,0);

        cv::waitKey(3);

        //cv::imshow(OPENCV_WINDOW01, image_name);

        cv_horizon.header.stamp = ros::Time::now();
        cv_horizon.header.frame_id = "horizon_view_image";
        cv_horizon.encoding = "rgb8";
        cv_horizon.image = depth_horizon_img_;
        sensor_msgs::Image horizon_img;
        cv_horizon.toImageMsg(horizon_img);
        horizon_image_pub_.publish(horizon_img);

        cv_expended_top.header.stamp = ros::Time::now();
        cv_expended_top.header.frame_id = "expanded_top_view_image";
        cv_expended_top.encoding = "mono8";
        cv_expended_top.image = depth_expanded_top_img;
        sensor_msgs::Image expanded_top_image;
        cv_expended_top.toImageMsg(expanded_top_image);
        expanded_top_image_pub_.publish(expanded_top_image);

        cv_original_top.header.stamp = ros::Time::now();
        cv_original_top.header.frame_id = "original_top_view_image";
        cv_original_top.encoding = "mono8";
        cv_original_top.image = depth_original_top_img;
        sensor_msgs::Image original_top_image;
        cv_original_top.toImageMsg(original_top_image);
        original_top_image_pub_.publish(original_top_image);
    }

    void ImagePrep::originalImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr_original;

        try
        {
            cv_ptr_original = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth_original_img_ = cv_ptr_original->image;
    }

    void ImagePrep::horizonPointsCallback(const depth_flight_controller_msgs::HorizonPoints& msg)
    {
        pt1_ = cv::Point(msg.pt_left.x, msg.pt_left.y);
        pt2_ = cv::Point(msg.pt_right.x, msg.pt_right.y);
        horizon_center_ = cv::Point(msg.pt_center.x, msg.pt_center.y);
        max_depth_pos_ = cv::Point(msg.pt_max_depth.x, msg.pt_max_depth.y);
        min_depth_left_pos_ = cv::Point(msg.pt_min_depth_left.x, msg.pt_min_depth_left.y);
        min_depth_right_pos_ = cv::Point(msg.pt_min_depth_right.x, msg.pt_min_depth_right.y);
        min_depth_ib_pos_ = cv::Point(msg.pt_min_depth_ib.x, msg.pt_min_depth_ib.y);
    }

    float ImagePrep::euclideanDist(cv::Point& p, cv::Point& q)
    {
        cv::Point diff = p - q;
        return copysign(cv::sqrt(diff.x*diff.x + diff.y*diff.y),(p.x-q.x));
    }

    void ImagePrep::depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img)
    {
        //Process images
        if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
            mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
        cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_prep");

    depth_flight_controller::ImagePrep ip;
    ros::spin();

    return 0;
}