//
// Created by nilsiism on 13.11.17.
//

#include "c_space_expander.h"


namespace depth_flight_controller {

    CSpaceExpander::CSpaceExpander()
            : it_(nh_)
    {
        double focal_length_ = 151.81;
        double drone_radius_ = 0.3;

        // Build (u,z) lookup table
        for (int u = 0; u < 160; ++u)
            for (int z_cm = 0; z_cm <= max_depth_; ++z_cm) {
                float z = float(z_cm) / 100;
                float x_u = (float(u) - 80) * z / focal_length_;
                float alpha_u = atan(x_u/z);
                float dist_to_point = (sqrt(pow(z, 2) + pow(x_u, 2)));

                float alpha_1_u;
                float r_1_x_u;
                float r_2_x_u;
                int u_low;
                int u_high;

                if (dist_to_point > 0.2)
                {
                    if (z > 0.205)
                    {
                        alpha_1_u = asin(drone_radius_ / dist_to_point);
                        r_1_x_u = z * tan(alpha_u - alpha_1_u);
                        r_2_x_u = z * tan(alpha_u + alpha_1_u);

                        u_low = int(focal_length_ * r_1_x_u / z) +80;
                        u_low = std::min(std::max(u_low , 0),159);
                        utable_[u][z_cm][0] = u_low;

                        u_high = int(focal_length_ * r_2_x_u / z+80);
                        u_high = std::min(std::max(u_high , 0),159);
                        utable_[u][z_cm][1] = u_high-u_low+1;

                    } else if (u < 80 )
                    {
                        alpha_1_u = asin(drone_radius_ / dist_to_point);
                        r_2_x_u = z * tan(alpha_u + alpha_1_u);

                        u_low = 0;
                        utable_[u][z_cm][0] = u_low;

                        u_high = int(focal_length_ * r_2_x_u / z+80);
                        u_high = std::min(std::max(u_high , 0),159);
                        utable_[u][z_cm][1] = u_high-u_low+1;
                    } else
                    {
                        alpha_1_u = asin(drone_radius_ / dist_to_point);
                        r_1_x_u = z * tan(alpha_u - alpha_1_u);

                        u_low = int(focal_length_ * r_1_x_u / z) +80;
                        u_low = std::min(std::max(u_low , 0),159);
                        utable_[u][z_cm][0] = u_low;

                        u_high = 159;
                        utable_[u][z_cm][1] = u_high-u_low+1;
                    }

                } else
                {
                    utable_[u][z_cm][0] = 0;
                    utable_[u][z_cm][1] = 159;
                }
            }

        // Build (v,z) lookup table
        for (int v = 0; v < 120; ++v)
            for (int z_cm = 0; z_cm <= max_depth_; ++z_cm) {
                float z = float(z_cm) / 100;
                float y_v = (float(v) - 60) * z / focal_length_;
                float alpha_v = atan(y_v /z);
                float dist_to_point = (sqrt(pow(z, 2) + pow(y_v, 2)));

                float alpha_1_v;
                float r_1_y_v;
                float r_2_y_v;
                int v_low;
                int v_high;

                if (dist_to_point > 0.2)
                {
                    if (z > 0.205)
                    {
                        alpha_1_v = asin(drone_radius_ / dist_to_point);
                        r_1_y_v = z * tan(alpha_v - alpha_1_v);
                        r_2_y_v = z * tan(alpha_v + alpha_1_v);

                        v_low = int(focal_length_ * r_1_y_v / z) +60;
                        v_low = std::min(std::max(v_low , 0),119);
                        vtable_[v][z_cm][0] = v_low;

                        v_high = int(focal_length_ * r_2_y_v / z+60);
                        v_high = std::min(std::max(v_high,0),119);
                        vtable_[v][z_cm][1] = v_high-v_low+1;
                    } else if (v < 60 )
                    {
                        alpha_1_v = asin(drone_radius_ / dist_to_point);
                        r_2_y_v = z * tan(alpha_v + alpha_1_v);

                        v_low = 0;
                        vtable_[v][z_cm][0] = v_low;

                        v_high = int(focal_length_ * r_2_y_v / z+60);
                        v_high = std::min(std::max(v_high,0),119);
                        vtable_[v][z_cm][1] = v_high-v_low+1;
                    } else
                    {
                        alpha_1_v = asin(drone_radius_ / dist_to_point);
                        r_1_y_v = z * tan(alpha_v - alpha_1_v);

                        v_low = int(focal_length_ * r_1_y_v / z) +60;
                        v_low = std::min(std::max(v_low , 0),119);
                        vtable_[v][z_cm][0] = v_low;

                        v_high = 119;
                        vtable_[v][z_cm][1] = v_high-v_low+1;
                    }

                } else
                {
                    alpha_1_v = 0;
                    r_1_y_v = 0;
                    r_2_y_v = 0;
                    v_low = 0;
                    vtable_[v][z_cm][0] = 0;
                    v_high = 0;
                    vtable_[v][z_cm][1] = 119;
                }
            }

        for (int z_cm = 0; z_cm < max_depth_; ++z_cm)
            reduced_depth_[z_cm] = float(std::max(z_cm,20))/100-drone_radius_;

        image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/disparity", 1, &CSpaceExpander::imageCb, this);
        image_pub_ = it_.advertise("/hummingbird/vi_sensor/camera_depth/depth/expanded", 1);

        state_estimate_original_img_pub_ = nh_.advertise<quad_msgs::QuadStateEstimate>("/hummingbird/state_estimate_original_img", 1);
        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &CSpaceExpander::stateEstimateCallback, this);
    }


    CSpaceExpander::~CSpaceExpander()
    {

    }

    void CSpaceExpander::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
        state_estimate_msg_ = *msg;
    }


    void CSpaceExpander::imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        ros::Time start = ros::Time::now();
        quad_msgs::QuadStateEstimate state_estimate_original_img_msg = state_estimate_msg_;



        cv_bridge::CvImagePtr cv_ptr_original;

        try
        {
            cv_ptr_original = cv_bridge::toCvCopy(msg);
            //cv_ptr_expanded = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth_float_img_original_ = cv_ptr_original->image;
        cv::Mat mask = cv::Mat(depth_float_img_original_ != depth_float_img_original_);
        depth_float_img_original_.setTo(4.9, mask);

        depth_float_img_rounded_ = depth_float_img_original_.clone();

        //cv::GaussianBlur(depth_float_img_original_, depth_float_img_original_, cv::Size(3,3), 0, 0 );

        // Round image values to [cm]
        CSpaceExpander::changeImagePrecision(depth_float_img_original_, depth_float_img_rounded_); // Input: Original image values in [m]; Output: Rounded image values in [cm]

        // Expand c-space
        //CSpaceExpander::expandImage(depth_float_img_original_, depth_float_img_rounded_, depth_float_img_expanded_);
        CSpaceExpander::expandImage(depth_float_img_original_, depth_float_img_rounded_);

        // Publish expanded image to "/hummingbird/vi_sensor/camera_depth/depth/expanded"


        ros::Time end = ros::Time::now();
        //std::cout << end-start << std::endl;

        state_estimate_original_img_pub_.publish(state_estimate_original_img_msg);
        image_pub_.publish(cv_ptr_original->toImageMsg());
    }

    float CSpaceExpander::roundToPrecision(float imageDepth,int precision)
    {
        // Do initial checks
        float roundedImageDepth = roundf(imageDepth * precision_);
        return roundedImageDepth;
    }

    void CSpaceExpander::changeImagePrecision(cv::Mat& IO, cv::Mat& IR)
    {
        // Accept only float type matrices
        CV_Assert(IO.depth() == CV_32FC1);

        int channels = IO.channels();
        int nRows = IO.rows;
        int nCols = IO.cols * channels;

        if (IO.isContinuous())
        {
            nCols *= nRows;
            nRows = 1;
        }

        int i,j;
        float* pO;
        float* pR;
        for( i = 0; i < nRows; ++i)
        {
            pO = IO.ptr<float>(i);
            pR = IR.ptr<float>(i);
            for ( j = 0; j < nCols; ++j)
            {
                pR[j] = CSpaceExpander::roundToPrecision(pO[j], 2);
            }
        }
    }

    //void CSpaceExpander::expandImage(cv::Mat& IO, cv::Mat& IR, cv::Mat& IE)
    void CSpaceExpander::expandImage(cv::Mat& IO, cv::Mat& IR)
    {
        CV_Assert(IO.depth() == CV_32FC1);

        for (int v = 0; v < 120; ++v)
        {
            for (int u = 0; u < 160; ++u)
            {
                int z_old = IR.at<float>(v, u);

                if (z_old >= 0)
                {
                    int x = utable_[u][z_old][0];
                    int y = vtable_[v][z_old][0];
                    int w = utable_[u][z_old][1];
                    int h = vtable_[v][z_old][1];
                    float z_new = reduced_depth_[z_old];

                    cv::Rect roi = cv::Rect(x, y, w, h);

                    IO(roi).setTo(z_new, IO(roi) > z_new);
                }
            }
        }
        cv::GaussianBlur(IO, IO, cv::Size( 5, 3), 0, 0 );
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "c_space_expander");

    depth_flight_controller::CSpaceExpander cse;

    ros::spin();

    return 0;
}


