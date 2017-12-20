#include "c_space_expander_horizon.h"


namespace depth_flight_controller {

    CSpaceExpanderHorizon::CSpaceExpanderHorizon()
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

        image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/disparity", 1, &CSpaceExpanderHorizon::imageCallback, this);
        image_pub_ = it_.advertise("/hummingbird/vi_sensor/camera_depth/depth/expanded", 1);

        state_estimate_original_img_pub_ = nh_.advertise<quad_msgs::QuadStateEstimate>("/hummingbird/state_estimate_original_img", 1);
        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate", 1, &CSpaceExpanderHorizon::stateEstimateCallback, this);

        K = (cv::Mat_<double>(3,3)<<151.8076510090423, 0.0, 80.5, 0.0, 151.8076510090423, 60.5, 0.0, 0.0, 1.0);
        T = (cv::Mat_<double>(3,1) <<  0, 0, 0);
        distCoeffs = (cv::Mat_<double>(4,1) <<  0, 0, 0, 0);

        body_cam_rot_ << 0, -1, 0, 0, 0, 1, 1, 0, 0;
        horizon_center_point_world_ << 100, 0, 0;
        horizon_left_point_world_ << 1000, 100, 0;
        horizon_right_point_world_ << 1000, -100, 0;
        rvec = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    }


    CSpaceExpanderHorizon::~CSpaceExpanderHorizon()
    {

    }

    void CSpaceExpanderHorizon::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
        state_estimate_msg_ = *msg;
    }


    void CSpaceExpanderHorizon::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        quad_msgs::QuadStateEstimate state_estimate_original_img_msg = state_estimate_msg_;
        QuadState state_estimate_original_img_ = state_estimate_;


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
        CSpaceExpanderHorizon::changeImagePrecision(depth_float_img_original_, depth_float_img_rounded_); // Input: Original image values in [m]; Output: Rounded image values in [cm]

        std::vector<cv::Point> horizon_points = buildHorizon(state_estimate_);

        // Expand c-space
        CSpaceExpanderHorizon::expandImage(depth_float_img_original_, depth_float_img_rounded_, horizon_points);

        state_estimate_original_img_pub_.publish(state_estimate_original_img_msg);
        image_pub_.publish(cv_ptr_original->toImageMsg());
    }

    float CSpaceExpanderHorizon::roundToPrecision(float imageDepth,int precision)
    {
        // Do initial checks
        float roundedImageDepth = roundf(imageDepth * precision_);
        return roundedImageDepth;
    }

    void CSpaceExpanderHorizon::changeImagePrecision(cv::Mat& IO, cv::Mat& IR)
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
                pR[j] = CSpaceExpanderHorizon::roundToPrecision(pO[j], 2);
            }
        }
    }

    //void CSpaceExpanderHorizon::expandImage(cv::Mat& IO, cv::Mat& IR, cv::Mat& IE)
    void CSpaceExpanderHorizon::expandImage(cv::Mat& IO, cv::Mat& IR, std::vector<cv::Point> horizon_points)
    {
        CV_Assert(IO.depth() == CV_32FC1);

        cv::Point edge_left_pos = horizon_points.at(0);
        cv::Point edge_right_pos = horizon_points.at(1);

        int v_min_edge = std::min(edge_left_pos.y, edge_right_pos.y);
        int v_max_edge = std::max(edge_left_pos.y, edge_right_pos.y);

        int v_min = std::max(5, v_min_edge);
        int v_max = std::min(114,v_max_edge);

        for (int v = v_min-5; v < v_max+6; ++v)
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

    std::vector<cv::Point> CSpaceExpanderHorizon::buildHorizon(const QuadState state_estimate)
    {
        // Calculate edge points of line
        Eigen::Matrix3d rvec_state_estimate = CSpaceExpanderHorizon::tiltCalculator(state_estimate);

        Eigen::Vector3d horizon_center_point_cam = body_cam_rot_ * rvec_state_estimate *horizon_center_point_world_;
        Eigen::Vector3d horizon_left_point_cam = body_cam_rot_ * rvec_state_estimate * horizon_left_point_world_;
        Eigen::Vector3d horizon_right_point_cam = body_cam_rot_ * rvec_state_estimate * horizon_right_point_world_;

        std::vector<cv::Point3f> horizon_3D_points;
        std::vector<cv::Point2f> projected_horizon_points;

        horizon_3D_points.push_back(cv::Point3d(horizon_left_point_cam(0),horizon_left_point_cam(1),horizon_left_point_cam(2)));
        horizon_3D_points.push_back(cv::Point3d(horizon_right_point_cam(0),horizon_right_point_cam(1),horizon_right_point_cam(2)));
        horizon_3D_points.push_back(cv::Point3d(horizon_center_point_cam(0),horizon_center_point_cam(1),horizon_center_point_cam(2)));

        cv::Rodrigues(rvec, rvecR);
        cv::projectPoints( cv::Mat(horizon_3D_points), rvecR, T, K, distCoeffs, projected_horizon_points);

        cv::Point pt1 = cv::Point(projected_horizon_points[0].x,projected_horizon_points[0].y);
        cv::Point pt2 = cv::Point(projected_horizon_points[1].x,projected_horizon_points[1].y);
        cv::Point horizon_center = cv::Point(projected_horizon_points[2].x,projected_horizon_points[2].y);

        std::vector<cv::Point> horizon_points = CSpaceExpanderHorizon::fullLine(pt1, pt2, horizon_center);

        return horizon_points;
    }

    Eigen::Matrix3d CSpaceExpanderHorizon::tiltCalculator(const QuadState &state_estimate)
    {
        Eigen::Quaterniond q = state_estimate.orientation;
        Eigen::Vector3d euler_angles = CSpaceExpanderHorizon::toEulerAngle(q);
        Eigen::Matrix3d state_estimate_rot_mat = eulerAnglesZYXToRotationMatrix(euler_angles);
        Eigen::Matrix3d state_estimate_rot_mat_inv = state_estimate_rot_mat.inverse();
        return state_estimate_rot_mat;
    }

    Eigen::Vector3d CSpaceExpanderHorizon::toEulerAngle(const Eigen::Quaterniond& q)
    {
        double roll = atan2(2*q.w()*q.x() + 2*q.y()*q.z(), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
        double pitch = -asin(2*q.x()*q.z() - 2*q.w()*q.y());
        yaw_ = atan2(2*q.w()*q.z() + 2*q.x()*q.y(), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());

        Eigen::Vector3d euler_angles(roll, pitch, 0);

        return euler_angles;
    }

    std::vector<cv::Point> CSpaceExpanderHorizon::fullLine(cv::Point a, cv::Point b, cv::Point center_pos)
    {
        double slope = Slope(a.x, a.y, b.x, b.y);

        cv::Point edge_left_pos = cv::Point(0,0);
        cv::Point edge_right_pos = cv::Point(depth_float_img_original_.cols-1,depth_float_img_original_.rows-1);

        edge_left_pos.y = -(a.x - edge_left_pos.x) * slope + a.y;
        edge_right_pos.y = -(b.x - edge_right_pos.x) * slope + b.y;

        cv::LineIterator it(depth_float_img_original_, edge_left_pos, edge_right_pos, 8);

        for(int i = 0; i < it.count; i++, ++it)
        {
            if(i == 0)
            {
                edge_left_pos = it.pos();
            }

            if(i == (it.count - 1))
            {
                edge_right_pos = it.pos();
            }
        }

        std::vector<cv::Point> horizon_points;
        horizon_points.push_back(edge_left_pos);
        horizon_points.push_back(edge_right_pos);
        horizon_points.push_back(center_pos);

        return horizon_points;
    }

    double CSpaceExpanderHorizon::Slope(int x0, int y0, int x1, int y1)
    {
        return (double)(y1-y0)/(x1-x0);
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "c_space_expander_horizon");

    depth_flight_controller::CSpaceExpanderHorizon cse;

    ros::spin();

    return 0;
}


