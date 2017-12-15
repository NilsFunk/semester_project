//
// Created by nilsiism on 13.11.17.
//

#include "target_finder.h"

namespace depth_flight_controller
{
    TargetFinder::TargetFinder()
            : it_(nh_)
    {
        expanded_image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_depth/depth/expanded", 1, &TargetFinder::expandedImageCallback, this);

        state_estimate_sub_ = nh_.subscribe("/hummingbird/state_estimate_original_img", 1, &TargetFinder::stateEstimateCallback, this);

        target_pub_ = nh_.advertise<depth_flight_controller_msgs::Target>("/hummingbird/target", 1);

        horizon_points_pub_ =  nh_.advertise<depth_flight_controller_msgs::HorizonPoints>("/hummingbird/horizon_points", 1);

        K = (cv::Mat_<double>(3,3)<<151.8076510090423, 0.0, 80.5, 0.0, 151.8076510090423, 60.5, 0.0, 0.0, 1.0);
        T = (cv::Mat_<double>(3,1) <<  0, 0, 0);
        distCoeffs = (cv::Mat_<double>(4,1) <<  0, 0, 0, 0);

        body_cam_rot_ << 0, -1, 0, 0, 0, 1, 1, 0, 0;
        horizon_center_point_world_ << 100, 0, 0;
        horizon_left_point_world_ << 1000, 100, 0;
        horizon_right_point_world_ << 1000, -100, 0;
        rvec = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    TargetFinder::~TargetFinder()
    {

    }

    Eigen::Vector3d TargetFinder::toEulerAngle(const Eigen::Quaterniond& q)
    {
        double roll = atan2(2*q.w()*q.x() + 2*q.y()*q.z(), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
        double pitch = -asin(2*q.x()*q.z() - 2*q.w()*q.y());
        yaw_ = atan2(2*q.w()*q.z() + 2*q.x()*q.y(), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());

        //double roll = 5*2*3.14/360;// 5*2*3.14/360;
        //double pitch = 5*2*3.14/360;
        //double yaw = 0;

        //std::cout << "roll: " << roll << std::endl;
        //std::cout << "pitch: " << pitch << std::endl;
        //std::cout << "yaw: " << yaw << std::endl;

        Eigen::Vector3d euler_angles(roll, pitch, 0);

        return euler_angles;
    }

    Eigen::Matrix3d TargetFinder::tiltCalculator(const QuadState &state_estimate)
    {
        Eigen::Quaterniond q = state_estimate.orientation;
        Eigen::Vector3d euler_angles = TargetFinder::toEulerAngle(q);
        //std::cout << "euler_angles: " << euler_angles << std::endl;
        Eigen::Matrix3d state_estimate_rot_mat = eulerAnglesZYXToRotationMatrix(euler_angles);
        //std::cout << "rotation_mat: " << state_estimate_rot_mat << std::endl;
        Eigen::Matrix3d state_estimate_rot_mat_inv = state_estimate_rot_mat.inverse();
        //std::cout << "rotation_mat_inv: " << state_estimate_rot_mat_inv << std::endl;
        return state_estimate_rot_mat;
    }

    void TargetFinder::expandedImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        is_max_valid_ = true;
        QuadState state_estimate_image = state_estimate_;
        quad_msgs::QuadStateEstimate state_estimate_image_msg = state_estimate_msg_;


        cv_bridge::CvImagePtr cv_ptr_expanded;

        try
        {
            cv_ptr_expanded = cv_bridge::toCvCopy(msg);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth_expanded_img_ = cv_ptr_expanded->image;
        std::vector<cv::Point> horizon_points = TargetFinder::buildHorizon(state_estimate_image);

        TargetFinder::horizonAnalyze(horizon_points, state_estimate_image_msg);

    }

    std::vector<cv::Point> TargetFinder::buildHorizon(const QuadState state_estimate)
    {
        // Calculate edge points of line
        Eigen::Matrix3d rvec_state_estimate = TargetFinder::tiltCalculator(state_estimate);

        Eigen::Vector3d horizon_center_point_cam = body_cam_rot_ * rvec_state_estimate *horizon_center_point_world_;
        Eigen::Vector3d horizon_left_point_cam = body_cam_rot_ * rvec_state_estimate * horizon_left_point_world_;
        Eigen::Vector3d horizon_right_point_cam = body_cam_rot_ * rvec_state_estimate * horizon_right_point_world_;

        //std::cout << "PEC: " << horizon_center_point_cam << std::endl;
        //std::cout << "PEL: " << horizon_left_point_cam << std::endl;
        //std::cout << "PER: " << horizon_right_point_cam << std::endl;

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

        //std::cout << "pt1: " << pt1 << std::endl;
        //std::cout << "pt2: " << pt2 << std::endl;
        //std::cout << "center: " << horizon_center << std::endl;

        std::vector<cv::Point> horizon_points = TargetFinder::fullLine(pt1, pt2, horizon_center);

        return horizon_points;
    }

    void TargetFinder::horizonAnalyze(std::vector<cv::Point> horizon_points, quad_msgs::QuadStateEstimate state_estimate_image_msg)
    {
        cv::Point min_depth_left_pos;
        cv::Point min_depth_right_pos;
        cv::Point min_depth_ib_pos;
        cv::Point edge_left_pos = horizon_points.at(0);
        cv::Point edge_right_pos = horizon_points.at(1);
        cv::Point center_pos = horizon_points.at(2);
        cv::Point max_depth_pos;
        double max_depth = -1;

        if (is_max_valid_ == true)
        {

            //std::cout << "edge_left_pos: " << edge_left_pos << std::endl;
            //std::cout << "edge_right_pos: " << edge_right_pos << std::endl;
            //std::cout << "center: " << center_pos << std::endl;


            cv::LineIterator it2(depth_expanded_img_, edge_left_pos, edge_right_pos, 8);
            std::vector<cv::Point> free_space_points;
            //free_space_points.erase(free_space_points.begin(),free_space_points.end());

            double depth_edge_left = depth_expanded_img_.at<float>(edge_left_pos);
            double depth_edge_right = depth_expanded_img_.at<float>(edge_right_pos);
            double depth_center = depth_expanded_img_.at<float>(center_pos);

            // Reset values
            double min_depth_left = 6.0;
            double min_depth_right = 6.0;
            double min_depth_ib = 6.0;
            double min_dist_center_max = 180;
            double max_dist_center_ib_min = 180;
            double max_dist_center_left_min = 180;
            double max_dist_center_right_min = 180;
            bool in_free_space = false;

            // Iterate over line
            for(int i = 0; i < it2.count; i++, ++it2)
            {
                float expanded_depth = depth_expanded_img_.at<float>(it2.pos());

                // Creat position point
                int x_pos = it2.pos().x;
                int y_pos = it2.pos().y;
                cv::Point current_pos(x_pos,y_pos);

                // Calc dist to center (e.g. needed change of orientation)
                float current_dist_center = euclideanDist(current_pos, center_pos);

                int side = leftOfSecArg(current_pos, center_pos);

                if (side == -1 && (expanded_depth < min_depth_left || (expanded_depth == min_depth_left && current_dist_center > max_dist_center_left_min)))
                {
                    max_dist_center_left_min = current_dist_center;
                    min_depth_left = expanded_depth;
                    min_depth_left_pos = current_pos;

                } else if (side == +1 && (expanded_depth < min_depth_right || (expanded_depth == min_depth_right && current_dist_center > max_dist_center_right_min)))
                {
                    max_dist_center_right_min = current_dist_center;
                    min_depth_right = expanded_depth;
                    min_depth_right_pos = current_pos;
                }

                if (expanded_depth < 4.5)
                {
                    in_free_space = false;
                }

                if (expanded_depth > max_depth || (expanded_depth == max_depth && current_dist_center < min_dist_center_max) || in_free_space == true)
                {
                    if (in_free_space == false)
                    {
                        free_space_points.erase(free_space_points.begin(), free_space_points.end());
                    }

                    if (expanded_depth >= 4.5)
                    {
                        in_free_space = true;
                        free_space_points.push_back(current_pos);
                    }
                    min_dist_center_max = current_dist_center;
                    max_depth = expanded_depth;
                    max_depth_pos = current_pos;
                }
            }


            // This part allows to find the minimum depth between the center and the max depth pos

            int max_left_of_horizon = leftOfSecArg(max_depth_pos, center_pos); // -1 := true ; 0 := is horizon ; 1 := false

            if (max_left_of_horizon == 0)
            {
                min_depth_ib = max_depth;
                min_depth_ib_pos = max_depth_pos;
            } else
            {
                // Iterate over line
                for(int i = 0; i < it2.count; i++, ++it2)
                {
                    float expanded_depth = depth_expanded_img_.at<float>(it2.pos());

                    // Creat position point
                    int x_pos = it2.pos().x;
                    int y_pos = it2.pos().y;
                    cv::Point pos(x_pos,y_pos);

                    // Calc dist to center (e.g. needed change of orientation)
                    float dist_center = euclideanDist(pos, center_pos);
                    int pos_left_of_horizon = leftOfSecArg(pos, center_pos);
                    int pos_right_of_max = leftOfSecArg(max_depth_pos, pos);

                    if ((pos_right_of_max == -1 && pos_left_of_horizon == -1) || (pos_right_of_max == -1 && pos_left_of_horizon == 0) || (pos_right_of_max == 1 && pos_left_of_horizon == 0) || (pos_right_of_max == 1 && pos_left_of_horizon == 1) || (pos_right_of_max == 0 && pos_left_of_horizon == 0))
                    {
                        if (expanded_depth < min_depth_ib || (expanded_depth == min_depth_ib && dist_center > max_dist_center_ib_min))
                        {
                            max_dist_center_ib_min = dist_center;
                            min_depth_ib = expanded_depth;
                            min_depth_ib_pos = pos;
                        }
                    }
                }
            }

            if (max_depth >= 4.5)
            {
                int length_free_space = free_space_points.size();
                if ((length_free_space >= 90) && depth_expanded_img_.at<float>(center_pos) > 4.5)
                {
                    max_depth_pos = center_pos;
                } else
                {
                    int take_pos = int(depth_edge_right/(depth_edge_left+ depth_edge_right)*length_free_space);
                    max_depth_pos = free_space_points[take_pos];
                }
            }
        } else
        {
            max_depth_pos = cv::Point(-50,-50);
            center_pos = cv::Point(-50,-50);

        }

        min_depth_left_pos = cv::Point(-50,-50);
        min_depth_right_pos = cv::Point(-50,-50);
        //min_depth_ib_pos = cv::Point(-50,-50);

        depth_flight_controller_msgs::Target target;
        target.position = state_estimate_image_msg.position;
        target.yaw = yaw_;
        double target_dist_center = euclideanDistSign(center_pos, max_depth_pos);
        double target_angle = atan(target_dist_center /151.18);

        target.depth = max_depth;
        target.Y = target_dist_center / 151.18 * max_depth;

        std::cout << "max depth pos: " << max_depth_pos << std::endl;
        std::cout << "max depth : " << max_depth << std::endl;
        std::cout << "center pos: " << center_pos << std::endl;
        std::cout << "original_x_pos: " << target.position.x << std::endl;
        std::cout << "original_y_pos: " << target.position.y << std::endl;
        std::cout << "original_yaw: " << target.yaw << std::endl;
        std::cout << "target_dist_center: " << target_dist_center << std::endl;
        std::cout << "targt angle: " << target_angle << std::endl;
        std::cout << "target_Y: " << target.Y << std::endl;

        if (target_angle == 0)
        {
            target.side = 0;
            target.valid = is_max_valid_;
        } else if(max_depth_pos.x < center_pos.x)
        {
            target.side = 1;
            target.valid = is_max_valid_;
        } else
        {
            target.side = -1;
            target.valid = is_max_valid_;
        }

        target_pub_.publish(target);

        depth_flight_controller_msgs::HorizonPoints hps;

        Eigen::Vector3d eig_left(edge_left_pos.x, edge_left_pos.y,0);
        Eigen::Vector3d eig_right(edge_right_pos.x, edge_right_pos.y,0);
        //Eigen::Vector3d eig_right(edge_right_pos.x, edge_right_pos.y,0);
        Eigen::Vector3d eig_center(center_pos.x,center_pos.y,0);
        Eigen::Vector3d eig_max_depth(max_depth_pos.x, max_depth_pos.y,0);
        Eigen::Vector3d eig_min_depth_left(min_depth_left_pos.x, min_depth_left_pos.y,0);
        Eigen::Vector3d eig_min_depth_right(min_depth_right_pos.x, min_depth_right_pos.y,0);
        Eigen::Vector3d eig_min_depth_ib(min_depth_ib_pos.x, min_depth_ib_pos.y,0);

        hps.pt_left = eigenToGeometry(eig_left);
        hps.pt_right = eigenToGeometry(eig_right);
        hps.pt_center = eigenToGeometry(eig_center);
        hps.pt_max_depth = eigenToGeometry(eig_max_depth);
        hps.pt_min_depth_left = eigenToGeometry(eig_min_depth_left);
        hps.pt_min_depth_right = eigenToGeometry(eig_min_depth_right);
        hps.pt_min_depth_ib = eigenToGeometry(eig_min_depth_ib);

        horizon_points_pub_.publish(hps);
    }

    int TargetFinder::leftOfSecArg(cv::Point& p, cv::Point& q)
    {
        if (p.x < q.x)
        {
            return -1;
        } else if (p.x > q.x)
        {
            return 1;
        } else
        {
            return 0;
        }
    }


    float TargetFinder::euclideanDist(cv::Point& p, cv::Point& q)
    {
        cv::Point diff = p - q;
        return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
    }


    float TargetFinder::euclideanDistSign(cv::Point& p, cv::Point& q)
    {
        cv::Point diff = p - q;
        return copysign(cv::sqrt(diff.x*diff.x + diff.y*diff.y),(p.x-q.x));
    }


    double TargetFinder::Slope(int x0, int y0, int x1, int y1)
    {
        return (double)(y1-y0)/(x1-x0);
    }


    std::vector<cv::Point> TargetFinder::fullLine(cv::Point a, cv::Point b, cv::Point center_pos)
    {
        double slope = Slope(a.x, a.y, b.x, b.y);

        cv::Point edge_left_pos = cv::Point(0,0);
        cv::Point edge_right_pos = cv::Point(depth_expanded_img_.cols-1,depth_expanded_img_.rows-1);

        edge_left_pos.y = -(a.x - edge_left_pos.x) * slope + a.y;
        edge_right_pos.y = -(b.x - edge_right_pos.x) * slope + b.y;

        cv::LineIterator it(depth_expanded_img_, edge_left_pos, edge_right_pos, 8);

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

        if (it.count < 150 || center_pos.x < edge_left_pos.x || center_pos.x > edge_right_pos.x )
        {

            is_max_valid_ = false;
        }

        //std::cout << "pt1 update: " << edge_left_pos << std::endl;
        //std::cout << "pt2 update: " << edge_right_pos << std::endl;
        //std::cout << "center update: " << center_pos << std::endl;

        std::vector<cv::Point> horizon_points;
        horizon_points.push_back(edge_left_pos);
        horizon_points.push_back(edge_right_pos);
        horizon_points.push_back(center_pos);

        return horizon_points;
    }

    void TargetFinder::stateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr &msg)
    {
        state_estimate_ = QuadState(*msg);
        state_estimate_msg_ = *msg;

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_finder");

    depth_flight_controller::TargetFinder hp;
    ros::spin();

    return 0;
}
