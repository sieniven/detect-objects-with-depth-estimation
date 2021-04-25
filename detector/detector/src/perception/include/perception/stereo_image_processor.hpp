// Garuda Robotics CoPilot Stereo Camera Image Processing Node
// Author: Niven Sie, sieniven@gmail.com

#ifndef COPILOT_DAA_STEREO_IMAGE_PROCESSOR_HPP_
#define COPILOT_DAA_STEREO_IMAGE_PROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <copilot_daa_msg/msg/disp_info.hpp>

#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>

namespace copilotdaa
{
class StereoImageProcessor : public rclcpp::Node {
    public:
        StereoImageProcessor();
        
        rclcpp::Publisher<copilot_daa_msg::msg::DispInfo>::SharedPtr disp_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_filtered_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thresh_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr canny_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grad_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blob_img_pub;
        
        // parameters for StereoSGBM
        int window_size, minDisparity, numDisparities, blockSize, P1, P2, disp12MaxDiff,
                preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange;

        // parameters for WLSFilter
        double lambda, sigma, visual_multiplier;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub;

        cv::Mat frame, raw_image, image_left, image_right, blur_left, blur_right;

        rclcpp::Parameter left_dist_param, right_dist_param;
        rclcpp::Parameter left_intrinsic_param, right_intrinsic_param;
        rclcpp::Parameter left_rect_param, right_rect_param;
        rclcpp::Parameter new_left_mat_param, new_right_mat_param;
        rclcpp::Parameter left_proj_param, right_proj_param;
        rclcpp::Parameter disparity_to_depth_param;

        std::string frame_id;
        
        cv::Mat left_dist, right_dist;
        cv::Mat left_intrinsic, right_intrinsic;
        cv::Mat left_rect, right_rect;
        cv::Mat new_left_mat, new_right_mat;
        cv::Mat left_proj, right_proj;
        cv::Mat disparity_to_depth;

        void get_camera_info();
        void process_img_callback();
        void get_disparity_image();
        void publish_images();

        std::string mat_type2encoding(int mat_type);
        int encoding2mat_type(const std::string & encoding);

        cv::Mat get_adaptive_thresholding_edge_detection(cv::Mat img);
        cv::Mat get_canny_edge_detection(cv::Mat img);
        cv::Mat gradient_threshold(cv::Mat img);
        cv::Mat get_blob_detection(cv::Mat img);

        // declare variables for output maps 1 and 2 for cv::initUndistortRectifyMap(),
        // used to obtain the undistortion and rectification transformation map
        cv::Mat left_map1, left_map2;
        cv::Mat right_map1, right_map2;

        // declare variables for rectified left and right frames
        cv::Mat rect_left_frame, rect_right_frame;

        // declare variables for gray left and right frames
        cv::Mat gray_left, gray_right;

        // declare variables for computing disparity maps
        cv::Mat disp_left, disp_right;

        // declare variables for computing filtered frames
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        cv::Mat filtered_disp, filtered_disp_norm, filtered_disp_color;
        cv::Mat thresh_img, canny_img, grad_img, blob_img;

        double min, max;

};
}

#endif // COPILOT_DAA_STEREO_IMAGE_PROCESSOR_HPP_