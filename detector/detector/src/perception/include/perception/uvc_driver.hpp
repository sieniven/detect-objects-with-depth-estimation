// Garuda Robotics CoPilot Stereo Camera Video Capture
// Author: Niven Sie, sieniven@gmail.com

#ifndef COPILOT_DAA_UVC_DRIVER_HPP_
#define COPILOT_DAA_UVC_DRIVER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <array>

namespace copilotdaa
{
class UVCDriver : public rclcpp::Node {
    public:
        UVCDriver(const int & video_device_id);
        void start_record();
        void stop_record();

        rclcpp::Node::SharedPtr node_handle_;
        
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_img_pub;

        int frame_id;
        std::string frame_id_str;
        
        cv::VideoCapture cap;
        cv::Mat frame;

        void publish_image();
        std::string mat_type2encoding(int mat_type);
};
}

#endif  // COPILOT_DAA_UVC_DRIVER_HPP_