// Garuda Robotics CoPilot stereo Camera Video Capture
// Author: Niven Sie, sieniven@gmail.com
// 
// This code contains the UVCDriver node to run our stereo cameras and publish the raw
// images into topic name "copilot_daa/raw_image" on ROS2 CoPilot software.

#include "copilot_daa_perception/uvc_driver.hpp"
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <memory>
#include <functional>

using copilotdaa::UVCDriver;

UVCDriver::UVCDriver(const int & video_device_id)
: Node("UVCDriverNode")
{
    RCLCPP_INFO(this->get_logger(), "Initialize UVCDriver");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    // set video capture. use if deployed on local machine
    // cap = cv::VideoCapture(video_device_id);
    // use second cap if deployed on Jetson Nano
    cap = cv::VideoCapture(video_device_id, cv::CAP_V4L2);
    
    // create raw image publisher with topic name "copilot_daa/raw_image"
    stereo_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/raw_image", 
        10);
    
    if (!cap.isOpened()) {
        std::cout << "Error: Cannot run stereo cameras! Please check!" << std::endl;
    }
    else {
        std::cout << "Stereo camera opened successful!" << std::endl;
    }
    // set stereo camera width and height dimensions (1280 x 480, width x height)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // set FPS to be 25 FPS
    cap.set(cv::CAP_PROP_FPS, 25);
}

// function to start video capturing and recording on Garuda CoPilot
void UVCDriver::start_record()
{
    frame_id = 1;
    // cv::namedWindow("Raw Feed");

    while (1) {
        cap >> frame;
        if (frame.empty()) {
            std::cout << "Error: Video camera is disconnected!" << std::endl;
            break;
        }

        // if escape key is pressed
        if (cv::waitKey(10) == 27) {
            std::cout << "Error: Esc key pressed. Video camera is disconnected!" << std::endl;
            break;
        }
        // show video feed on opencv GUI
        // cv::imshow("Raw Feed", frame);

        // publish raw image frames as ROS2 messages on CoPilot topic
        publish_image();

        // spin the UVCDriver node once
        rclcpp::spin_some(node_handle_);

        frame_id++;
    }
}

// function to stop video capturing and recording on Garuda CoPilot
void UVCDriver::stop_record()
{
    std::cout << "Stop capturing completed!" << std::endl;
    cap.release();

    cv::destroyAllWindows();
}

// function to publish image
void UVCDriver::publish_image()
{
    frame_id_str = std::to_string(frame_id);
    rclcpp::Time timestamp = this->now();
    std_msgs::msg::Header header;
    std::string encoding;
    
    header.stamp = timestamp;
    header.frame_id = frame_id_str;

    // publish raw image frame
    encoding = mat_type2encoding(frame.type());
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
        header, encoding, frame).toImageMsg();
    stereo_img_pub->publish(*msg);
}

std::string UVCDriver::mat_type2encoding(int mat_type)
{
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

