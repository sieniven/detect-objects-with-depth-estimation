// Garuda Robotics CoPilot Stereo Camera Image Processing Node
// Author: Niven Sie, sieniven@gmail.com
// 
// This code runs the ImagePublisherNode and UVCDriver.

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "copilot_daa_perception/uvc_driver.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>

int main(int argc, char * argv[])
{
    printf("Launching CoPilot DAA Perception...\n");

    rclcpp::init(argc, argv);
    
    // initialize UVCDriver
    int video_device_id = 0;
    rclcpp::executors::SingleThreadedExecutor executor;
    auto uvc_driver_node = std::make_shared<copilotdaa::UVCDriver>(video_device_id);

    uvc_driver_node->start_record();
    uvc_driver_node->stop_record();
    
    printf("CoPilot DAA Perception interrupted. Shutting down...\n");
    rclcpp::shutdown();
    return 0;
}