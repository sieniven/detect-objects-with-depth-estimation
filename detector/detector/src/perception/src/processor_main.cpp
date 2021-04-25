// Garuda Robotics CoPilot Stereo Camera Image Processing Node
// Author: Niven Sie, sieniven@gmail.com
// 
// This code runs and spins the StereoImageProcessor node.

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "copilot_daa_perception/stereo_image_processor.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>

int main(int argc, char * argv[])
{
    printf("Launching CoPilot DAA Processor...\n");

    rclcpp::init(argc, argv);
    
    // initialize stereo image processor node
    auto stereo_img_proc_node = std::make_shared<copilotdaa::StereoImageProcessor>();

    rclcpp::spin(stereo_img_proc_node);
    
    printf("CoPilot DAA Processor shutting down...\n");
    rclcpp::shutdown();
    cv::destroyAllWindows();

    return 0;
}