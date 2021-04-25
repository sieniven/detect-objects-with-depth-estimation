// Garuda Robotics CoPilot Stereo Camera Image Processing Node
// Author: Niven Sie, sieniven@gmail.com
// 
// This code contains the ROS2 Node class StereoImageProcessor to process the raw images
// obtained from topic name "sync_stereo/image_raw" on ROS2 CoPilot software.
// 
// The Stereo Image Processor node will obtain disparty map and obtain segmented image 
// using watershed algorithm. The image processor will also implement a blob detector to 
// detector obstacles for depth estimation of the obstacles in the frame. We will also 
// implement bounding boxes into these obstacles and obtain obstacle detection frame.
// 
// The processor node will also publish the disparity frames, segmented image frames, and 
// blob detected obtstacles image frames.
// 
// Topic names:
// 1. raw left image: copilot_daa/raw_left_image
// 2. disparity map: copilot_daa/disparity_filtered_image
// 3. disparity thresholded image: copilot_daa/disparity_threshold_image
// 4. disparity canny image: copilot_daa/disparity_canny_image
// 5. disparity gradient image: copilot_daa/disparity_grad_image
// 6. blob image: copilot_daa/object_detect_image

#include "copilot_daa_perception/stereo_image_processor.hpp"
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <memory>
#include <functional>

using copilotdaa::StereoImageProcessor;

StereoImageProcessor::StereoImageProcessor()
: Node("StereoImageProcessorNode")
{
    RCLCPP_INFO(this->get_logger(), "Initializing CoPilot Stereo Image Processor Node...");

    // declare ROS2 parameters
    //  distortion model
    this->declare_parameter("distortion_model", "plumb bob");
    
    // camera dimensions
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    
    // distortion matrix
    this->declare_parameter("left_distortion",
        std::vector<double>{0.10914496434736994, -0.5164917925641962, 
        0.0004939784496380255, 0.0017035646973372872, 0.750225516796563});

    this->declare_parameter("right_distortion",
        std::vector<double>{-0.023398645389188576, 0.06054885267837794, 
        -0.0023501644034139837, 0.0028599789382829336, -0.035709302626510155});

    // intrinsic matrix
    this->declare_parameter("left_intrinsic",
        std::vector<double>{386.65423583984375, 0.0, 361.89831246459653, 
        0.0, 383.94366455078125, 251.00344623337514, 0.0, 0.0, 1.0});

    this->declare_parameter("right_intrinsic",
        std::vector<double>{389.8464837271582, 0.0, 359.2387742197719,
        0.0, 388.6429163725939, 253.04058043501843, 0.0, 0.0, 1.0 });

    // rectification matrix
    this->declare_parameter("left_rectification",
        std::vector<double>{0.9981136575637966, -0.0585274476271129, 0.01853818920163794, 
        0.05841731536006565, 0.9982715544668554, 0.006428126356162717, 
        -0.018882368779970547, -0.005333049463835162, 0.9998074888360626});

    this->declare_parameter("right_rectification",
        std::vector<double>{0.9981946420064934, -0.05697027052913437, 0.019022222408688304,
        0.05708124883303519, 0.9983552410861551, -0.0053426236500790335,
        -0.018686564724140108, 0.006418790512433739, 0.9998047866594598});
    
    // new camera matrix
    this->declare_parameter("new_left_mat",
        std::vector<double>{505.1258239746094, 0.0, 335.2074720647888,
        0.0, 502.7547607421875, 219.76922062501035, 0.0, 0.0, 1.0});
    
    this->declare_parameter("new_right_mat",
        std::vector<double>{386.65423583984375, 0.0, 361.89831246459653,
        0.0, 383.94366455078125, 251.00344623337514, 0.0, 0.0, 1.0});

    // projection matrix
    this->declare_parameter("left_projection",
        std::vector<double>{382.4308123701471, 0.0, 320.1807384490967, 0.0, 0.0,
        382.4308123701471, 247.1542568206787, 0.0, 0.0, 0.0, 1.0, 0.0});

    this->declare_parameter("right_projection",
        std::vector<double>{382.4308123701471, 0.0, 320.1807384490967, -935.2555593136844,
        0.0, 382.4308123701471, 247.1542568206787, 0.0, 0.0, 0.0, 1.0, 0.0});

    this->declare_parameter("disparity_to_depth",
        std::vector<double>{1.0, 0.0, 0.0, -320.1807384490967, 0.0, 1.0, 0.0, -247.1542568206787,
        0.0, 0.0, 0.0, 382.4308123701471, 0.0, 0.0, 0.4089051474345526, -0.0});
            
    // wsize default 3; 5; 7 for SGBM reduced size image;
    // 15 for SGBM full size image (1300px and above); 5 Works nicely
    window_size = 15;
    minDisparity = -1;
    numDisparities = 80;
    blockSize = window_size;
    P1 = 8 * window_size * window_size;
    P2 = 32 * window_size * window_size;
    disp12MaxDiff = 12;
    preFilterCap = 63;
    uniquenessRatio = 10;
    speckleWindowSize = 50;
    speckleRange = 4;

    // set parameters for WLSFilter
    lambda = 80000;
    sigma = 1.3;
    visual_multiplier = 6;
    
    // create filtered disparity image publisher with topic name "copilot_daa/disparity_info"
    disp_pub = this->create_publisher<copilot_daa_msg::msg::DispInfo> (
        "copilot_daa/disparity_info",
        10);
    
    // create filtered disparity image publisher with topic name "copilot_daa/disparity_filtered_image"
    disp_filtered_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/disparity_filtered_image",
        10);

    // create disparity thresholded image publisher with topic name "copilot_daa/disparity_threshold_image"
    thresh_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/disparity_threshold_image",
        10);
    
    // create disparity canny image publisher with topic name "copilot_daa/disparity_canny_image"
    canny_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/disparity_canny_image",
        10);
    
    // create disparity gradient image publisher with topic name "copilot_daa/disparity_grad_image"
    grad_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/disparity_grad_image",
        10);
    
    // create blob image publisher with topic name "copilot_daa/object_detect_image"
    blob_img_pub = this->create_publisher<sensor_msgs::msg::Image> (
        "copilot_daa/object_detect_image",
        10);

    RCLCPP_INFO(this->get_logger(), "Successfully launched CoPilot Stereo Image Processor Node!");

    get_camera_info();
    process_img_callback();
}

// function to create subscription to raw stereo image and process the stereo images to get
// depth and disparity maps, for depth estimation capabilities of surroundings
void StereoImageProcessor::process_img_callback()
{
    // set qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    // cv::namedWindow("Filtered Normalized");
    // cv::namedWindow("Filtered Color");
    // cv::namedWindow("Thresholded");
    // cv::namedWindow("Canny Edge Detection");
    // cv::namedWindow("Grad Thresh");
    // cv::namedWindow("Object Detection");
    
    
    // create raw image subscriber that subscribes to topic name "copilot_daa/raw_image"
    this->raw_img_sub = this->create_subscription<sensor_msgs::msg::Image> (
        "copilot_daa/raw_image",
        qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            // decode message and split into left and right image frames
            raw_image = cv::Mat(
                msg->height, msg->width, encoding2mat_type(msg->encoding),
                const_cast<unsigned char *>(msg->data.data()), msg->step);
            
            if (msg->encoding == "rgb8") {
                cv::cvtColor(raw_image, raw_image, cv::COLOR_RGB2BGR);
            }

            image_left  = raw_image(cv::Range(0, 480), cv::Range(0, 640));
            image_right = raw_image(cv::Range(0, 480), cv::Range(640, 1280));
            frame_id = msg->header.frame_id;
            
            // resize to reduce computational cost
            // cv::resize(image_left, image_left, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
            // cv::resize(image_right, image_right, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);

            // convert to grayscale
            cv::cvtColor(image_left, blur_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(image_right, blur_right, cv::COLOR_BGR2GRAY);

            // apply GaussianBlur on images
            cv::GaussianBlur(blur_left, blur_left, cv::Size(3, 3), 0);
            cv::GaussianBlur(blur_right, blur_right, cv::Size(3, 3), 0);

            // get disparity image and publish on CoPilot ROS2 topics
            get_disparity_image();
            publish_images();
            
            cv::waitKey(1);
        }
    );
}

// function to get camera info from ROS2 parameters. when launching stereo image processor 
// node, ensure that camera info cameras in location: 
// ..../copilot_daa/copilot_daa_calibration/data/intrinsic/camera_info.yaml
void StereoImageProcessor::get_camera_info()
{
    left_dist_param = this->get_parameter("left_distortion");
    right_dist_param = this->get_parameter("right_distortion");
    left_intrinsic_param = this->get_parameter("left_intrinsic");
    right_intrinsic_param =this->get_parameter("right_intrinsic");
    left_rect_param = this->get_parameter("left_rectification");
    right_rect_param = this->get_parameter("right_rectification");
    new_left_mat_param = this->get_parameter("new_left_mat");
    new_right_mat_param = this->get_parameter("new_right_mat");
    left_proj_param = this->get_parameter("left_projection");
    right_proj_param = this->get_parameter("right_projection");
    disparity_to_depth_param = this->get_parameter("disparity_to_depth");

    // variable declaration
    std::vector<double> unpacker;

    // get left camera distortion matrix
    unpacker = left_dist_param.as_double_array();
    double temp[5];
    for (int i=0; i < 5; i++) {
        temp[i] = unpacker[i];
    };
    left_dist = cv::Mat(1, 5, CV_64FC1, temp).clone();

    // get right camera distortion matrix
    unpacker = right_dist_param.as_double_array();
    for (int i=0; i < 5; i++) {
        temp[i] = unpacker[i];
    };
    right_dist = cv::Mat(1, 5, CV_64FC1, temp).clone();

    // variable declaration
    double two_d_temp[9];

    // get left camera intrinsic matrix
    unpacker = left_intrinsic_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    left_intrinsic = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();

    // get right camera intrinsic matrix
    unpacker = right_intrinsic_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    right_intrinsic = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();
    
    // get left camera rectification matrix
    unpacker = left_rect_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    left_rect = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();

    // get right camera rectification matrix
    unpacker = right_rect_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    right_rect = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();
    
    // get left new camera matrix
    unpacker = new_left_mat_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    new_left_mat = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();

    // get right new camera matrix
    unpacker = new_right_mat_param.as_double_array();
    for (int i=0; i<9; i++) {
        two_d_temp[i] = unpacker[i];
    }
    new_right_mat = cv::Mat(3, 3, CV_64FC1, two_d_temp).clone();

    // variable declaration
    double proj_temp[12];

    // get left camera projection matrix
    unpacker = left_proj_param.as_double_array();
    for (int i=0; i<12; i++) {
        proj_temp[i] = unpacker[i];
    }
    left_proj = cv::Mat(3, 4, CV_64FC1, proj_temp).clone();

    // get right camera projection matrix
    unpacker = right_proj_param.as_double_array();
    for (int i=0; i<12; i++) {
        proj_temp[i] = unpacker[i];
    }
    right_proj = cv::Mat(3, 4, CV_64FC1, proj_temp).clone();

    // variable declaration
    double dtd_temp[16];

    // get right camera projection matrix
    unpacker = disparity_to_depth_param.as_double_array();
    for (int i=0; i<16; i++) {
        dtd_temp[i] = unpacker[i];
    }
    disparity_to_depth = cv::Mat(4, 4, CV_64FC1, dtd_temp).clone();
}

// function to obtain disparity and depth maps
void StereoImageProcessor::get_disparity_image()
{
    // undistort and rectify left and right images
    cv::initUndistortRectifyMap(
        left_intrinsic, left_dist, left_rect, new_left_mat,
        image_left.size(), CV_32FC1, left_map1, left_map2);

    cv::initUndistortRectifyMap(
        right_intrinsic, right_dist, right_rect, new_right_mat,
        image_right.size(), CV_32FC1, right_map1, right_map2);

    // initialize rect frame variables with frame size and type
    rect_left_frame = cv::Mat(blur_left.size(), blur_left.type());
    rect_right_frame = cv::Mat(blur_right.size(), blur_right.type());

    // remap to get rectified images
    cv::remap(
        blur_left, rect_left_frame, left_map1, left_map2, 
        cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(
        blur_right, rect_right_frame, right_map1, right_map2, 
        cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // create StereoSGBM object for left matcher
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize, P1, P2, disp12MaxDiff,
        preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
    left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    // create right matcher
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    // create wls filter, and set parameters
    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);

    // compute left and right matcher
    left_matcher->compute(rect_left_frame, rect_right_frame, disp_left);
    right_matcher->compute(rect_right_frame, rect_left_frame, disp_right);

    // filtering and visualization
    wls_filter->filter(disp_left, rect_left_frame, filtered_disp, disp_right);

    cv::normalize(filtered_disp, filtered_disp_norm, 255, 0, cv::NORM_MINMAX);
    filtered_disp_norm.convertTo(filtered_disp_norm, CV_8UC1);

    cv::minMaxIdx(filtered_disp_norm, &min, &max);
    cv::convertScaleAbs(filtered_disp_norm, filtered_disp_norm, int(255/(max-min)));

    // cv::imshow("Filtered Normalized", filtered_disp_norm);

    cv::applyColorMap(filtered_disp_norm, filtered_disp_color, cv::COLORMAP_BONE);
    // cv::imshow("Filtered Color", filtered_disp_color);

    // thresh_img = get_adaptive_thresholding_edge_detection(filtered_disp_norm);
    // // cv::imshow("Thresholded", thresh_img);

    // canny_img = get_canny_edge_detection(filtered_disp_norm);
    // // cv::imshow("Canny Edge Detection", canny_img);

    // grad_img = gradient_threshold(filtered_disp_norm);
    // // cv::imshow("Grad Thresh", grad_img);

    // blob_img = get_blob_detection(filtered_disp_norm);
    // // cv::imshow("Object Detection", blob_img);
}

// function to publish imagesvb 
void StereoImageProcessor::publish_images()
{
    rclcpp::Time timestamp = this->now();
    std_msgs::msg::Header header;
    std::string encoding;

    header.stamp = timestamp;
    header.frame_id = frame_id;

    // publish disparity info
    copilot_daa_msg::msg::DispInfo disp_info;
    std_msgs::msg::Int16MultiArray disp_msg;
    std::vector<int16_t> disp_array;
    std_msgs::msg::MultiArrayDimension dim0, dim1, dim2;

    encoding = mat_type2encoding(image_left.type());
    sensor_msgs::msg::Image::SharedPtr raw_msg = cv_bridge::CvImage(
        header, encoding, image_left).toImageMsg();
    
    disp_array.assign(filtered_disp.begin<int16_t>(), filtered_disp.end<int16_t>());
    
    dim0.label = "height";
    dim0.size   = 480;
    dim0.stride = 307200;  // (1x640x480)
    dim1.label  = "width";
    dim1.size   = 640;
    dim1.stride = 640;
    dim2.label  = "channel";
    dim2.size   = 1;
    dim2.stride = 1;
    
    std::vector<std_msgs::msg::MultiArrayDimension> dim = {dim0, dim1, dim2};
    disp_msg.layout.dim = dim;
    disp_msg.layout.data_offset = 0;
    disp_msg.data = disp_array;

    disp_info.image = *raw_msg;
    disp_info.disparity = disp_msg;
    disp_pub->publish(disp_info);

    // publish filtered disparity frame
    encoding = mat_type2encoding(filtered_disp_color.type());
    sensor_msgs::msg::Image::SharedPtr filtered_msg = cv_bridge::CvImage(
        header, encoding, filtered_disp_color).toImageMsg();
    disp_filtered_img_pub->publish(*filtered_msg);

    // // publish adaptive threshold frame
    // encoding = mat_type2encoding(thresh_img.type());
    // sensor_msgs::msg::Image::SharedPtr thresh_msg = cv_bridge::CvImage(
    //     header, encoding, thresh_img).toImageMsg();
    // thresh_img_pub->publish(*thresh_msg);
    
    // // publish canny edge detection frame
    // encoding = mat_type2encoding(canny_img.type());
    // sensor_msgs::msg::Image::SharedPtr canny_msg = cv_bridge::CvImage(
    //     header, encoding, canny_img).toImageMsg();
    // canny_img_pub->publish(*canny_msg);

    // // publish gradient threshold frame
    // encoding = mat_type2encoding(grad_img.type());
    // sensor_msgs::msg::Image::SharedPtr grad_msg = cv_bridge::CvImage(
    //     header, encoding, grad_img).toImageMsg();
    // grad_img_pub->publish(*grad_msg);

    // // // publish blob detection frame
    // encoding = mat_type2encoding(blob_img.type());
    // sensor_msgs::msg::Image::SharedPtr blob_msg = cv_bridge::CvImage(
    //     header, encoding, blob_img).toImageMsg();
    // blob_img_pub->publish(*blob_msg);
}

std::string StereoImageProcessor::mat_type2encoding(int mat_type)
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

int StereoImageProcessor::encoding2mat_type(const std::string & encoding)
{
    if (encoding == "mono8") {
        return CV_8UC1;
    } else if (encoding == "bgr8") {
        return CV_8UC3;
    } else if (encoding == "mono16") {
        return CV_16SC1;
    } else if (encoding == "rgba8") {
        return CV_8UC4;
    } else if (encoding == "bgra8") {
        return CV_8UC4;
    } else if (encoding == "32FC1") {
        return CV_32FC1;
    } else if (encoding == "rgb8") {
        return CV_8UC3;
    } else {
        throw std::runtime_error("Unsupported encoding type");
    }
}

// this function thresholds the img finds contours of the thresholded image
cv::Mat StereoImageProcessor::get_adaptive_thresholding_edge_detection(cv::Mat img)
{
    cv::Mat edge;
    std::vector<std::vector<cv::Point>> contours;

    cv::adaptiveThreshold(img, edge, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3, 2);
    cv::findContours(edge, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    cv::drawContours(img, contours, -1, cv::Scalar(255, 0, 0), -1);
    
    return img.clone();
}

// this function gets edge detected frames using Canny Edge Detection algorithm
cv::Mat StereoImageProcessor::get_canny_edge_detection(cv::Mat img)
{
    cv::Mat edge;
    std::vector<std::vector<cv::Point>> contours;

    cv::Canny(img, edge, 100, 200);
    cv::findContours(edge, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    cv::drawContours(img, contours, -1, cv::Scalar(255, 0, 0), -1);

    return img.clone();
}

// function for gradient thresholding
cv::Mat StereoImageProcessor::gradient_threshold(cv::Mat img)
{
    cv::Mat sobelx, sobely;
    cv::Sobel(img, sobelx, CV_32FC1, 1, 0, 3);
    cv::Sobel(img, sobely, CV_32FC1, 0, 1, 3);

    // calculate magnitude of sobel gradients in both x and y directions, using
    // magnitude = sqrt(x^2 + y^2)
    cv::Mat sobelx_sq = sobelx.mul(sobelx);
    cv::Mat sobely_sq = sobely.mul(sobely);
    cv::Mat mag_sobel = sobelx_sq + sobely_sq;
    cv::sqrt(mag_sobel, mag_sobel);
    cv::normalize(mag_sobel, mag_sobel, 255, 0, cv::NORM_MINMAX);
    mag_sobel.convertTo(mag_sobel, CV_8UC1);

    // apply thresholding
    cv::Mat thresh;
    cv::adaptiveThreshold(mag_sobel, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3, 2);

    return thresh.clone();
}

// function to apply blob detection
cv::Mat StereoImageProcessor::get_blob_detection(cv::Mat img)
{
    // perform morphological transformation
    // cv::morphologyEx(img, img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    cv::SimpleBlobDetector::Params params;

    params.minThreshold = 10;
    params.maxThreshold = 200;

    params.filterByArea = true;
    params.minArea = 100;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    cv::Mat img_with_keypoints;
    cv::drawKeypoints(img, keypoints, img_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    return img_with_keypoints.clone();
}