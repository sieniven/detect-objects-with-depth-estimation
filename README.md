# Detect Objects with Depth Estimation

Hello! This is a project I did on developing a software for detection and depth estimation of detected objects using stereo cameras. This software is built on ROS2 architecture that can be incorporated into robotics systems running in ROS framework as their perception capabilities. In this project, I have implemented YOLO-v4 Tiny model as our object detection and classification model. It has been tested to be able to run on the Jetson Nano at ~5 FPS. Below lists some of the capabilities of the software:

* Continuous publishing of live stereo-camera video feeds into ROS2 architecture (DDS-RTPS)
* Stereo image processor to obtain disparity maps and depth maps, for depth estimation
* Image segmentation, image processing techniques and blob detector on disparity frames
* Obstacle detection capabilities using Scaled-YOLOv4 (tiny) model
* Implementation of depth estimation capabilities with obstacles detected

Author and owner: Niven Sie (sieniven@gmail.com)

## Table of Contents

   1. [Getting started](#markdown-header-1-getting-started)
   2. [Setup](#markdown-header-2-setup)
   3. [Installing](#markdown-header-3-installing)
      1. [Installing on computer](#markdown-header-31-installing-on-computer)
      2. [Installing using Docker environment](#markdown-header-32-installing-using-docker-environment)
   4. [Run](#markdown-header-4-run)
      1. [Run software locally](#markdown-header-41-run-software-locally)
      2. [Run software on Docker](#markdown-header-42-run-software-docker)
   5. [Acknowledgments](#markdown-header-7-acknowledgments)


## 1. Getting started

Welcome to my project! This software is verified to be working on a Jetson Nano. For the current software version, it is able to run in a Dockerized environment or locally. For the full setup, installation and deployment guide, refer to the steps below in this markdown file.

Please note that this repository should be stored in ***/home/$USER***

## 2. Setup

Pull this repository

``` bash
git clone https://github.com/sieniven/detect-objects-with-depth-estimation-ros.git
```

## 3. Installing

### 3.1 Installing on computer

The following step-by-step instructions will guide you on the installation process on your computer.

1. Install ROS2 Eloquent. Refer to: https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/
	* NOTE: please skip the environment setup options in the installation process.

2. Install ROS2 launch packages
	```
	sudo apt-get update && sudo apt-get install -y ros-eloquent-launch*
	```

3. Install required dependencies. Check Tensorflow website for more information on this.
	* CUDA: 11.0
	* CUDnn: 8.0

5. Install Tensorflow-GPU (2.4.0).
	* NOTE: Ensure that all dependencies are installed as well.

6. Install Boost libraries
	``` bash
	sudo apt-get update && sudo apt-get install libboost-all-dev
	```

7. Install required dependencies from **requirements.txt** file
	```
	cd detect-obstacles-ros/detector/bin
	python3 -m pip install -r requirements.txt
	```

8. Install OpenCV dependencies
	``` bash
	sudo sudo apt-get purge *libopencv*
	sudo apt-get install build-essential cmake git unzip pkg-config
	sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
	sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
	sudo apt-get install libgtk2.0-dev libcanberra-gtk*
	sudo apt-get install python3-dev python3-numpy python3-pip
	sudo apt-get install libxvidcore-dev libx264-dev libgtk-3-dev
	sudo apt-get install libtbb2 libtbb-dev libdc1394-22-dev
	sudo apt-get install libv4l-dev v4l-utils
	sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
	sudo apt-get install libavresample-dev libvorbis-dev libxine2-dev
	sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
	sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
	sudo apt-get install libopenblas-dev libatlas-base-dev libblas-dev
	sudo apt-get install liblapack-dev libeigen3-dev gfortran
	sudo apt-get install libhdf5-dev protobuf-compiler
	sudo apt-get install libprotobuf-dev libgoogle-glog-dev libgflags-dev
	```

9. Install OpenCV and OpenCV_Contrib libraries
	* Install version 4.5.2, refer to installation guide here: https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html.
	* Install OpenCV and OpenCV_Contrib

	``` bash
	wget https://github.com/opencv/opencv/archive/4.5.2.zip -O opencv-4.5.2.zip
	wget https://github.com/opencv/opencv_contrib/archive/4.5.2.zip -O opencv-contrib-4.5.2.zip
	unzip opencv-4.5.2.zip
	unzip opencv-contrib-4.5.2.zip
	mv opencv-4.5.2 opencv
	mv opencv_contrib-4.5.2 opencv_contrib
	mkdir -p build && cd build
	cmake -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv
	make -j3
	sudo make install
	```

10. All dependencies have been configured and installed. Build the software:
	``` bash
	cd detect-obstacles-ros/detector/detector
	./build_detector.sh
	```

### 3.2 Installing using Docker environment

The following step-by-step instructions will guide you on the installation process using Docker. 

1. Install Docker Engine. Refer to https://docs.docker.com/engine/install/ to install Docker Engine.

2. build detector docker image

	``` bash
	cd detect_obstacles_ros/dockerfiles
	docker-compose -f bringup-launch.yaml build
	```

## 4. Run

### 4.1 Run software locally

1. Ensure that the stereo cameras are configured to the correct video port in the perception capability. 
	* Go into ***"/detect-obstacles-ros/detector/detector/src/perception/src/perception_main.cpp"***. If the video port index is 0, then input the correct video index in the script: **"int video_device_id = 0"**.

2. Ensure OpenCV VideoCapture object is configured correctly if used on ARM processor.
	* On Jetson Nano: Go into ***"/detect-obstacles-ros/detector/detector/src/perception/src/uvc_driver.cpp"***, and ensure **"cap = cv::VideoCapture(video_device_id, cv::CAP_V4L2)"** is used (uncommented).
	* On local machine: Go into ***"/detect-obstacles-ros/detector/detector/src/perception/src/uvc_driver.cpp"***, and ensure **"cap = cv::VideoCapture(video_device_id)"** is used (uncommented).

3. Before running the software, if software is deployed for testing and the OpenCV window is to be used for visualization, then we need to ensure that **"cv::imshow" (in C++ scripts)** and **"cv2.imshow" (in python scripts)** are uncommented. For actual deployment, we should uncomment these lines instead. Thus, for this we have to look into 3 scripts and comment/uncomment all of these lines. The scripts are specifically in:
	* ***"/detect-obstacles-ros/detector/detector/src/perception/src/uvc_driver.cpp"***
	* ***"/detect-obstacles-ros/detector/detector/src/perception/src/stereo_image_processor.cpp"***
	* ***"/detect-obstacles-ros/detector/detector/src/detection/detection/detector_node.py"***

4. Before running the software, ensure that the **CORRECT** model interface is being used. For the detection capabilities we are using YOLOv4 Tiny, we can deploy the convolutional neural network using TensorRT, TFLite, and Tensorflow-GPU. For deployment on the Jetson Nano, using TensorRT is our **PRIMARY** interface for deployment. To ensure the correct model is selected:
	* To launch model using TensorRT, go inside the ***"/detect-obstacles-ros/detector/detector/src/detection/detection/detector_node.py"*** script. Ensure that model path for tensorRT framework is uncommented, and comment out Tensorflow-GPU model path. These codes can be found in the **init function** of class ImageDetectorNode.
	* To launch model using Tensorflow-GPU, go inside the ***"/detect-obstacles-ros/detector/detector/src/detection/detection/detector_node.py"*** script. Ensure that model path for tensorflow-gpu framework is uncommented, and comment out TensorRT model path. These codes can be found in the **init function** of class ImageDetectorNode.
	* To launch the model using Tensorflow-Lite, go inside the ***"/detect-obstacles-ros/detector/detector/src/detection/detection/detector_main.py"*** script. Ensure that the detector_node is initialized with **ImageDetectorNode(flag="tflite")**, and comment out **ImageDetectorNode()** line.
		
5. Before running software, after all configurations are set, we will need to rebuild the software if any changes are made. Please refer to **Step 10 in Section 3.1**. 

6. Finally, to launch the software:
	``` bash
	cd /detect-obstacles-ros/detector/bin
	./bringup.sh
	```

7. To launch detector processes manually, there are 3 main processes to run.

	* Run perception. The main process that publishes raw camera frames into the ROS2 DDS-RTPS, and 
		``` bash
		cd /detect-obstacles-ros/detector/bin
		./perception.sh
		```

	* Run processor. The main pipeline that continuously process stereo image frames to get depth + disparity maps from raw stereo images. 
		``` bash
		cd /detect-obstacles-ros/detector/bin
		./processor.sh

	* Run detector. The main pipeline that feeds image frames with our convolutional neural network for obstacle detection. It also uses depth maps to calculate the distance estimation the obstacles are from our UAS.
		``` bash
		cd /detect-obstacles-ros/detector/bin
		./detector.sh

### 4.2 Run software on Docker

To run the softwre with Docker, we use Docker-compose to bring up our Docker container and run the software as services inside the container. This file is located in /detect-obstacles-ros/dockerfiles/bringup-launch.yaml.

1. Ensure that video port connected to stereo cameras is configured correctly in docker-compose file. This video port number is also the video index port number that openCV uses in their VideoCapture initialization function. For example, if the video port connected is index 2, then under detector services, ensure that "devices" is configured at **/dev/video2**:/dev/video2 (only the first port matters, leave the second port after the ":" as it is).

2. Once the configurations of the containers are correct, we will launch our detector service in the docker container, using the docker-compose command:
``` bash
cd /Desktop/detect-obstacles-ros/dockerfiles
docker-compose -f bringup-launch.yaml up
```
