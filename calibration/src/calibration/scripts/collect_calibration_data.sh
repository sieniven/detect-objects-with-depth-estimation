#!/bin/bash
#(/ros_entrypoint.sh ros2 launch copilot_daa_calibration stereo_img_saver_server.launch.py; kill -INT -$$) &
(/ros_entrypoint.sh ros2 launch copilot_daa_calibration synced_stereo_img_saver_server.launch.py; kill -INT -$$) &
python3  ../copilot_daa_calibration/user_input_node.py --client-service /stereo/image_saver ; kill -INT -$$
