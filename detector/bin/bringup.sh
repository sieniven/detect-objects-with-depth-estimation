#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/eloquent/setup.bash"

# setup copilot_suite workspace environment
source "../detector/install/setup.bash"

# run copilot_daa_bringup copilot_daa.launch.py
ros2 launch copilot_daa_bringup bringup.launch.py