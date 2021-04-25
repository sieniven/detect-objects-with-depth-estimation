#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/eloquent/setup.bash"

# setup copilot_suite workspace environment
source "../copilot-daa/install/setup.bash"

# run perception perception
ros2 run perception perception