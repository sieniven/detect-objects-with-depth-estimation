#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/eloquent/setup.bash"

# setup copilot_suite workspace environment
source "../detector/install/setup.bash"

# run perception processor
ros2 run perception processor