#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([    
        Node(
            package='copilot_daa_perception',
            node_executable='copilot_perception',
            output='screen',
            emulate_tty=True),
        Node(
            package='copilot_daa_perception',
            node_executable='copilot_processor',
            output='screen',
            emulate_tty=True),
        Node(
            package='copilot_daa_detection',
            node_executable='copilot_detector',
            output='screen',
            emulate_tty=True),
    ])