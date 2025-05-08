#!/bin/bash
set -e

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Allow commands passed as arguments (like "ros2 run ...")
exec "$@"
# ros2 run my_demo_package listener

