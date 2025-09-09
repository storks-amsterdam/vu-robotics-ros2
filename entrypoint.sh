#!/bin/bash
ROS_PATH=/opt/ros/jazzy
source ${ROS_PATH}/setup.bash
source /opt/conda/etc/profile.d/conda.sh
conda activate ros2

exec "$@"