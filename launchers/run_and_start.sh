#!/bin/bash

source /environment.sh

source /opt/ros/${ROS_DISTRO}/setup.bash
source submission_underlay_ws/install/setup.bash
source dev_ws/install/setup.bash 


set -eux

dt-exec-BG ros2 launch random_action random_action.launch.py
dt-exec-FG ros2 run duckietown_bridge duckietown_bridge || true

copy-ros-logs
