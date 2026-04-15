#!/bin/bash

# 1. Define the workspace directory, change accordingly
WS_DIR=~/ur10

# 2. Source the base ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# 3. Source the local workspace overlay
source ${WS_DIR}/install/setup.bash

# 4. Dynamically find the exact package locations
UR_SHARE=$(ros2 pkg prefix ur_description)/share
CUSTOM_SHARE=$(ros2 pkg prefix ur10_description)/share

# 5. Export Gazebo Paths
export GZ_SIM_SYSTEM_PLUGIN_PATH=${WS_DIR}/install/lib:/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH=${UR_SHARE}:${CUSTOM_SHARE}:${WS_DIR}/src:/opt/ros/jazzy/share

# 6. Launch the simulation
ros2 launch ur10_moveit_config ur10_gazebo_moveit.launch.py
