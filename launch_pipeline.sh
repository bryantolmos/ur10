#!/bin/bash

# 1. Define the workspace directory
WS_DIR=~/ur10

# 2. Source the base ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# 3. Source the local workspace overlay
source ${WS_DIR}/install/setup.bash

# 4. Launch the V1 Planner Pipeline
ros2 launch ur10_planner v1_pipeline.launch.py