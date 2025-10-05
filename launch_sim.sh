#!/bin/bash

# 1. Define the workspace directory, change accordingly
WS_DIR=~/ws

# 2. Source the base ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# 3. Source the local workspace overlay
source ${WS_DIR}/install/setup.bash

# 4. Explicitly set the Gazebo paths.
export GZ_SIM_SYSTEM_PLUGIN_PATH=${WS_DIR}/install/lib:/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH=${WS_DIR}/src/:/opt/ros/jazzy/share

# 5. Launch the simulation
ros2 launch ur10_moveit_config ur10_gazebo_moveit.launch.py
