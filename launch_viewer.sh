#!/bin/bash
# Note: Build files first before running this script

# 1. Source the workspace
source install/setup.bash

# 2. Define a cleanup function to kill the background process when you close the script
cleanup() {
    echo "Shutting down nodes..."
    kill $PUB_PID
}

# Trap the exit signal (Ctrl+C) to run the cleanup function
trap cleanup EXIT

# 3. Run the Object Publisher in the BACKGROUND (&)
echo "Starting Object Publisher..."
ros2 run object_pub object_publisher &
PUB_PID=$! # Capture the Process ID so we can kill it later

# 4. Sleep briefly to let the publisher start
sleep 1

# 5. Run the VTK Viewer in the FOREGROUND
echo "Starting VTK Viewer..."

# If there is an error with running vtk_node, like "error while loading shared libraries: 
#   libjawt.so: cannot open shared object file: No such file or directory"
# The fix is to find the file path, its using a few commands to find the two libraries, but this is one that has worked so far for me

# This is for Java 21, adjust if using a different version, remove the hastag to use
#LD_LIBRARY_PATH=/usr/lib/jvm/java-21-openjdk-amd64/lib:/usr/lib/jvm/java-21-openjdk-amd64/lib/server:$LD_LIBRARY_PATH ros2 run vtk_viewer vtk_node

# This fixed the "GLEW could not be initialized" error - Jesus
# Forced machine to use Xwayland backend instead of default Wayland, not sure why this is the way it is but it works
QT_QPA_PLATFORM=xcb \
WAYLAND_DISPLAY= \
LD_LIBRARY_PATH=/usr/lib/jvm/java-21-openjdk-amd64/lib:/usr/lib/jvm/java-21-openjdk-amd64/lib/server:$LD_LIBRARY_PATH \
  ros2 run vtk_viewer vtk_node

#ros2 run vtk_viewer vtk_node