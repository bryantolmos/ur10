#!/bin/bash
# Note: Build files first before running this script

# 1. Source the workspace
# check if the file exist first to avoid errors if ran from wrong dir
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
else
  echo "ERROR: install/setup.bash not found, ensure you are in correct workspace directory"
  exit 1
fi

# 2. Define a cleanup function to kill the background process when you close the script
cleanup() {
    echo ""
    echo "Shutting down nodes..."
    # check if PID is set and the process is running before trying to kill it
    if [ -n "$PUD_ID" ]; then
      kill $PUD_ID 2>/dev/null
      echo "Object Publisher (PID $PUD_ID) stopped"
    fi
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

# ISSUE FIX - BRYANT
# instead of hardcoding java 21 we find where libjawt.so actually is on the computer running this
JAVA_LIB_PATH=$(find /usr/lib/jvm -name libjawt.so 2>/dev/null | head -n 1 | xargs dirname)

if [ -z "$JAVA_LIB_PATH" ]; then
  echo "WARNING: libjawt.so not found VTK might crash"
else
  echo "Found java libraries at: $JAVA_LIB_PATH"
fi

export LD_LIBRARY_PATH=$JAVA_LIB_PATH:$JAVA_LIB_PATH/server:$LD_LIBRARY_PATH

QT_QPA_PLATFORM=xcb \
WAYLAND_DISPLAY= \
ros2 run vtk_viewer vtk_node