# Comprehensive Report on UR10 Gazebo Integration (ROS 2 Jazzy)

**Contributors:** Bryant Olmos
**Date:** 2025-10-02
**Packages:** `ur10_moveit_config`, `ur10_description`
**Status:** **Functional Bringup**

---

## I. Report Objective

This document provides a detailed documentation of the troubleshooting steps, issues identified, and corrections implemented to successfully integrate our UR10 MoveIt configuration with the Gazebo simulator for physics-based control. The objective has been achieved, and the system is successfully gets initialized  on ROS 2 Jazzy, utilizing the `gz_ros2_control` framework.

---

## II. Workflow and Resolution Summary

The final, stable workflow involves a dedicated launch script (`launch_sim.sh`) that correctly prepares the shell environment for both ROS 2 and Gazebo. This script then executes a single launch file that sets up all necessary nodes for the simulation.

The debugging process resolved configuration, environment, and launch-related errors.

| Issue | Root Cause | File(s) Affected | Corrective Action Taken | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Plugin Not Found** | Gazebo's environment variables were not set, preventing it from finding the `gz_ros2_control` shared library. This was identified as a known issue in `ros_gz`. | `launch_sim.sh` (created) | Created a script to explicitly `export GZ_SIM_SYSTEM_PLUGIN_PATH`, pointing it to the system ROS 2 install path (`/opt/ros/jazzy/lib`). | **Resolved** |
| **Controller Activation Failure** | A mismatch existed between the controller's YAML configuration (requesting `velocity` commands) and the hardware interface definition in the URDF (only providing `position` commands). | `ros2_controllers.yaml` | Removed `velocity` from the `command_interfaces` list in the YAML file to match the hardware interface. | **Resolved** |
| **Missing Clock Bridge** | When using simulated time, ROS 2 nodes were not receiving time updates because the bridge from Gazebo's clock to the ROS `/clock` topic was missing. | `ur10_gazebo_moveit.launch.py` | Added a `ros_gz_bridge` `parameter_bridge` node to the main launch file to translate and publish the `/clock` topic. | **Resolved** |
| **Gazebo Launch Method** | Directly calling the `gz sim` executable was less robust and failed to properly set up resource paths compared to using the official ROS 2 launch integration. | `ur10_gazebo_moveit.launch.py` | The launch file was updated to use `IncludeLaunchDescription` to call the official `gz_sim.launch.py` from the `ros_gz_sim` package. | **Resolved** |
| **Mesh Loading Failure** | A combination of incorrect URI schemes (`model://` vs. `package://`) and Gazebo's inability to find local workspace resources blocked mesh loading. | `end_effector.urdf.xacro`, `launch_sim.sh` | Reverted all mesh paths to the ROS-standard `package://`. Added the local workspace `src` directory to the `GZ_SIM_RESOURCE_PATH` in the launch script. | **Resolved** |

---

## III. Verification of Successful Operation

The final launch script successfully starts the entire simulation stack, and the resulting log is clean of all critical errors. The system's ability to initialize all components confirms that the Gazebo backend, `ros2_control` interface, and MoveIt frontend are all communicating correctly.

**Successful Log Snippets:**

| Log Message | Significance |
| :--- | :--- |
| `[gazebo-1] [INFO] ... [controller_manager]: Resource Manager has been successfully initialized.`| Confirms the `gz_ros2_control` plugin loaded correctly and the Controller Manager is running inside Gazebo. |
| `[spawner-5] [INFO] ... Configured and activated arm_controller` <br> `[spawner-6] [INFO] ... Configured and activated joint_state_broadcaster`| Confirms both the arm trajectory controller and the joint state broadcaster connected to the Controller Manager and were successfully activated. |
| `[move_group-7] [INFO] ... You can start planning now!` | Confirms that MoveIt has a valid robot model, is receiving joint states from the simulation, and is ready to accept motion planning requests. |
| *(Absence of Errors)* | The final log is free from any `Could not find shared library` or `Unable to find file` errors, indicating all plugins and meshes loaded successfully. |

---

## IV. Conclusion and Next Steps

The `ur10_moveit_config` package and its related descriptions are now stable for physics-based simulation with Gazebo and MoveIt.
**Next Steps:**

1. **Testing:** Do thorough testing of motion planning using gazebo and moveit

**Known Minor Warnings:**

1.  **Octomap Errors:** The environment is currently missing a plugin required to load point cloud data for dynamic collision avoidance (`occupancy_map_monitor/PointCloudOctomapUpdater`). This is **safe to ignore** for basic planning but must be addressed when integrating sensor data.
2.  **End-Effector Warning:** A minor warning remains regarding the inability to identify the parent group for the `welding_torch_end_effector` in the SRDF. This is a non-critical MoveIt configuration warning and does not affect the functionality of the `arm` planning group.

---

## Appendix A: Final Launch Script

The following script, named `launch_sim.sh` and placed in the workspace root (`~/ws`)

**Usage:**
```bash
# Make the script executable once
chmod +x launch_sim.sh

# Run the simulation
./launch_sim.sh
```

**launch_sim.sh**
```bash
#!/bin/bash

echo "--- Sourcing and Configuring Environment ---"

# 1. Define the workspace directory
WS_DIR=~/ws

# 2. Source the base ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# 3. Source the local workspace overlay
source ${WS_DIR}/install/setup.bash

# 4. Explicitly set the Gazebo paths.
export GZ_SIM_SYSTEM_PLUGIN_PATH=${WS_DIR}/install/lib:/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH=${WS_DIR}/src/ur10:/opt/ros/jazzy/share

# 5. Verify the environment
echo "GZ_SIM_SYSTEM_PLUGIN_PATH is set to: ${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "GZ_SIM_RESOURCE_PATH is set to: ${GZ_SIM_RESOURCE_PATH}"
echo "------------------------------------------"

# 6. Launch the simulation
ros2 launch ur10_moveit_config ur10_gazebo_moveit.launch.py
```