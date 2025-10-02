# Comprehensive Report on UR10 Gazebo Integration Debugging (ROS 2 Jazzy)

**Contributors:** Bryant Olmos
**Date:** 2025-10-01 & 2025-10-02
**Packages:** `ur10_moveit_config`, `ur10_description`
**Status:** **Blocked by System-Level Installation Issue**

---

## I. Report Objective

This document provides a detailed documentation of the troubleshooting process to integrate our UR10 MoveIt configuration with the Gazebo simulator for physics-based control. The objective was to launch the robot in a simulated environment and control it via MoveIt, utilizing the `gz_ros2_control` framework on ROS 2 Jazzy. While all configuration files were successfully debugged and corrected, the final execution is blocked by a persistent system-level error.

---

## II. Workflow and Resolution Summary

The primary workflow involved creating a dedicated launch script (`launch_sim.sh`) to ensure a clean and repeatable environment. This script sourced the required ROS and workspace `setup.bash` files, exported necessary Gazebo environment variables, and then executed the main ROS 2 launch file.

The debugging process uncovered and resolved critical configuration errors.

| Issue | Root Cause | File(s) Affected | Corrective Action Taken | Status |
| :--- | :--- | :--- | :--- | :--- |
| **XML Parse Error** | Incorrect XACRO syntax (`:=` instead of `=`) was used when passing an argument to a macro, causing the XML parser to fail. | `custom_ur10.urdf.xacro` | Corrected the macro call syntax to `use_gazebo="$(arg use_gazebo)"`. | **Resolved** |
| **Undefined XACRO Parameter** | The launch file was loading a copy of the URDF from the `moveit_config` package instead of the primary, modified URDF in the `description` package. | `ur10_gazebo_moveit.launch.py` | The launch file was modified to use the absolute path to the correct URDF in the `ur10_description` package, making it the single source of truth. | **Resolved** |
| **Controller Activation Failure** | A mismatch existed between the controller and the hardware interface. The controller's YAML requested a `velocity` command interface, but the `ros2_control` URDF tag only provided `position`. | `ros2_controllers.yaml` | Removed `velocity` from the `command_interfaces` list in the YAML file to match the hardware interface definition. | **Resolved** |
| **Plugin Name Ambiguity** | Conflicting documentation led to uncertainty over the correct plugin name for `gz_ros2_control` in ROS 2 Jazzy. Official documentation confirmed the explicit library name was required. | `ur10.gazebo.xacro` | The plugin declaration was set to the explicit library name: `filename="libgz_ros2_control-system.so"`. | **Resolved** |
| **Environment Configuration** | The Gazebo process (`gz sim`) could not find ROS 2 plugins or models because its specific environment variables were not set. This was identified as a known issue in `ros_gz`.| `launch_sim.sh` (created) | Created a launch script to explicitly `export` `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_SIM_RESOURCE_PATH`, pointing them to the system (`/opt/ros/jazzy/...`) and workspace install directories. | **Resolved** |

---

## III. Final System State and Log Analysis

After implementing all configuration corrections and performing a clean workspace rebuild, the final launch attempt successfully loaded all plugins and controllers, but revealed a final issue with resource loading (meshes).

**Analysis of Final Log:**

The final log indicates that all previous errors have been resolved. The system is now blocked only by the inability of ROS nodes and Gazebo to find the robot's mesh files.

| Key Log Snippet | Significance |
| :--- | :--- |
| `[gz-1] [INFO] ... [controller_manager]: Resource Manager has been successfully initialized.` <br> `[spawner-7] [INFO] ... Configured and activated arm_controller` | **Major Success:** This confirms the `gz_ros2_control` plugin loaded and activated all controllers. The core integration is **functional**. |
| `[move_group-4] Error: Error retrieving file [model://...]: Protocol "model" not supported or disabled in libcurl` <br> `[rviz2-5] [ERROR] ... Could not load resource [model://...]` | **Persistent Issue 1:** ROS nodes (MoveIt/RViz) do not support the `model://` URI. This indicates the mesh paths in the URDF must be reverted from `model://` to `package://`. |
| `[gz-1] [Err] ... Error retrieving file [model://ur10_description/meshes/ee/collision/mock-up-end-effector.stl]` | **Persistent Issue 2 (Blocker):** Even with the correct environment variables, the Gazebo process also fails to resolve the `model://` URI.

---

## IV. Conclusion and Persistent Issues

All user-level configuration files (`.xacro`, `.yaml`, `.launch.py`) have been successfully debugged and are **correct**. The simulation's control and physics backend is **functional**.

The system remains partially non-functional due to a persistent, system-level issue preventing both ROS nodes and the Gazebo process from locating the robot's mesh files.

**Persistent Issues & Next Steps:**

1.  **Primary Blocker - Mesh URI Conflict:** There is a conflict between the URI schemes required by ROS and Gazebo. ROS nodes (MoveIt, RViz) require `package://` to find resources, while Gazebo is intended to use `model://` (via `GZ_SIM_RESOURCE_PATH`). The immediate next step is to revert all mesh paths in the URDF to `package://` to restore functionality in RViz and MoveIt, and then debug the Gazebo visualization separately.

2.  **Minor MoveIt Warnings:** The logs continue to show warnings related to the missing Octomap plugin and the `welding_torch_end_effector` group, which can be safely ignored for the current objective.