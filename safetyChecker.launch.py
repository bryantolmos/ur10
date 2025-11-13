from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the config file inside the package
    config = PathJoinSubstitution([
        FindPackageShare("ur10_planner"),
        "config",
        "safety_checker_params.yaml"
    ])

    safety_checker_node = Node(
        package="ur10_planner",
        executable="safety_checker_node",
        name="safety_checker_node",
        output="screen",
        parameters=[config],
        remappings=[
            # Optional: adapt topics if your planner uses different names
            ("/planned_trajectory", "/ur10_planner/planned_trajectory"),
            ("/safe_trajectory", "/ur10_planner/safe_trajectory")
        ]
    )

    return LaunchDescription([safety_checker_node])


# Build workspace + running launch file
# colcon build --packages-select ur10_planner
# source install/setup.bash
# ros2 launch ur10_planner safetyChecker.launch.py

# Expected logs:
# [INFO] [safety_checker_node]: Safety Checker Node initializing...
# [INFO] [safety_checker_node]: Safety Checker ready. Params -> max_vel: 1.20 rad/s, max_acc: 2.50 rad/s²

