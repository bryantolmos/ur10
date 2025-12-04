import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur10_with_custom_ee", package_name="ur10_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur10_description"),
                "urdf",
                "robots",
                "custom_ur10.urdf.xacro",
            ),
            mappings={"use_gazebo": "true"},
        )
        .to_moveit_configs()
    )

    vtk_viewer_node = Node(
        package="vtk_viewer",
        executable="vtk_node",
        name="vtk_viewer",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        vtk_viewer_node
    ])