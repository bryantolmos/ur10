import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
)

def generate_launch_description():
    # Find the absolute path to the correct URDF file
    urdf_xacro_path = os.path.join(
        get_package_share_directory("ur10_description"),
        "urdf",
        "robots",
        "custom_ur10.urdf.xacro",
    )

    # Update MoveItConfigsBuilder to use the correct URDF path
    moveit_config = (
        MoveItConfigsBuilder("ur10_with_custom_ee", package_name="ur10_moveit_config")
        .robot_description(
            file_path=urdf_xacro_path,  # <--- CORRECTED PATH
            mappings={
                "use_gazebo": "true",
            },
        )
        .robot_description_semantic(file_path="config/ur10_with_custom_ee.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', 'empty.sdf'],
        output='screen'
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "ur10",
            "-allow_renaming",
            "true",
        ],
        output="screen",
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # Generate other MoveIt launches
    move_group_launch = generate_move_group_launch(moveit_config)
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)
    static_tf_launch = generate_static_virtual_joint_tfs_launch(moveit_config)
    spawn_controllers_launch = generate_spawn_controllers_launch(moveit_config)

    return LaunchDescription(
        [
            start_gazebo_cmd,
            spawn_entity,
            rsp_node,
            move_group_launch,
            moveit_rviz_launch,
            static_tf_launch,
            spawn_controllers_launch,
        ]
    )