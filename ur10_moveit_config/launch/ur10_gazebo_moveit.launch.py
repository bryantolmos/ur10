import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        "custom_ur10.urdf.xacro"
    )

    # Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10_with_custom_ee", package_name="ur10_moveit_config")
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={"use_gazebo": "true"},
        )
        .robot_description_semantic(file_path="config/ur10_with_custom_ee.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "ur10",
            "-allow_renaming", "true",
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

    # Controller Spawners
    spawn_controllers_launch = generate_spawn_controllers_launch(moveit_config)

    # Bridge to publish the /clock topic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # === ADD THESE COMPONENTS BACK ===
    # Generate MoveIt launches
    move_group_launch = generate_move_group_launch(moveit_config)
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)
    static_tf_launch = generate_static_virtual_joint_tfs_launch(moveit_config)
    # ===============================

    return LaunchDescription([
        gazebo,
        clock_bridge,
        rsp_node,
        spawn_entity,
        spawn_controllers_launch,
        # === ADD THESE TO THE LAUNCH LIST ===
        move_group_launch,
        moveit_rviz_launch,
        static_tf_launch,
        # ====================================
    ])