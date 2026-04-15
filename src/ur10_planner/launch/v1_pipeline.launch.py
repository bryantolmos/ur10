import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the parameters for the Path Generator
    config_file = os.path.join(
        get_package_share_directory('ur10_planner'),
        'config',
        'weld_path.yaml'
    )

    # 2. Build the MoveIt parameters for the Supervisor with Pilz pipeline
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
        .robot_description_semantic(file_path="config/ur10_with_custom_ee.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    # Combine MoveIt dict with our custom parameters
    supervisor_params = moveit_config.to_dict()
    supervisor_params.update({
        'enable_rviz': False,        # Publish trajectory visualization
        'use_sim_time': True,
        'planning_group': 'arm',
        'ee_link_name': 'custom_tcp_link'
    })

    # 3. Define the Nodes
    path_generator = Node(
        package='ur10_planner',
        executable='path_generator_node',
        name='path_generator_node',
        parameters=[config_file]
    )

    trajectory_supervisor = Node(
        package='ur10_planner',
        executable='trajectory_supervisor_node',
        name='trajectory_supervisor',
        parameters=[supervisor_params]
    )

    safety_validator = Node(
        package='ur10_planner',
        executable='safety_validator_node',
        name='safety_validator'
    )

    return LaunchDescription([
        path_generator,
        trajectory_supervisor,
        safety_validator
    ])