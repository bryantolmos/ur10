import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # File paths
    config_file = os.path.join(get_package_share_directory('ur10_planner'), 'config', 'weld_path.yaml')
    bt_xml = os.path.join(get_package_share_directory('ur10_planner'), 'config', 'weld_bt.xml')

    # RESTORED FIX: The exact physical blueprint paths and Pilz parameters from V1
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

    # Supervisor specific parameters
    supervisor_params = moveit_config.to_dict()
    supervisor_params.update({'enable_rviz': False, 'use_sim_time': True})

    # Node definitions
    path_gen_node = Node(
        package='ur10_planner', executable='path_generator_node', 
        name='path_generator_node', parameters=[config_file]
    )
    
    supervisor_node = Node(
        package='ur10_planner', executable='trajectory_supervisor_node', 
        name='trajectory_supervisor', parameters=[supervisor_params]
    )
    
    validator_node = Node(
        package='ur10_planner', executable='safety_validator_node', 
        name='safety_validator'
    )
    
    bt_node = Node(
        package='ur10_planner', executable='bt_weld_orchestrator_node', 
        name='bt_weld_orchestrator', parameters=[{'bt_xml_file': bt_xml}]
    )

    return LaunchDescription([path_gen_node, supervisor_node, validator_node, bt_node])