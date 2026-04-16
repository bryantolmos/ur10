import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('ur10_planner'), 'config', 'weld_path.yaml')
    bt_xml = os.path.join(get_package_share_directory('ur10_planner'), 'config', 'weld_bt.xml')

    moveit_config = (
        MoveItConfigsBuilder("ur10_with_custom_ee", package_name="ur10_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur10_description"),
                "urdf", "robots", "custom_ur10.urdf.xacro"
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

    supervisor_params = moveit_config.to_dict()
    supervisor_params.update({'enable_rviz': False, 'use_sim_time': True})

    path_gen_node = Node(package='ur10_planner', executable='path_generator_node', name='path_generator_node', parameters=[config_file])
    validator_node = Node(package='ur10_planner', executable='safety_validator_node', name='safety_validator')
    bt_node = Node(package='ur10_planner', executable='bt_weld_orchestrator_node', name='bt_weld_orchestrator', parameters=[{'bt_xml_file': bt_xml}])

    # The Supervisor is now a managed LifecycleNode
    supervisor_node = LifecycleNode(
        package='ur10_planner', executable='trajectory_supervisor_node', 
        name='trajectory_supervisor', namespace='', parameters=[supervisor_params]
    )

    # 1. Trigger "Configure" the moment the node process boots
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=supervisor_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == supervisor_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )

    # 2. Trigger "Activate" the moment the node finishes configuring
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=supervisor_node, goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == supervisor_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    return LaunchDescription([
        path_gen_node, validator_node, bt_node, supervisor_node, 
        configure_event, activate_event
    ])