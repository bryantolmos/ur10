import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Step 1: Build the MoveIt configuration dictionary
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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Step 2: Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Step 3: Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "ur10"],
        output="screen",
    )

    # Step 4: Start the Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # Step 5: Start the ros2_control spawner nodes
    from moveit_configs_utils.launches import generate_spawn_controllers_launch
    spawn_controllers = generate_spawn_controllers_launch(moveit_config)

    # Step 6: Start the clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Step 7: Manually define the MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, 
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Step 8: Manually define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # Step 9: Launch lifecycle planner node
    lifecycle_planner_node = Node(
        package="ur10_planner",
        executable="lifecycle_planner",
        name="lifecycle_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        spawn_controllers,
        move_group_node,
        rviz_node,
        lifecycle_planner_node,
    ])