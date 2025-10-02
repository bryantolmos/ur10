import os

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def runLaunch():
    return LaunchDescription([
        LifecycleNode(
            package='nodeLifecycleBase',
            executable='nodeLifecycleLaunch',
            name='nodeLifecycleLaunch',
            output='screen',
            emulate_tty=True
        )
    ])

#Attempt to run with
#ros2 launch nodeLifecycleBase nodeLifecycleLaunch.py

