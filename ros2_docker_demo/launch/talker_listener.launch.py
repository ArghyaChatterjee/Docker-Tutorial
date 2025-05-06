from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='demo_nodes',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])