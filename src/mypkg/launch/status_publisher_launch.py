import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',
            executable='status_publisher',
            name='status_publisher_node',
            output='screen'
        )
    ])
