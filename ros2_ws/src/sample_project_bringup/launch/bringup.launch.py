from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker',
        output='screen'
    )
    return LaunchDescription([talker])
