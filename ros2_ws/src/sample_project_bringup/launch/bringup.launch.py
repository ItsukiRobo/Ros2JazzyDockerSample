from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Analog Input
    ai_node = Node(
        package='peripheral',
        executable='ai1616llpe_test',
        name='AI_board',
        output='screen',
        parameters=[{'update_rate': 1000.0}],
    )
    # Analog Output
    ao_node = Node(
        package='peripheral',
        executable='ao1608llpe_test',
        name='AO_board',
        output='screen',
        parameters=[{
            'update_rate': 1000.0,
            'subscribe_topic': '/actuators/valve_voltage',
        }],
    )

    return LaunchDescription([
        ai_node,
        ao_node,
    ])
