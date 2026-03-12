from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ai_params = {
        'update_rate': 1000.0,
    }
    ao_params = {
        'update_rate': 1000.0,
        'subscribe_topic': '/actuators/valve_voltage',
    }
    pse53x_params = {
        'subscribe_topic_name': 'ai1616llpe/voltage',
        'sensor_idx': [0, 1, 2, 3, 4],
        'sensor_type_str': ['1MPa', '1MPa', '1MPa', '1MPa', '101kPa'],
        'publish_topic_name': '/pressure',
    }

    # Analog Input
    ai_node = Node(
        package='peripheral',
        executable='ai1616llpe_test',
        name='AI_board',
        output='screen',
        parameters=[ai_params],
    )
    # Analog Output
    ao_node = Node(
        package='peripheral',
        executable='ao1608llpe_test',
        name='AO_board',
        output='screen',
        parameters=[ao_params],
    )
    # Pressure Sensor
    pse53x_node = Node(
        package='peripheral',
        executable='pse53x',
        name='pse53x',
        output='screen',
        parameters=[pse53x_params],
    )

    return LaunchDescription([
        ai_node,
        ao_node,
        pse53x_node,
    ])
