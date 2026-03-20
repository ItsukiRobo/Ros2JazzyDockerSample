from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ai_params = {
        'update_rate': 1000.0,
    }
    ao_params = {
        'update_rate': 1000.0,
        'subscribe_topic': '/controller/output_voltage',
    }
    pressure_and_force_params = {
        'subscribe_topic_name': 'ai1616llpe/voltage',
        'publish_topic_name': '/pressure_and_force',
        'pressure_sensor_idx': [0, 1, 2, 3, 4, 5],
        'pressure_sensor_type_str': ['1MPa', '1MPa', '1MPa', '1MPa', '101kPa', '101kPa'],
        'pressure_cutoff_frequency_hz': 10.0,
        'loadcell_signal_plus_idx': [6],
        'loadcell_signal_minus_idx': [7],
        'loadcell_cutoff_frequency_hz': [10.0],
    }
    cnt_params = {
        'cnt_indexes': 4,
        'mm_per_step': 0.01,
        'subscribe_topic_name': '/pressure_and_force',
    }
    cylinder_force_controller_params = {
        'subscribe_topic_name': '/pressure_and_force',
        'publish_topic_name': '/controller/output_voltage',
        'debug_publish_topic_name': '/controller/debug',
        'head_pressure_index': 0,
        'rod_pressure_index': 1,
        'control_period_s': 0.001,
        'kp': 0.04,
        'ki': 0.01,
        'kd': 0.0,
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
    # Pressure and Force
    pressure_and_force_node = Node(
        package='peripheral',
        executable='pressure_and_force',
        name='pressure_and_force',
        output='screen',
        parameters=[pressure_and_force_params],
    )
    # Counter Board  (Encoder)
    cnt_node = Node(
        package='peripheral',
        executable='cnt3204mtlpe_test',
        name='cnt3204mtlpe',
        output='screen',
        parameters=[cnt_params],
    )
    # Cylinder Force Controller
    cylinder_force_controller_node = Node(
        package='controller',
        executable='cylinder_force_controller',
        name='cylinder_force_controller',
        output='screen',
        parameters=[cylinder_force_controller_params],
    )

    return LaunchDescription([
        ai_node,
        ao_node,
        pressure_and_force_node,
        cnt_node,
        cylinder_force_controller_node,
    ])
