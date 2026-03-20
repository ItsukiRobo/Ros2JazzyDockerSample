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
    pressure_sensor_params = {
        'subscribe_topic_name': 'ai1616llpe/voltage',
        'sensor_idx': [0, 1, 2, 3, 5],
        'sensor_type_str': ['1MPa', '1MPa', '1MPa', '1MPa', '101kPa'],
        'publish_topic_name': '/pressure',
        'cutoff_frequency_hz': 10.0,
    }
    cnt_params = {
        'cnt_indexes': 4,
        'mm_per_step': 0.01,
    }
    loadcell_params = {
        'subscribe_topic_name': 'ai1616llpe/voltage',
        'publish_topic_name': '/loadcell/debug',
        'publish_raw_difference': True,
        'signal_plus_idx': [0],
        'signal_minus_idx': [1],
        'cutoff_frequency_hz': [0.0],
        'rated_load_n': [1.0],
        'rated_output_voltage_v': [1.0],
        'zero_balance_voltage_v': [0.0],
    }
    cylinder_force_controller_params = {
        'subscribe_topic_name': '/pressure',
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
    # Pressure Sensor
    pressure_sensor_node = Node(
        package='peripheral',
        executable='pressure_sensor',
        name='pressure_sensor',
        output='screen',
        parameters=[pressure_sensor_params],
    )
    # Counter Board  (Encoder)
    cnt_node = Node(
        package='peripheral',
        executable='cnt3204mtlpe_test',
        name='cnt3204mtlpe',
        output='screen',
        parameters=[cnt_params],
    )
    # Load Cell
    loadcell_node = Node(
        package='peripheral',
        executable='loadcell',
        name='loadcell',
        output='screen',
        parameters=[loadcell_params],
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
        pressure_sensor_node,
        cnt_node,
        loadcell_node,
        cylinder_force_controller_node,
    ])
