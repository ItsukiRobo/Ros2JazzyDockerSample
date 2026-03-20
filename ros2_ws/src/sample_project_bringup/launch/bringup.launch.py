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
        'publish_topic_name': '/pressure_force_and_length',
    }
    cylinder_controller_params = {
        'subscribe_topic_name': '/pressure_force_and_length',
        'publish_topic_name': '/controller/output_voltage',
        'debug_publish_topic_name': '/controller/debug',
        'action_name': '/cylinder_controller/track_sine_force',
        'length_action_name': '/cylinder_controller/track_sine_length',
        'head_pressure_index': 0,
        'rod_pressure_index': 1,
        'force_index': 6,
        'length_index': 7,
        'control_period_s': 0.001,
        'kp': 0.04,
        'ki': 0.01,
        'kd': 0.0,
        'force_kp': 0.1,
        'force_ki': 0.0,
        'force_kd': 0.0,
        'force_output_limit_n': 200.0,
        'length_kp': 0.1,
        'length_ki': 0.0,
        'length_kd': 0.0,
        'base_pressure_kpa': 50.0,
        'startup_target_force_n': 0.0,
        'feasible_force_min_n': -200.0,
        'feasible_force_max_n': 0.0,
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
    cylinder_controller_node = Node(
        package='controller',
        executable='cylinder_controller',
        name='cylinder_controller',
        output='screen',
        parameters=[cylinder_controller_params],
    )

    return LaunchDescription([
        ai_node,
        ao_node,
        pressure_and_force_node,
        cnt_node,
        cylinder_controller_node,
    ])
