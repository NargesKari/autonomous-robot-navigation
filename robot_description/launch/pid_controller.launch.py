from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_info = [
        ('use_sim_time', 'true', 'Use simulation time if true'),
        ('max_linear_vel', '0.5', 'Maximum linear velocity (m/s)'),
        ('max_angular_vel', '1.0', 'Maximum angular velocity (rad/s)'),
        ('linear_kp', '0.5', 'Linear velocity proportional gain'),
        ('linear_ki', '0.0', 'Linear velocity integral gain'),
        ('linear_kd', '0.1', 'Linear velocity derivative gain'),
        ('angular_kp', '1.0', 'Angular velocity proportional gain'),
        ('angular_ki', '0.0', 'Angular velocity integral gain'),
        ('angular_kd', '0.1', 'Angular velocity derivative gain'),
        ('lookahead_distance', '0.5', 'Lookahead distance for path following (m)'),
        ('goal_tolerance', '0.2', 'Goal tolerance distance (m)'),
        ('control_frequency', '20.0', 'Control loop frequency (Hz)'),
    ]

    ld_args = [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in params_info]

    node_params = {n: LaunchConfiguration(n) for n, v, d in params_info}

    pid_path_node = Node(
        package='robot_description',
        executable='pid_controller_node',
        name='pid_controller',
        output='screen',
        parameters=[node_params]
    )

    return LaunchDescription(ld_args + [pid_path_node])