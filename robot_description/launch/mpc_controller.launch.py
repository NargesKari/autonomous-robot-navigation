from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_info = [
        ('use_sim_time', 'true', 'Use simulation time'),
        ('horizon_steps', '5', 'MPC lookahead horizon steps'),
        ('linear_max_vel', '0.3', 'Maximum linear velocity'),
        ('angular_max_vel', '0.6', 'Maximum angular velocity'),
        ('control_period_ms', '100', 'Control loop period in milliseconds'),
    ]

    ld_args = [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in params_info]

    node_params = {n: LaunchConfiguration(n) for n, v, d in params_info}

    mpc_node = Node(
        package='robot_description',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        parameters=[node_params],
        remappings=[
            ('/global_path', '/global_path'),
            ('/amcl_pose', '/amcl_pose'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription(ld_args + [mpc_node])