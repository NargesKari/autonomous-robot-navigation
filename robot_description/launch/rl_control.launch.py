import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_description')
    world = os.path.join(bringup_dir, "world", "depot.sdf")
    urdf_file = os.path.join(bringup_dir, 'src', 'description', 'robot.urdf')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'config.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo in headless mode (no GUI) for faster training'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(bringup_dir, 'world'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc}
        ]
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r -v 4 -s ", world]}.items(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )
    
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    bridge_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    bridge_service_control = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/depot/control@ros_gz_interfaces/srv/ControlWorld'
        ],
        output='screen'
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot",
            "-topic", "/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.9",
        ],
        output="screen",
    )

    frame_id_converter_node = Node(
        package='robot_description',
        executable='frame_id_converter_node',
        name='frame_id_converter_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ekf_diff_imu_node = Node(
        package='robot_description',
        executable='ekf_diff_imu_node',
        name='ekf_diff_imu_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        headless_arg,
        use_rviz_arg,
        gz_resource_path,
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        bridge_topics,
        bridge_service_control,
        spawn_entity,
        frame_id_converter_node,
        ekf_diff_imu_node,
        rviz_node,
    ])
