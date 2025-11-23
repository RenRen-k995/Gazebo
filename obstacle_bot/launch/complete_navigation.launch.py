import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'obstacle_bot'
    
    # --- 1. CONFIGURATION ---
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # A. Set the Model Path (So Gazebo finds the furniture)
    gazebo_models_path = os.path.join(os.environ.get('HOME', '/home/runner'), 'gazebo_models')
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gazebo_models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_models_path

    # B. Select the World File
    world_path = os.path.join(pkg_share, 'worlds', 'house.sdf')
    
    # C. File Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2.rviz')
    
    # D. Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- 2. NODES ---
    
    # Node A: Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # Node B: Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path}'}.items(),
    )

    # Node C: Spawn Robot (delayed to ensure Gazebo is ready)
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.2'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Node D: Bridge (Gazebo <-> ROS2 communication)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node E: Custom TF Broadcaster (odom -> base_link transform)
    tf_broadcaster = Node(
        package=pkg_name,
        executable='tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node F: Static TF for base_link -> base_footprint
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node G: SLAM Toolbox (for mapping and localization) - Delayed
    slam = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Node H: Nav2 (Navigation Stack) - Delayed to allow SLAM to initialize
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params
                }.items(),
            )
        ]
    )

    # Node I: RViz2 (Visualization) - Delayed
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp,
        gazebo,
        bridge,
        tf_broadcaster,
        base_footprint_tf,
        spawn,
        slam,
        nav2,
        rviz
    ])
