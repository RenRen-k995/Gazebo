import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'obstacle_bot'
    
    # --- 1. CONFIGURATION ---
    pkg_share = get_package_share_directory(pkg_name)
    
    # A. Set the Model Path
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
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Node B: Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path}'}.items(),
    )

    # Node C: Spawn Robot
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.2'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Node D: Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node E: Custom TF Broadcaster
    tf_broadcaster = Node(
        package=pkg_name,
        executable='tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node F: Static TF for base_footprint
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node G: Simple Obstacle Avoidance
    avoidance = TimerAction(
        period=5.0,
        actions=[
            Node(
                package=pkg_name,
                executable='avoidance',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Node H: RViz2 (Optional - for visualization)
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        bridge,
        tf_broadcaster,
        base_footprint_tf,
        spawn,
        avoidance,
        rviz
    ])
