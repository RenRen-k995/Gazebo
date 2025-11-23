import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'obstacle_bot'
    file_subpath = 'urdf/robot.urdf'

    # Find the file
    urdf_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    
    # Read the file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Node 1: Publish the robot state (links and joints)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # Node 2: Joint State Publisher GUI (sliders to move wheels)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz2 (The Visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])