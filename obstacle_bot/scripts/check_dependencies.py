#!/usr/bin/env python3
"""
Verify that all required ROS2 packages are available for obstacle_bot
"""

import sys

def check_package(package_name):
    """Check if a ROS2 package is available"""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package_name)
        return True
    except Exception:
        return False

def main():
    print("Checking required ROS2 packages...\n")
    
    required_packages = [
        ('obstacle_bot', 'This package (needs to be built)'),
        ('robot_state_publisher', 'Robot state publishing'),
        ('ros_gz_sim', 'Gazebo simulation'),
        ('ros_gz_bridge', 'Gazebo-ROS bridge'),
        ('tf2_ros', 'Transform broadcasting'),
        ('slam_toolbox', 'SLAM mapping'),
        ('nav2_bringup', 'Nav2 navigation stack'),
        ('rviz2', 'Visualization'),
    ]
    
    all_ok = True
    
    for package, description in required_packages:
        available = check_package(package)
        status = "✓" if available else "✗"
        
        if not available:
            all_ok = False
            
        print(f"{status} {package:25s} - {description}")
    
    print("\n" + "="*60)
    
    if all_ok:
        print("✓ All required packages are available!")
        print("\nYou can now launch the robot:")
        print("  ros2 launch obstacle_bot complete_navigation.launch.py")
        return 0
    else:
        print("✗ Some packages are missing!")
        print("\nInstall missing packages:")
        print("  sudo apt update")
        print("  sudo apt install ros-${ROS_DISTRO}-navigation2 \\")
        print("                   ros-${ROS_DISTRO}-nav2-bringup \\")
        print("                   ros-${ROS_DISTRO}-slam-toolbox \\")
        print("                   ros-${ROS_DISTRO}-ros-gz \\")
        print("                   ros-${ROS_DISTRO}-robot-state-publisher")
        print("\nFor obstacle_bot, build the package:")
        print("  cd ~/ros2_ws")
        print("  colcon build --packages-select obstacle_bot")
        print("  source install/setup.bash")
        return 1

if __name__ == '__main__':
    sys.exit(main())
