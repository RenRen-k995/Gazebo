# Quick Start Guide - ROS2 Navigation with Gazebo Harmonic

Get up and running with ROS2 navigation in minutes!

## Prerequisites Check

Before starting, ensure you have:
- ROS2 (Humble or Jazzy)
- Gazebo Harmonic
- A working ROS2 workspace

## 5-Minute Setup

### 1. Install Dependencies (2 minutes)

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-interactive-marker-twist-server
```

### 2. Build the Packages (1 minute)

```bash
cd ~/ros2_ws
colcon build --packages-select bme_ros2_navigation bme_ros2_navigation_py
source install/setup.bash
```

### 3. Test Basic Simulation (2 minutes)

```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

You should see Gazebo Harmonic with a robot and home environment.

## Common First Tasks

See the full [README.md](README.md) for detailed instructions on:
- SLAM Mapping
- Localization
- Autonomous Navigation
- Waypoint Following

For detailed troubleshooting, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
