# Week 7-8: ROS2 Navigation with Gazebo Harmonic

This repository contains ROS2 packages for robot navigation using SLAM, localization, and the Nav2 navigation stack with Gazebo Harmonic simulation.

Based on the [MOGI-ROS Week 7-8 Navigation course](https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation).

## Overview

This project demonstrates:
- **SLAM (Simultaneous Localization and Mapping)** using SLAM Toolbox
- **Localization** using AMCL and SLAM Toolbox
- **Autonomous Navigation** using Nav2 navigation stack
- **Waypoint Navigation** for following predefined paths
- **Exploration** capabilities for autonomous environment mapping
- **Gazebo Harmonic** simulation integration

## Packages

### bme_ros2_navigation
Main ROS2 package (ament_cmake) containing:
- Robot URDF model with differential drive and lidar
- Gazebo world files (empty, home)
- Launch files for mapping, localization, and navigation
- Configuration files for SLAM, Nav2, AMCL, and EKF
- Pre-configured RViz layouts
- Pre-built maps

### bme_ros2_navigation_py
Python utilities package (ament_python) containing:
- `send_initialpose.py` - Publishes initial robot pose for localization
- `slam_toolbox_load_map.py` - Loads serialized SLAM Toolbox maps
- `follow_waypoints.py` - Waypoint following node

## Prerequisites

- **ROS2**: Humble or Jazzy recommended
- **Gazebo Harmonic**: Latest version
- **Nav2**: Navigation stack for ROS2
- **SLAM Toolbox**: For mapping and localization
- **robot_localization**: For sensor fusion (EKF)
- **interactive_marker_twist_server**: For manual robot control in RViz

## Installation

### 1. Install ROS2 Dependencies

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-ros-gz \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-robot-localization \
                 ros-${ROS_DISTRO}-interactive-marker-twist-server
```

### 2. Clone and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone <this-repository-url>

# Build packages
cd ~/ros2_ws
colcon build --packages-select bme_ros2_navigation bme_ros2_navigation_py

# Source workspace
source install/setup.bash
```

## Usage

### Basic Simulation

Launch the robot in Gazebo without navigation:

```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

This will:
- Start Gazebo Harmonic with the home world
- Spawn the mogi_bot robot
- Launch sensor fusion (EKF)
- Bridge Gazebo and ROS2 topics

### Mapping (SLAM)

Create a map of the environment:

**Terminal 1** - Start simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

**Terminal 2** - Start SLAM mapping:
```bash
ros2 launch bme_ros2_navigation mapping.launch.py
```

Control the robot using the interactive marker in RViz or keyboard teleop to explore and build the map.

**Save the map:**
```bash
# Save as .pgm and .yaml
ros2 run nav2_map_server map_saver_cli -f my_map

# Or use SLAM Toolbox's "Save Map" and "Serialize Map" buttons in RViz
```

### Localization

Localize the robot on an existing map using AMCL:

**Terminal 1** - Start simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

**Terminal 2** - Start localization:
```bash
ros2 launch bme_ros2_navigation localization.launch.py
```

Set the initial pose using the "2D Pose Estimate" tool in RViz.

**Alternative**: Use SLAM Toolbox for localization:
```bash
ros2 launch bme_ros2_navigation localization_slam_toolbox.launch.py
```

### Navigation

Navigate autonomously with Nav2:

**Terminal 1** - Start simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

**Terminal 2** - Start navigation:
```bash
ros2 launch bme_ros2_navigation navigation.launch.py
```

1. Set initial pose using "2D Pose Estimate" in RViz
2. Set goal using "2D Goal Pose" in RViz
3. Watch the robot navigate autonomously

### Waypoint Navigation

**GUI Method:**
1. Launch simulation and navigation as above
2. Add "Nav2 Goal" toolbar in RViz (if not present)
3. Switch to waypoint mode
4. Click waypoints on the map
5. Start navigation

**Programmatic Method:**

**Terminal 3** - Run waypoint follower:
```bash
ros2 run bme_ros2_navigation_py follow_waypoints
```

Edit `follow_waypoints.py` to customize waypoints.

### Navigation with SLAM

Navigate while simultaneously building the map:

**Terminal 1** - Start simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

**Terminal 2** - Start SLAM + Navigation:
```bash
ros2 launch bme_ros2_navigation navigation_with_slam.launch.py
```

Set goals using "2D Goal Pose" - the robot will navigate while continuously updating the map.

### Exploration

Autonomous exploration of unknown environments:

**Terminal 1** - Start simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

**Terminal 2** - Start SLAM + Navigation:
```bash
ros2 launch bme_ros2_navigation navigation_with_slam.launch.py
```

**Terminal 3** - Start exploration (requires m-explore-ros2):
```bash
ros2 launch explore_lite explore.launch.py
```

The robot will autonomously explore and map the entire environment.

## Project Structure

```
.
├── bme_ros2_navigation/              # Main navigation package (ament_cmake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/                       # Configuration files
│   │   ├── amcl_localization.yaml
│   │   ├── ekf.yaml
│   │   ├── gz_bridge.yaml
│   │   ├── navigation.yaml
│   │   ├── slam_toolbox_localization.yaml
│   │   ├── slam_toolbox_mapping.yaml
│   │   └── waypoints.yaml
│   ├── launch/                       # Launch files
│   │   ├── check_urdf.launch.py
│   │   ├── localization.launch.py
│   │   ├── localization_slam_toolbox.launch.py
│   │   ├── mapping.launch.py
│   │   ├── navigation.launch.py
│   │   ├── navigation_with_slam.launch.py
│   │   ├── spawn_robot.launch.py
│   │   └── world.launch.py
│   ├── maps/                         # Saved maps
│   │   ├── my_map.pgm
│   │   ├── my_map.yaml
│   │   ├── serialized.data
│   │   └── serialized.posegraph
│   ├── meshes/                       # 3D models
│   │   ├── lidar.dae
│   │   ├── mogi_bot.dae
│   │   └── wheel.dae
│   ├── rviz/                         # RViz configurations
│   │   ├── localization.rviz
│   │   ├── mapping.rviz
│   │   ├── navigation.rviz
│   │   ├── rviz.rviz
│   │   └── urdf.rviz
│   ├── urdf/                         # Robot description
│   │   ├── materials.xacro
│   │   ├── mogi_bot.gazebo
│   │   └── mogi_bot.urdf
│   └── worlds/                       # Gazebo worlds
│       ├── empty.sdf
│       └── home.sdf
│
└── bme_ros2_navigation_py/           # Python utilities (ament_python)
    ├── bme_ros2_navigation_py/
    │   ├── __init__.py
    │   ├── follow_waypoints.py
    │   ├── send_initialpose.py
    │   └── slam_toolbox_load_map.py
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Gazebo Harmonic                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   Physics    │  │  DiffDrive   │  │    Lidar     │          │
│  │   Engine     │  │   Plugin     │  │   Sensor     │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
                              ▲ │
                    ros_gz_bridge
                              │ ▼
┌─────────────────────────────────────────────────────────────────┐
│                           ROS 2                                  │
│                                                                   │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐    │
│  │ robot_         │  │  SLAM Toolbox  │  │     Nav2       │    │
│  │ localization   │  │  or AMCL       │  │  (planning &   │    │
│  │ (EKF)          │  │  (map->odom)   │  │  control)      │    │
│  └────────────────┘  └────────────────┘  └────────────────┘    │
│                                                                   │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐    │
│  │ robot_state_   │  │  trajectory_   │  │     RViz2      │    │
│  │  publisher     │  │    server      │  │(visualization) │    │
│  └────────────────┘  └────────────────┘  └────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

## TF Tree

```
map (from SLAM/AMCL)
 └─ odom (from robot_localization EKF)
     └─ base_link (from robot_state_publisher)
         ├─ base_footprint
         ├─ lidar_link
         ├─ imu_link
         └─ wheel_links
```

## Key Features

### SLAM Toolbox
- Real-time mapping with loop closure
- Map serialization for continued mapping
- Localization mode for pose estimation on existing maps

### AMCL (Adaptive Monte Carlo Localization)
- Particle filter-based localization
- Configurable particle count and distributions
- Works with pre-built maps

### Nav2 Navigation Stack
- Global and local path planning
- Dynamic obstacle avoidance
- Behavior trees for complex navigation tasks
- Cost map layers (static, inflation, obstacle)
- Multiple controller and planner plugins

### Robot Localization (EKF)
- Sensor fusion between odometry and IMU
- Smooth and accurate state estimation
- Covariance tracking

## Troubleshooting

### No map appearing
- Ensure SLAM Toolbox is running: `ros2 node list | grep slam`
- Check scan topic: `ros2 topic echo /scan --no-arr`
- Verify TF tree: `ros2 run tf2_tools view_frames`

### Robot not moving
- Check cmd_vel: `ros2 topic echo /cmd_vel`
- Verify odometry: `ros2 topic echo /odom`
- Check Gazebo bridge: `ros2 topic list | grep gz`

### Localization not working
- Set initial pose using "2D Pose Estimate" in RViz
- Check map is loaded: `ros2 topic echo /map --once`
- Verify scan data: `ros2 topic hz /scan`

### Navigation failures
- Ensure map and robot frames are correct
- Check costmap: visible in RViz
- Verify goal pose is reachable (not in obstacle)
- Check Nav2 lifecycle nodes are active: `ros2 lifecycle list`

### Clock/Time issues
- Ensure `use_sim_time:=true` is set for all nodes
- Check clock topic: `ros2 topic hz /clock`

## Development

### Running Tests

```bash
colcon test --packages-select bme_ros2_navigation bme_ros2_navigation_py
```

### Code Style

```bash
# For Python
flake8 bme_ros2_navigation_py/

# For CMake/launch files
ament_lint bme_ros2_navigation/
```

## Resources

- [ROS2 Navigation Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Harmonic](https://gazebosim.org/)
- [MOGI-ROS Course](https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation)

## License

Apache License 2.0

## Maintainer

Based on work by David Dudas (david.dudas@outlook.com)

## Acknowledgments

This project is based on the BME MOGI ROS2 Navigation course materials.
