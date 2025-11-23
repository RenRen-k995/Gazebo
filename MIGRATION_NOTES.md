# Migration Notes: obstacle_bot → bme_ros2_navigation

## Overview

This document describes the migration from the previous `obstacle_bot` package to the new `bme_ros2_navigation` structure based on the MOGI-ROS Week 7-8 Navigation course.

## What Changed

### New Package Structure

The repository now contains two packages instead of one:

1. **bme_ros2_navigation** (ament_cmake)
   - Main navigation package
   - Contains URDF, worlds, launch files, configs, maps, meshes, and RViz configs
   - Based on the MOGI-ROS reference implementation

2. **bme_ros2_navigation_py** (ament_python)
   - Python utility scripts
   - Contains helper nodes for SLAM and navigation

### Previous Package (obstacle_bot)

The old `obstacle_bot` package has been preserved in the repository for reference but is not the primary package anymore. Key files from `obstacle_bot`:
- `obstacle_bot/urdf/robot.urdf` - Simple 4-wheel robot
- `obstacle_bot/worlds/house.sdf` - House world
- `obstacle_bot/obstacle_bot/avoidance.py` - Simple obstacle avoidance
- `obstacle_bot/obstacle_bot/tf_broadcaster.py` - TF broadcaster

These files remain available but the main focus is now on the `bme_ros2_navigation` packages.

## Key Improvements

### 1. Complete Navigation Stack
- Full SLAM Toolbox integration (mapping + localization modes)
- AMCL localization support
- Nav2 navigation stack with path planning
- Waypoint navigation capabilities
- Exploration support

### 2. Better Robot Model
- Professional robot model (mogi_bot) with meshes
- Improved sensor configuration
- EKF sensor fusion for better odometry
- IMU integration

### 3. Multiple Worlds
- Empty world for testing
- Home world with furniture and walls

### 4. Comprehensive Launch Files
- `spawn_robot.launch.py` - Basic simulation
- `mapping.launch.py` - SLAM mapping
- `localization.launch.py` - AMCL localization
- `localization_slam_toolbox.launch.py` - SLAM Toolbox localization
- `navigation.launch.py` - Full navigation with AMCL
- `navigation_with_slam.launch.py` - Navigation while mapping
- `world.launch.py` - Gazebo world only
- `check_urdf.launch.py` - URDF validation

### 5. Pre-configured RViz Layouts
- `urdf.rviz` - Robot model visualization
- `rviz.rviz` - Basic simulation view
- `mapping.rviz` - SLAM mapping view
- `localization.rviz` - Localization view
- `navigation.rviz` - Navigation view

### 6. Configuration Files
- `ekf.yaml` - Extended Kalman Filter for sensor fusion
- `gz_bridge.yaml` - Gazebo-ROS bridge topics
- `slam_toolbox_mapping.yaml` - SLAM mapping parameters
- `slam_toolbox_localization.yaml` - SLAM localization parameters
- `amcl_localization.yaml` - AMCL parameters
- `navigation.yaml` - Nav2 stack parameters
- `waypoints.yaml` - Example waypoint definitions

## Migration Path for Users

If you were using the old `obstacle_bot` package:

### Option 1: Use New Structure (Recommended)
Follow the new README instructions to use `bme_ros2_navigation`:

```bash
cd ~/ros2_ws
colcon build --packages-select bme_ros2_navigation bme_ros2_navigation_py
source install/setup.bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

### Option 2: Keep Using obstacle_bot
The old package is still in the repository:

```bash
cd ~/ros2_ws
colcon build --packages-select obstacle_bot
source install/setup.bash
# Use old launch files as before
```

## Functionality Mapping

| Old (obstacle_bot) | New (bme_ros2_navigation) |
|-------------------|---------------------------|
| `launch_sim.launch.py` | `spawn_robot.launch.py` |
| `complete_navigation.launch.py` | `navigation_with_slam.launch.py` |
| `simple_avoidance.launch.py` | Not directly equivalent (use Nav2 instead) |
| `gazebo.launch.py` | `world.launch.py` + `spawn_robot.launch.py` |
| `robot.urdf` | `mogi_bot.urdf` (more advanced) |
| `house.sdf` | `home.sdf` (similar) |
| `avoidance.py` | Not needed (Nav2 handles this) |
| `tf_broadcaster.py` | Handled by EKF and robot_state_publisher |

## Dependencies

New dependencies required:

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization \
                 ros-${ROS_DISTRO}-interactive-marker-twist-server
```

All other dependencies remain the same (navigation2, slam-toolbox, ros-gz, etc.)

## Benefits of Migration

1. **Industry-standard approach**: Based on professional ROS2 navigation practices
2. **Better performance**: EKF sensor fusion, improved SLAM
3. **More features**: Multiple navigation modes, waypoint following, exploration
4. **Better maintained**: Based on actively maintained MOGI-ROS course materials
5. **Extensive documentation**: Complete tutorials and examples
6. **Professional robot model**: Proper meshes and realistic sensors

## Backward Compatibility

The old `obstacle_bot` package remains functional and can coexist with the new packages. You can build both:

```bash
colcon build --packages-select obstacle_bot bme_ros2_navigation bme_ros2_navigation_py
```

However, they use different robot models and cannot be used simultaneously in the same simulation.

## Questions or Issues?

- Review the new README.md for complete usage instructions
- Check the reference repository: https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation
- Old obstacle_bot README preserved as: README_old_obstacle_bot.md
