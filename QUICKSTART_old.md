# Quick Start Guide

## Option 1: Complete Navigation Stack (Recommended)

This launches Gazebo, SLAM, Nav2, and RViz for full autonomous navigation with mapping.

```bash
# Source ROS2 workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros2_ws
source install/setup.bash

# Launch everything
ros2 launch obstacle_bot complete_navigation.launch.py
```

### What to expect:
1. **Gazebo** opens with a house environment
2. **Robot** spawns in the center
3. **RViz** opens showing the robot, laser scans, and map
4. **SLAM** starts building a map as the robot moves
5. **Nav2** enables autonomous navigation

### How to use:
1. Wait 10-15 seconds for all nodes to initialize
2. In RViz:
   - Click "2D Pose Estimate" (top toolbar)
   - Click and drag on the map where the robot is to set initial pose
   - Click "2D Goal Pose" (top toolbar)
   - Click and drag to set a navigation goal
3. Watch the robot autonomously navigate!

## Option 2: Simple Obstacle Avoidance

For testing basic reactive obstacle avoidance without Nav2/SLAM.

```bash
ros2 launch obstacle_bot simple_avoidance.launch.py
```

The robot will:
- Drive forward when path is clear
- Turn away from obstacles detected by lidar
- Choose the direction with more free space

## Option 3: Manual Control Only

Just Gazebo simulation without autonomous navigation.

```bash
# Terminal 1: Launch Gazebo
ros2 launch obstacle_bot gazebo.launch.py

# Terminal 2: Send manual commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Troubleshooting

### "No module named 'ament_index_python'"
You need to install ROS2 and source it:
```bash
source /opt/ros/humble/setup.bash  # or jazzy
```

### "Package 'obstacle_bot' not found"
Build the package first:
```bash
cd ~/ros2_ws
colcon build --packages-select obstacle_bot
source install/setup.bash
```

### Robot not visible in Gazebo
- Check if robot spawned: `ros2 topic list | grep robot_description`
- Try restarting the launch file

### No map appearing in RViz
- SLAM needs motion to build a map
- Either:
  - Use Nav2 to send a goal pose
  - Manually drive: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
  - Run obstacle avoidance in another terminal

### Clock/TF warnings
These are normal during startup. Wait 10-15 seconds for all nodes to synchronize.

## Testing Navigation

### 1. Build a map first:
```bash
ros2 launch obstacle_bot complete_navigation.launch.py
```

Drive around (using Nav2 goals or teleop) to build a complete map.

### 2. Save the map (optional):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 3. Navigate:
Set goal poses in RViz and watch autonomous navigation!

## Key Topics

Monitor these topics to debug issues:

```bash
# Laser scan data
ros2 topic echo /scan

# Odometry
ros2 topic echo /odom

# Velocity commands
ros2 topic echo /cmd_vel

# Map
ros2 topic echo /map

# TF tree
ros2 run tf2_tools view_frames
```

## Performance Tips

- **Slow simulation?** Reduce RViz display rates or disable some visualizations
- **Navigation failing?** Increase costmap obstacle inflation radius
- **Map not accurate?** Adjust SLAM parameters in `slam_toolbox_params.yaml`
