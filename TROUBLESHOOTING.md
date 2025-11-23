# Troubleshooting Guide

## Before You Start

Run the dependency checker:
```bash
python3 obstacle_bot/scripts/check_dependencies.py
```

## Common Issues

### 1. Package Not Found Errors

**Error:**
```
Package 'obstacle_bot' not found
```

**Solution:**
```bash
cd ~/ros2_ws
colcon build --packages-select obstacle_bot
source install/setup.bash
```

### 2. Missing ROS2 Packages

**Error:**
```
Package 'slam_toolbox' not found
Package 'nav2_bringup' not found
```

**Solution:**
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-ros-gz \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui
```

### 3. Gazebo Not Starting

**Error:**
```
Unable to find or download file
```

**Solution:**
Ensure Gazebo Harmonic is installed:
```bash
sudo apt install gz-harmonic
```

### 4. Robot Not Spawning

**Symptoms:**
- Gazebo opens but no robot appears

**Solution:**
1. Check if Gazebo is ready:
   ```bash
   ros2 topic list | grep gz
   ```
2. Try increasing spawn delay in launch file
3. Check robot description:
   ```bash
   ros2 topic echo /robot_description
   ```

### 5. No Laser Scans

**Symptoms:**
- Robot spawns but no laser visualization in RViz
- `/scan` topic has no data

**Solution:**
1. Check if lidar sensor is publishing:
   ```bash
   ros2 topic hz /scan
   ```
2. Verify bridge is running:
   ```bash
   ros2 node list | grep bridge
   ```
3. Check bridge configuration:
   ```bash
   ros2 param dump /parameter_bridge
   ```

### 6. TF Transform Errors

**Error:**
```
No transform from [lidar_link] to [map]
No transform from [base_link] to [map]
No transform from [wheel_link] to [map]
Lookup would require extrapolation into the past
Transform from [frame] to [frame] does not exist
```

**Common Causes:**
1. **SLAM not running**: The `map` frame is published by SLAM Toolbox. If SLAM hasn't started yet (it starts 5 seconds after launch), you'll see these errors initially.
2. **Incomplete TF tree**: Missing transforms in the chain: map -> odom -> base_link -> {lidar_link, wheel_links}
3. **Topic namespacing issues**: Gazebo plugins publishing to model-scoped topics instead of global topics
4. **TF publishing conflicts**: Multiple nodes trying to publish the same transform

**Solution:**
1. **Wait for initialization**: These errors are normal for the first 5-10 seconds after launch. SLAM starts at t=5s and needs time to initialize.

2. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

3. Verify all TF publishers are running:
   ```bash
   ros2 topic echo /tf --once
   ros2 topic echo /odom --once
   ros2 node list | grep -E "(slam|robot_state|tf_broadcaster)"
   ```

4. Ensure use_sim_time is consistent:
   ```bash
   ros2 param get /robot_state_publisher use_sim_time
   ros2 param get /slam_toolbox use_sim_time
   ```

5. Check clock is publishing:
   ```bash
   ros2 topic hz /clock
   ```

6. Verify joint_states are being published:
   ```bash
   ros2 topic echo /joint_states --once
   ```

**If errors persist after 15 seconds:**
- Check that `/joint_states` topic is publishing (should show wheel joint positions)
- Verify `/odom` topic has data (needed for tf_broadcaster)
- Ensure `/scan` topic has laser data (needed for SLAM)
- Check that all topics use global namespace (start with `/`)


### 7. SLAM Not Building Map

**Symptoms:**
- Map topic exists but map is empty
- No map shown in RViz

**Solution:**
1. Robot must move to build map. Either:
   - Send Nav2 goal poses
   - Use teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
   - Run simple avoidance: `ros2 run obstacle_bot avoidance`

2. Check SLAM is running:
   ```bash
   ros2 node list | grep slam
   ros2 topic hz /map
   ```

3. Verify laser scans are reaching SLAM:
   ```bash
   ros2 topic info /scan
   ```

### 8. Navigation Not Working

**Symptoms:**
- Can't set goal pose
- Robot doesn't move to goal
- Nav2 errors

**Solution:**
1. Set initial pose first in RViz (2D Pose Estimate)
2. Wait for map to be built by SLAM
3. Check Nav2 status:
   ```bash
   ros2 node list | grep nav2
   ```
4. Check for costmap issues:
   ```bash
   ros2 topic echo /local_costmap/costmap
   ```

### 9. RViz Crashes or Slow

**Solution:**
1. Reduce visualization load:
   - Disable TF display or reduce marker scale
   - Lower LaserScan point size
   - Reduce path display buffer length

2. Lower update rates in RViz displays

3. If still slow, launch without RViz:
   ```bash
   # Edit complete_navigation.launch.py and comment out rviz node
   ```

### 10. Multiple Node Errors on Shutdown

**Symptoms:**
- Many error messages when pressing Ctrl+C

**Solution:**
This is normal for complex launch files. Give it 5-10 seconds to shut down cleanly.

## Diagnostic Commands

### Check All Topics
```bash
ros2 topic list
```

### Check All Nodes
```bash
ros2 node list
```

### Monitor Node Performance
```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### View Node Graph
```bash
ros2 run rqt_graph rqt_graph
```

## Getting Help

### Enable Debug Logging
Add to your launch file:
```python
import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
```

### Check Logs
```bash
ros2 run rqt_console rqt_console
```

### Report Issues
When reporting issues, include:
1. ROS2 distribution (Humble/Jazzy)
2. Gazebo version
3. Output of `ros2 topic list`
4. Output of `ros2 node list`
5. Error messages with context

## Performance Tuning

### Reduce Gazebo CPU Usage
- Lower physics update rate in world file
- Reduce sensor update rates
- Disable shadows in Gazebo

### Reduce Nav2 CPU Usage
- Increase costmap update intervals in nav2_params.yaml
- Reduce planner frequency
- Lower DWB samples

### Improve SLAM Accuracy
- Adjust SLAM parameters in slam_toolbox_params.yaml
- Increase laser scan resolution
- Drive slower for better matching
