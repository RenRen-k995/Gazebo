# Troubleshooting Guide

Common issues and solutions for the ROS2 Navigation project with Gazebo Harmonic.

## Installation Issues

### ROS2 Not Found
**Solution:** `source /opt/ros/${ROS_DISTRO}/setup.bash`

### Package Dependencies Missing
**Solution:** Run the installation command from README.md

### Gazebo Harmonic Not Installed
**Solution:** Follow https://gazebosim.org/docs/harmonic/install

## Build Issues

### Package Not Found During Build
**Solution:**
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select bme_ros2_navigation bme_ros2_navigation_py
source install/setup.bash
```

## Navigation Issues

### No Transform from Map to Base_Link
**Solution:**
- For AMCL: Set initial pose using "2D Pose Estimate" in RViz
- For SLAM: Wait 5-10 seconds for initialization
- Check TF tree: `ros2 run tf2_tools view_frames`

### Robot Won't Accept Goals
**Solution:**
```bash
ros2 lifecycle list
ros2 lifecycle get /controller_server
```

## SLAM Issues

### Map Not Building
**Solution:**
```bash
ros2 topic echo /scan --no-arr
ros2 topic hz /scan
ros2 node info /slam_toolbox
```

## More Help

See [README.md](README.md) for complete documentation.
