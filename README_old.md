# Gazebo Obstacle Avoidance Robot

A complete ROS2 navigation and obstacle avoidance robot project using Gazebo Harmonic, SLAM Toolbox, and Nav2.

## 📚 Documentation

- **[Quick Start Guide](QUICKSTART.md)** - Get started in minutes
- **[Troubleshooting Guide](TROUBLESHOOTING.md)** - Common issues and solutions
- **[Full Documentation](#usage)** - Detailed usage instructions below

## Features

- **Differential Drive Robot**: 4-wheel robot with lidar sensor
- **SLAM**: Real-time mapping and localization using SLAM Toolbox
- **Navigation**: Autonomous navigation with Nav2 stack
- **Obstacle Avoidance**: Simple reactive obstacle avoidance node
- **Visualization**: Pre-configured RViz2 for monitoring

## Prerequisites

- ROS2 (Humble or Jazzy recommended)
- Gazebo Harmonic
- Nav2
- SLAM Toolbox
- ros_gz (ROS-Gazebo bridge)

## Installation

**Check dependencies first:**
```bash
python3 obstacle_bot/scripts/check_dependencies.py
```

**Install ROS2 dependencies:**
```bash
# Install dependencies
sudo apt update
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-ros-gz \
                 ros-${ROS_DISTRO}-robot-state-publisher

# Clone and build
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --packages-select obstacle_bot
source install/setup.bash
```

## Usage

### Complete Navigation Stack (Recommended)

Launch everything (Gazebo, SLAM, Nav2, RViz):

```bash
ros2 launch obstacle_bot complete_navigation.launch.py
```

This will:
1. Start Gazebo with the house world
2. Spawn the robot
3. Launch SLAM Toolbox for mapping
4. Start Nav2 for navigation
5. Open RViz2 for visualization

### Individual Components

**Gazebo only:**
```bash
ros2 launch obstacle_bot gazebo.launch.py
```

**Simulation with Nav2 (no SLAM):**
```bash
ros2 launch obstacle_bot launch_sim.launch.py
```

**Simple obstacle avoidance:**
```bash
ros2 run obstacle_bot avoidance
```

## Navigation Instructions

1. **Wait for initialization**: Allow 10-15 seconds for all nodes to start
2. **Set initial pose**: In RViz, use "2D Pose Estimate" tool to set robot's starting position
3. **Set navigation goal**: Use "2D Goal Pose" tool to command the robot
4. **Watch navigation**: The robot will plan a path and navigate autonomously

## Project Structure

```
obstacle_bot/
├── config/
│   ├── bridge.yaml              # Gazebo-ROS bridge config
│   ├── nav2_params.yaml         # Nav2 stack parameters
│   ├── slam_toolbox_params.yaml # SLAM configuration
│   └── nav2.rviz                # RViz visualization config
├── launch/
│   ├── complete_navigation.launch.py  # Main launch file (SLAM + Nav2 + RViz)
│   ├── simple_avoidance.launch.py     # Simple reactive avoidance
│   ├── launch_sim.launch.py           # Simulation with Nav2 only
│   ├── gazebo.launch.py               # Gazebo only
│   └── response.launch.py             # Robot visualization
├── obstacle_bot/
│   ├── avoidance.py             # Simple obstacle avoidance node
│   └── tf_broadcaster.py        # TF transform publisher
├── scripts/
│   └── check_dependencies.py    # Verify ROS2 dependencies
├── urdf/
│   └── robot.urdf               # Robot description (4-wheel diff drive + lidar)
└── worlds/
    └── house.sdf                # Gazebo world file
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo Harmonic                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   Physics    │  │ DiffDrive    │  │    Lidar     │          │
│  │   Engine     │  │   Plugin     │  │    Sensor    │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
                              ▲ │
                    ros_gz_bridge
                              │ ▼
┌─────────────────────────────────────────────────────────────────┐
│                            ROS 2                                 │
│                                                                   │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐    │
│  │ tf_broadcaster │  │  SLAM Toolbox  │  │     Nav2       │    │
│  │ (odom->base)   │  │  (map->odom)   │  │  (planning)    │    │
│  └────────────────┘  └────────────────┘  └────────────────┘    │
│                                                                   │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐    │
│  │ robot_state_   │  │   obstacle_    │  │     RViz2      │    │
│  │  publisher     │  │   avoidance    │  │ (visualization)│    │
│  └────────────────┘  └────────────────┘  └────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

## TF Tree Structure

```
map (from SLAM)
 └─ odom (from odometry)
     └─ base_link (from tf_broadcaster)
         ├─ base_footprint (static)
         ├─ lidar_link (fixed)
         └─ wheel_links (from DiffDrive plugin)
```

## Troubleshooting

### No map appearing
- Ensure SLAM Toolbox is running: `ros2 node list | grep slam`
- Check scan topic: `ros2 topic echo /scan`
- Verify TF tree: `ros2 run tf2_tools view_frames`

### Robot not moving
- Check cmd_vel bridge: `ros2 topic echo /cmd_vel`
- Verify odometry: `ros2 topic echo /odom`
- Check navigation status: `ros2 topic echo /navigation_result`

### Clock issues
- Ensure use_sim_time is set to true
- Check clock topic: `ros2 topic hz /clock`

## Development

### Running tests
```bash
colcon test --packages-select obstacle_bot
```

### Code style
```bash
# Python linting
flake8 obstacle_bot/
```

## License

TODO: License declaration

## Maintainer

khoaphd (khoaphd@todo.todo)
