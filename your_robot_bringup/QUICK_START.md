# Quick Start Guide

Isaac ROS 3.2 Navigation Stack with RealSense D435i + Visual SLAM + Nvblox + Nav2

## Prerequisites

- NVIDIA GPU (RTX 4060 or better)
- Intel RealSense D435i camera
- Isaac ROS 3.2 Docker container
- ROS2 Humble

## 1. Docker Environment Setup

```bash
# Enter Isaac ROS Docker container
cd ~/workspaces/isaac_ros-dev
./scripts/run_dev.sh

# Inside container, navigate to workspace
cd /workspaces/isaac_ros-dev
```

## 2. Build the Package

```bash
# Clone/copy your_robot_bringup to src/
# (if not already there)

# Build
colcon build --packages-select your_robot_bringup

# Source workspace
source install/setup.bash
```

## 3. Verify Environment

```bash
# Run pre-launch checks
cd src/your_robot_bringup/your_robot_bringup
./verify_system.sh
```

## 4. Launch Navigation Stack

### Full Stack (with RViz)
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_rviz:=true
```

### Headless Mode (Docker/SSH)
```bash
ros2 launch your_robot_bringup master_bringup.launch.py
```

### Perception Only (no Nav2)
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_nav2:=false
```

### Odometry Only (no SLAM mapping)
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_slam:=false
```

## 5. Post-Launch Verification

```bash
# In another terminal
./verify_system.sh verify
```

## 6. Common Issues

### VSLAM not tracking
- Move camera slowly with textured surfaces visible
- Check: `ros2 topic echo /visual_slam/status --once`
- Ensure depth emitter is OFF (configured in realsense.launch.py)

### No TF published
- Wait 5-10 seconds for VSLAM initialization
- Check: `ros2 run tf2_tools view_frames`

### Nvblox not building map
- VSLAM must be tracking first
- Check: `ros2 topic echo /nvblox_node/static_map_slice`

### Nav2 not responding
- Ensure costmap is receiving data
- Check: `ros2 topic echo /global_costmap/costmap`

## 7. Useful Commands

```bash
# List all topics
./verify_system.sh topics

# Generate TF tree
./verify_system.sh tf

# Monitor VSLAM
ros2 topic hz /visual_slam/tracking/odometry

# Send navigation goal (RViz)
# Use "2D Goal Pose" tool in RViz
```

## 8. Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_rviz` | false | Launch RViz visualization |
| `enable_nav2` | true | Launch Nav2 navigation stack |
| `enable_slam` | true | Enable SLAM (false = odometry only) |
| `enable_ground_constraint` | true | Constrain to 2D plane |
| `voxel_size` | 0.05 | Nvblox voxel size (meters) |

## 9. Architecture

```
RealSense D435i
├── Stereo IR → Visual SLAM → TF (map→odom→base_link)
├── Depth → Nvblox → 2D Costmap → Nav2 Global Planner
└── Depth → LaserScan → Nav2 Local Planner
```

## 10. Next Steps

- Tune Nav2 parameters: `config/nav2_params.yaml`
- Adjust camera position: `master_bringup.launch.py` (static TF)
- Add Scout Mini: Uncomment Scout Mini section in `master_bringup.launch.py`
