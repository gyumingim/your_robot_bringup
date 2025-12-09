# CLAUDE.md - AI Assistant Guide for your_robot_bringup

## Repository Overview

This is a **ROS2 autonomous mobile robot navigation system** integrating NVIDIA Isaac ROS technology with Intel RealSense cameras for vision-based SLAM and 3D mapping.

**Project Type:** ROS2 Humble bringup/integration package
**Primary Language:** Python (launch files), YAML (configuration)
**License:** Apache-2.0
**Version:** 1.0.0

### Key Technologies
- **Hardware:** Intel RealSense D435i (stereo IR cameras + depth + IMU)
- **Visual SLAM:** NVIDIA Isaac ROS Visual SLAM
- **3D Mapping:** NVIDIA Nvblox (real-time reconstruction)
- **Navigation:** Nav2 (Navigation2 stack)
- **Localization:** Robot Localization EKF (optional)
- **Build System:** ament_cmake (ROS2)

---

## Repository Structure

```
your_robot_bringup/                    # Git repository root
├── .git/                              # Git metadata
├── CLAUDE.md                          # This file (AI assistant guide)
│
├── your_robot_bringup/                # Main ROS2 package
│   ├── CMakeLists.txt                 # Build configuration
│   ├── package.xml                    # Package metadata & dependencies
│   ├── README.md                      # User-facing documentation
│   ├── QUICK_START.md                 # Quick start guide
│   ├── verify_system.sh               # System verification script
│   │
│   ├── launch/                        # Launch files (organized by layer)
│   │   ├── master_bringup.launch.py   # Main entry point - launches entire stack
│   │   ├── sensors/                   # Sensor layer
│   │   │   ├── realsense.launch.py
│   │   │   └── depthimage_to_laserscan.launch.py
│   │   ├── perception/                # Perception layer
│   │   │   ├── vslam.launch.py
│   │   │   └── nvblox.launch.py
│   │   ├── localization/              # Localization layer
│   │   │   └── robot_localization.launch.py
│   │   ├── navigation/                # Navigation layer
│   │   │   └── nav2.launch.py
│   │   └── visualization/             # Visualization layer
│   │       └── rviz2.launch.py
│   │
│   ├── config/                        # Configuration files
│   │   ├── ekf.yaml                   # Robot Localization EKF config
│   │   └── nav2_params.yaml           # Nav2 navigation parameters
│   │
│   └── rviz/                          # Visualization configurations
│       └── isaac_navigation.rviz      # Custom RViz config
│
└── [dependency_dirs]/                 # Empty placeholder directories
    ├── isaac_ros_common/
    ├── isaac_ros_nitros/
    ├── isaac_ros_nvblox/
    ├── isaac_ros_visual_slam/
    ├── magic_enum/
    ├── navigation2/
    ├── realsense-ros/
    └── robot_localization/
```

### Important Notes on Structure
1. **No custom code:** This package contains NO custom Python nodes or C++ executables - it's purely a launch/configuration integration package
2. **Empty dependency dirs:** The placeholder directories indicate external packages should be installed system-wide or as workspace dependencies
3. **Layered architecture:** Launch files are organized by functional layer (sensors → perception → localization → navigation → visualization)

---

## System Architecture

### Data Flow

```
RealSense D435i Camera
│
├─ Stereo IR (infra1/2) ──→ Visual SLAM ──→ TF: map→odom→base_link
│                                        └──→ /visual_slam/tracking/odometry
│
├─ Depth Image ──┬──→ Nvblox ──→ 3D Mesh + Map Slice ──→ Nav2 Global Costmap
│                │
│                └──→ Depth to LaserScan ──→ /scan ──→ Nav2 Local Costmap
│
└─ IMU ──┬──→ Visual SLAM (fusion enabled)
         │
         └──→ [Optional] EKF (robot_localization) - Currently disabled

Nav2 Stack:
    Global Costmap (nvblox_layer + static_layer + inflation)
    Local Costmap (voxel_layer from /scan + inflation)
    ↓
    Planner Server (NavfnPlanner/Dijkstra)
    ↓
    Controller Server (DWB - Dynamic Window Approach)
    ↓
    /cmd_vel (velocity commands to robot base)
```

### TF Tree

```
map (published by VSLAM)
 └── odom (published by VSLAM)
      └── base_link (published by VSLAM)
           └── camera_link (static TF: x=0.05, y=0, z=0.1)
                └── camera_depth_frame (RealSense driver)
                     ├── camera_color_optical_frame
                     ├── camera_depth_optical_frame
                     ├── camera_infra1_optical_frame
                     ├── camera_infra2_optical_frame
                     └── camera_gyro_optical_frame (IMU)
```

### Startup Sequence (Critical!)

The system has **carefully timed initialization** to ensure proper startup:

| Time | Component | Reason |
|------|-----------|--------|
| 0.0s | Static TF (base_link→camera_link) | MUST be published first before any nodes |
| 0.5s | RealSense camera | Small delay to ensure TF is available |
| 2.0s | Visual SLAM | Wait for camera to fully initialize |
| 2.5s | Depth to LaserScan | After camera is ready |
| 3.0s | Nvblox | After VSLAM starts (needs odometry) |
| 4.0s | Robot Localization EKF | Optional, after odometry available |
| 10.0s | Nav2 | Wait for VSLAM to start tracking and publish odom frame |
| 11.0s | RViz2 | After all nodes are running |

**⚠️ CRITICAL:** Do NOT change these timings without understanding the dependencies!

### Key Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/camera/infra1/image_rect_raw` | Image | RealSense | VSLAM | Stereo left |
| `/camera/infra2/image_rect_raw` | Image | RealSense | VSLAM | Stereo right |
| `/camera/depth/image_rect_raw` | Image | RealSense | Nvblox, Depth2Scan | Depth data |
| `/camera/imu` | Imu | RealSense | VSLAM, EKF | IMU 200Hz |
| `/visual_slam/tracking/odometry` | Odometry | VSLAM | Nav2, EKF | Robot pose |
| `/visual_slam/status` | VisualSlamStatus | VSLAM | Monitoring | Tracking status |
| `/scan` | LaserScan | Depth2Scan | Nav2 Local | 2D laser scan |
| `/nvblox_node/mesh` | Mesh | Nvblox | RViz | 3D visualization |
| `/nvblox_node/static_map_slice` | OccupancyGrid | Nvblox | Nav2 Global | 2D occupancy |
| `/cmd_vel` | Twist | Nav2 Controller | Robot Base | Velocity cmds |

---

## Development Workflows

### Building the Package

```bash
# From workspace root
cd /path/to/workspace
colcon build --packages-select your_robot_bringup
source install/setup.bash
```

### Launching the System

**Full stack (headless for Docker):**
```bash
ros2 launch your_robot_bringup master_bringup.launch.py
```

**With RViz (for local testing):**
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_rviz:=true
```

**Without Nav2 (testing perception only):**
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_nav2:=false
```

**With EKF enabled:**
```bash
ros2 launch your_robot_bringup master_bringup.launch.py enable_robot_localization:=true
```

### Verification Workflow

**Before launching (check environment & hardware):**
```bash
cd your_robot_bringup/your_robot_bringup
./verify_system.sh
```

**After launching (verify running system):**
```bash
# In another terminal
./verify_system.sh verify
```

This checks:
- ROS2 environment
- RealSense USB connection
- Package availability
- Running nodes
- Topic availability
- TF tree correctness
- Data rates (camera ~30Hz, IMU ~200Hz)
- VSLAM tracking status

### Common Development Tasks

#### Modifying Camera Position
Edit `launch/master_bringup.launch.py` line 91-93:
```python
arguments=[
    '0.05', '0.0', '0.1',  # x, y, z relative to base_link
    '0.0', '0.0', '0.0',   # roll, pitch, yaw
    'base_link', 'camera_link'
]
```

#### Tuning Navigation Parameters
Edit `config/nav2_params.yaml`:
- **Robot dimensions:** `controller_server.robot_radius: 0.22`
- **Max velocities:** `controller_server.DWB.max_vel_x: 0.26`
- **Acceleration limits:** `controller_server.DWB.acc_lim_x: 2.5`
- **Goal tolerance:** `controller_server.DWB.xy_goal_tolerance: 0.25`

#### Adjusting EKF Sensor Fusion
Edit `config/ekf.yaml`:
- **Frequency:** `ekf_filter_node.ros__parameters.frequency: 50.0`
- **Process noise:** Covariance matrices for each state variable
- **Sensor inputs:** Enable/disable position/velocity/orientation for each sensor

#### Adding a New Launch File
1. Create file in appropriate `launch/` subdirectory
2. Follow existing pattern with proper imports
3. Add to `CMakeLists.txt` install directive if in new subdirectory
4. Include in `master_bringup.launch.py` if part of main stack

---

## Key Conventions and Patterns

### Launch File Conventions

1. **SPDX License Header:** All files start with `# SPDX-License-Identifier: Apache-2.0`
2. **Docstring:** Explain purpose and critical configuration at top of file
3. **Argument Declaration:** Use `DeclareLaunchArgument` with descriptions
4. **Timing:** Use `TimerAction` for sequential startup when needed
5. **Conditional Launch:** Use `IfCondition` for optional components
6. **Path Resolution:** Use `FindPackageShare` + `PathJoinSubstitution` for portability

### Configuration Conventions

1. **YAML Format:** All config files use YAML
2. **Namespacing:** Namespace parameters by node name
3. **ROS Parameters:** Follow ROS2 parameter naming conventions (lowercase, underscores)
4. **Comments:** Explain non-obvious parameter choices
5. **Units:** Always document units in comments (m, rad, Hz, etc.)

### Frame Naming Conventions

- `map`: Global fixed frame (VSLAM origin)
- `odom`: Odometry frame (continuous, may drift)
- `base_link`: Robot center/base
- `camera_link`: Physical camera mount
- `camera_*_frame`: Camera-specific frames (RealSense convention)
- `camera_*_optical_frame`: Optical frames (Z forward, X right, Y down)

### Git Commit Conventions

Based on recent history, commits use Korean messages:
- "통합 성공, 이후 수치 조저ㅏㄹ" (Integration success, then parameter tuning)
- "수정본" (Revised version)
- "없는 args 삭제" (Remove non-existent args)
- "my 삭제" (Delete my)

**Recommended for AI assistants:**
- Use clear, descriptive messages in Korean or English as appropriate
- Reference specific components changed (e.g., "Nav2 params tuning", "VSLAM timing fix")
- Use imperative mood for English commits ("Fix timing", not "Fixed timing")

---

## Critical Considerations for AI Assistants

### 🚨 CRITICAL RULES - NEVER VIOLATE THESE

1. **Static TF First:** The `base_link → camera_link` static transform MUST be published before any other nodes start. It has NO delay (period=0.0) in master_bringup.launch.py.

2. **Depth Emitter Disabled:** RealSense depth emitter MUST be disabled (set to 0) for stereo VSLAM to work. The infrared pattern interferes with stereo matching.

3. **Timing Dependencies:** The startup timing sequence is critical. Changing delays can cause initialization failures:
   - Camera before VSLAM (needs images)
   - VSLAM before Nvblox (needs odometry)
   - All sensors before Nav2 (needs maps and localization)

4. **No Custom Code:** This package is launch/config ONLY. Do not add Python nodes or C++ code - suggest creating a separate package instead.

5. **Frame Consistency:** All optical frames follow ROS convention (Z forward). Camera is mounted HORIZONTALLY (landscape orientation).

### ⚠️ Important Constraints

1. **CUDA Required:** Isaac ROS components (VSLAM, Nvblox) require NVIDIA GPU with CUDA. Cannot run on CPU-only systems.

2. **Headless by Default:** RViz is disabled by default (`enable_rviz:=false`) for Docker/server deployment. Enable only for local testing.

3. **EKF Optional:** Visual SLAM publishes TF directly (map→odom→base_link), so EKF is NOT needed by default. Enable only if fusing additional sensors.

4. **Ground Constraint:** Enabled by default (`enable_ground_constraint:=true`) for 2D planar navigation. Disable for 3D navigation.

5. **Nvblox Frame:** Uses `odom` as global frame (from VSLAM), not `map`. This prevents map drift issues during SLAM.

### 🔧 Common Pitfalls to Avoid

1. **Don't modify timing without testing:** The delays are carefully tuned. Changes can cause race conditions.

2. **Don't enable depth emitter:** Will break stereo VSLAM (infrared pattern interference).

3. **Don't skip static TF:** All other nodes expect `camera_link` to exist.

4. **Don't use both EKF TF and VSLAM TF:** Will create TF conflicts. Choose one localization method.

5. **Don't assume dependencies are present:** The placeholder directories are empty - packages must be installed separately.

6. **Don't hardcode paths:** Use `FindPackageShare` for portability across installations.

7. **Don't forget namespacing:** Camera topics are namespaced (e.g., `/camera/depth/...`).

### 📝 When Making Changes

**Before modifying launch files:**
1. Understand the current timing sequence
2. Check TF dependencies
3. Review topic remapping
4. Test with `verify_system.sh` after changes

**Before modifying config files:**
1. Read inline comments explaining parameters
2. Understand physical constraints (max velocity, robot size)
3. Keep units consistent
4. Test incrementally (small changes at a time)

**Before committing:**
1. Run `colcon build` to check for syntax errors
2. Launch the system and verify with `verify_system.sh verify`
3. Check that all expected topics are publishing
4. Verify TF tree with `ros2 run tf2_tools view_frames`

---

## File Reference Guide

### Launch Files

| File | Lines | Purpose | Key Parameters |
|------|-------|---------|----------------|
| `launch/master_bringup.launch.py` | 265 | Main entry point | enable_rviz, enable_nav2, enable_robot_localization |
| `launch/sensors/realsense.launch.py` | ~100 | RealSense driver | camera_name, enable_depth, enable_color |
| `launch/sensors/depthimage_to_laserscan.launch.py` | ~50 | Depth→Scan | scan_height, range_min, range_max |
| `launch/perception/vslam.launch.py` | ~80 | Visual SLAM | enable_imu_fusion, enable_ground_constraint |
| `launch/perception/nvblox.launch.py` | ~60 | 3D reconstruction | global_frame, voxel_size |
| `launch/localization/robot_localization.launch.py` | ~40 | EKF fusion | use_sim_time |
| `launch/navigation/nav2.launch.py` | ~50 | Nav2 stack | params_file, autostart |
| `launch/visualization/rviz2.launch.py` | ~30 | RViz2 | config file path |

### Configuration Files

| File | Lines | Purpose | Key Sections |
|------|-------|---------|--------------|
| `config/ekf.yaml` | ~150 | EKF parameters | frequency, two_d_mode, odom0, imu0 |
| `config/nav2_params.yaml` | ~600 | Nav2 parameters | controller, planner, costmaps, behaviors |

### Build Files

| File | Purpose | Key Sections |
|------|---------|--------------|
| `CMakeLists.txt` | Build configuration | install() directives for launch/config/rviz |
| `package.xml` | Package metadata | exec_depend for all runtime dependencies |

### Documentation

| File | Purpose | Audience |
|------|---------|----------|
| `README.md` | Quick overview | End users |
| `QUICK_START.md` | Getting started | End users |
| `CLAUDE.md` | Comprehensive guide | AI assistants |
| `verify_system.sh` | System verification | Developers/operators |

---

## Debugging and Troubleshooting

### Diagnostic Commands

**Check running nodes:**
```bash
ros2 node list
```

**Check active topics:**
```bash
ros2 topic list
```

**Monitor topic data rate:**
```bash
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /visual_slam/tracking/odometry
```

**View single message:**
```bash
ros2 topic echo /visual_slam/status --once
```

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames  # Generates frames.pdf
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo base_link camera_link
```

**Monitor node output:**
```bash
ros2 node info /visual_slam_node
ros2 topic echo /rosout | grep ERROR
```

### Common Issues and Solutions

**Issue: VSLAM not tracking**
- Check: `/visual_slam/status` topic → `vo_state` should be tracking
- Solution: Ensure sufficient visual features (textured environment)
- Solution: Check camera is outputting stereo images
- Solution: Verify depth emitter is disabled

**Issue: Nav2 not receiving costmap**
- Check: `/nvblox_node/static_map_slice` is publishing
- Solution: Ensure Nvblox initialized (needs depth + odometry)
- Solution: Check global_frame matches VSLAM odom frame

**Issue: TF lookup failures**
- Check: `ros2 run tf2_tools view_frames` for missing transforms
- Solution: Verify static TF publisher is running
- Solution: Check VSLAM is tracking (publishes map→odom→base_link)

**Issue: Camera not detected**
- Check: `lsusb | grep Intel` shows RealSense device
- Solution: Check USB connection, try different port
- Solution: Verify realsense2_camera package installed

**Issue: Navigation commands ignored**
- Check: `/cmd_vel` topic has subscribers
- Solution: Verify Nav2 stack is running and lifecycle nodes are active
- Solution: Check costmaps are receiving data (not all unknown)

**Issue: High latency or dropped frames**
- Check: Topic data rates with `ros2 topic hz`
- Solution: Reduce image resolution in realsense.launch.py
- Solution: Ensure CUDA GPU available for Isaac ROS nodes
- Solution: Close RViz if running on resource-constrained system

---

## Testing Procedures

### Manual Testing Workflow

1. **Pre-launch verification:**
   ```bash
   cd your_robot_bringup/your_robot_bringup
   ./verify_system.sh
   ```

2. **Launch system:**
   ```bash
   ros2 launch your_robot_bringup master_bringup.launch.py
   ```

3. **Post-launch verification:**
   ```bash
   # In new terminal
   ./verify_system.sh verify
   ```

4. **Functional testing:**
   - Slowly move camera and verify VSLAM tracks
   - Check Nvblox builds map in RViz
   - Send navigation goal and verify planning
   - Check collision avoidance with obstacles

### Automated Testing

Currently no automated tests are configured. The package uses `ament_lint` for code quality:

```bash
# From workspace root
colcon test --packages-select your_robot_bringup
colcon test-result --verbose
```

### Performance Benchmarks

Expected data rates (from `verify_system.sh`):
- **Camera depth:** ~30 Hz
- **Camera IMU:** ~200 Hz
- **VSLAM odometry:** ~30 Hz (when tracking)
- **Scan:** ~30 Hz
- **Nvblox mesh:** ~1-5 Hz (depends on map complexity)

---

## Git Workflow

### Current Branch
Based on git status, development occurs on feature branches:
- Branch pattern: `claude/claude-md-miy4mafbrqsuswwg-01SEZ16yKD86weuPmUcaLLZj`
- Main branch reference needed (not specified in current context)

### Commit Guidelines

1. **Test before committing:** Always verify system launches successfully
2. **Descriptive messages:** Explain WHAT changed and WHY
3. **Atomic commits:** One logical change per commit
4. **No secrets:** Never commit credentials, API keys, or sensitive data

### Recommended Workflow

```bash
# Check status
git status

# Stage specific files
git add path/to/file

# Commit with clear message
git commit -m "Fix VSLAM timing to prevent initialization race condition"

# Push to feature branch
git push -u origin claude/your-session-id
```

---

## Dependencies and External Packages

### Required ROS2 Packages

**Core ROS2 (apt installable):**
- `robot_state_publisher`
- `joint_state_publisher`
- `tf2_ros`
- `tf2_tools`

**Sensors:**
- `realsense2_camera` - Install from Intel RealSense ROS wrapper
- `realsense2_description`
- `depthimage_to_laserscan`

**NVIDIA Isaac ROS (requires Isaac ROS setup):**
- `isaac_ros_visual_slam`
- `isaac_ros_nvblox`
- `nvblox_ros`
- `nvblox_msgs`
- `isaac_ros_common`
- `isaac_ros_nitros`

**Navigation:**
- `navigation2` (meta-package)
- `nav2_bringup`
- `nav2_common`
- `nav2_msgs`

**Localization:**
- `robot_localization`

**Visualization:**
- `rviz2`
- `rviz_common`
- `rviz_default_plugins`

### Installation Notes

1. **Isaac ROS packages** require Docker container or manual CUDA installation
2. **RealSense packages** require librealsense2 SDK installed
3. Empty dependency directories suggest workspace-based installation or system packages

### Checking Dependencies

```bash
# List installed ROS2 packages
ros2 pkg list

# Check specific package
ros2 pkg prefix isaac_ros_visual_slam

# Verify with rosdep
rosdep install --from-paths src --ignore-src -r -y
```

---

## AI Assistant Quick Reference

### When User Asks To...

**"Launch the robot"**
→ Provide: `ros2 launch your_robot_bringup master_bringup.launch.py`

**"Debug VSLAM issues"**
→ Check: `/visual_slam/status` topic, verify stereo images publishing, ensure depth emitter disabled

**"Tune navigation"**
→ Edit: `config/nav2_params.yaml` (velocities, costmaps, planners)

**"Change camera position"**
→ Edit: `launch/master_bringup.launch.py` line 91-93 (static TF arguments)

**"Add new sensor"**
→ Create: New launch file in `launch/sensors/`, add to `master_bringup.launch.py` with appropriate timing

**"Enable RViz"**
→ Provide: `ros2 launch your_robot_bringup master_bringup.launch.py enable_rviz:=true`

**"Why is [component] not working?"**
→ Run: `./verify_system.sh verify` and analyze output

**"Visualize TF tree"**
→ Provide: `ros2 run tf2_tools view_frames`

### Key File Locations (Quick Access)

- Main launch: `/home/user/your_robot_bringup/your_robot_bringup/launch/master_bringup.launch.py`
- Nav2 config: `/home/user/your_robot_bringup/your_robot_bringup/config/nav2_params.yaml`
- EKF config: `/home/user/your_robot_bringup/your_robot_bringup/config/ekf.yaml`
- Verification: `/home/user/your_robot_bringup/your_robot_bringup/verify_system.sh`
- Build config: `/home/user/your_robot_bringup/your_robot_bringup/CMakeLists.txt`
- Dependencies: `/home/user/your_robot_bringup/your_robot_bringup/package.xml`

### Critical Values to Remember

- **Camera offset:** x=0.05, y=0.0, z=0.1 (from base_link)
- **Robot radius:** 0.22m
- **Max linear velocity:** 0.26 m/s
- **Max angular velocity:** 1.0 rad/s
- **Nvblox voxel size:** 0.05m (5cm)
- **ESDF slice height:** 0.5m
- **EKF frequency:** 50Hz
- **Nav2 controller frequency:** 20Hz

---

## Additional Resources

### ROS2 Documentation
- ROS2 Humble Docs: https://docs.ros.org/en/humble/
- Nav2 Documentation: https://navigation.ros.org/
- TF2 Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html

### NVIDIA Isaac ROS
- Isaac ROS Documentation: https://nvidia-isaac-ros.github.io/
- Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/
- Nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/

### Hardware Documentation
- RealSense D435i: https://www.intelrealsense.com/depth-camera-d435i/
- RealSense ROS Wrapper: https://github.com/IntelRealSense/realsense-ros

### Community Resources
- ROS Discourse: https://discourse.ros.org/
- Nav2 Discussions: https://github.com/ros-planning/navigation2/discussions
- Isaac ROS GitHub: https://github.com/NVIDIA-ISAAC-ROS

---

## Changelog

### 2025-12-09
- Initial CLAUDE.md creation
- Documented complete system architecture
- Added AI assistant guidelines and quick reference
- Included troubleshooting procedures and critical rules

### Future Enhancements
- Add automated test procedures
- Include parameter tuning guidelines
- Add performance optimization tips
- Document recovery procedures for common failures

---

## Contact and Support

**Maintainer:** Your Name <your.email@example.com>
**License:** Apache-2.0
**Repository:** Based on git status, appears to be private development repository

For issues or questions:
1. Check this CLAUDE.md guide first
2. Run `verify_system.sh` for diagnostics
3. Review ROS2/Nav2/Isaac ROS documentation
4. Check topic outputs and TF tree for debugging

---

*This guide is maintained for AI assistants to understand and work with the your_robot_bringup codebase. Last updated: 2025-12-09*
