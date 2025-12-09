#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# System Verification Script for Isaac ROS 3.2 Navigation Stack
#
# Usage:
#   ./verify_system.sh          - Pre-launch checks
#   ./verify_system.sh verify   - Post-launch verification
#   ./verify_system.sh topics   - List all relevant topics
#   ./verify_system.sh tf       - Generate TF tree PDF

set -e

echo "========================================="
echo "Isaac ROS 3.2 Navigation System Verification"
echo "========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check function
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK]${NC}"
        return 0
    else
        echo -e "${RED}[FAIL]${NC}"
        return 1
    fi
}

# Check function with warning
check_status_warn() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK]${NC}"
        return 0
    else
        echo -e "${YELLOW}[WARN]${NC}"
        return 1
    fi
}

# =========================================
# PRE-LAUNCH CHECKS
# =========================================
echo -e "${BLUE}=== Pre-Launch Environment Checks ===${NC}"
echo ""

echo "1. Checking ROS2 environment..."
echo -n "   ROS_DISTRO: "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}NOT SET${NC}"
    echo "   Please source ROS2: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}$ROS_DISTRO${NC}"
fi

echo -n "   Workspace sourced: "
if ros2 pkg list 2>/dev/null | grep -q "your_robot_bringup"; then
    echo -e "${GREEN}yes${NC}"
else
    echo -e "${YELLOW}no (run: source install/setup.bash)${NC}"
fi

echo ""
echo "2. Checking Docker environment..."
echo -n "   Running in Docker: "
if [ -f /.dockerenv ]; then
    echo -e "${GREEN}yes${NC}"
    echo -n "   NVIDIA Container: "
    if nvidia-smi > /dev/null 2>&1; then
        echo -e "${GREEN}yes${NC}"
        echo "   GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
    else
        echo -e "${RED}no (Isaac ROS requires NVIDIA GPU)${NC}"
    fi
else
    echo -e "${YELLOW}no${NC}"
    echo -n "   NVIDIA GPU available: "
    if nvidia-smi > /dev/null 2>&1; then
        echo -e "${GREEN}yes${NC}"
    else
        echo -e "${RED}no${NC}"
    fi
fi

echo ""
echo "3. Checking RealSense camera..."
echo -n "   USB device: "
if lsusb 2>/dev/null | grep -i "Intel.*RealSense\|8086:0b3a\|8086:0b5c" > /dev/null; then
    echo -e "${GREEN}detected${NC}"
else
    echo -e "${YELLOW}not found (check USB connection)${NC}"
fi

echo -n "   realsense2_camera package: "
ros2 pkg list 2>/dev/null | grep -q "realsense2_camera"
check_status

echo ""
echo "4. Checking Isaac ROS 3.2 packages..."
packages=("isaac_ros_visual_slam" "nvblox_ros")
for pkg in "${packages[@]}"; do
    echo -n "   $pkg: "
    ros2 pkg list 2>/dev/null | grep -q "$pkg"
    check_status
done

echo ""
echo "5. Checking Navigation packages..."
packages=("nav2_bringup" "nav2_bt_navigator" "robot_localization")
for pkg in "${packages[@]}"; do
    echo -n "   $pkg: "
    ros2 pkg list 2>/dev/null | grep -q "$pkg"
    check_status
done

# =========================================
# POST-LAUNCH VERIFICATION
# =========================================
if [ "$1" == "verify" ]; then
    echo ""
    echo -e "${BLUE}=== Post-Launch Verification ===${NC}"
    echo ""

    echo "6. Verifying running nodes..."
    echo "   Core nodes:"
    nodes=("realsense2_camera_node" "visual_slam_node" "nvblox_node")
    for node in "${nodes[@]}"; do
        echo -n "     $node: "
        ros2 node list 2>/dev/null | grep -q "$node"
        check_status_warn || echo "       (may not be running yet)"
    done

    echo "   Nav2 nodes:"
    nav2_nodes=("bt_navigator" "controller_server" "planner_server")
    for node in "${nav2_nodes[@]}"; do
        echo -n "     $node: "
        ros2 node list 2>/dev/null | grep -q "$node"
        check_status_warn || echo "       (Nav2 may be disabled)"
    done

    echo ""
    echo "7. Verifying camera topics..."
    topics=(
        "/camera/depth/image_rect_raw"
        "/camera/color/image_raw"
        "/camera/imu"
        "/camera/infra1/image_rect_raw"
        "/camera/infra2/image_rect_raw"
    )
    for topic in "${topics[@]}"; do
        echo -n "   $topic: "
        ros2 topic list 2>/dev/null | grep -q "$topic"
        check_status
    done

    echo ""
    echo "8. Verifying VSLAM topics..."
    topics=(
        "/visual_slam/tracking/odometry"
        "/visual_slam/status"
        "/visual_slam/vis/landmarks_cloud"
        "/visual_slam/vis/slam_path"
    )
    for topic in "${topics[@]}"; do
        echo -n "   $topic: "
        ros2 topic list 2>/dev/null | grep -q "$topic"
        check_status_warn
    done

    echo ""
    echo "9. Verifying Nvblox topics..."
    topics=(
        "/nvblox_node/mesh_marker"
        "/nvblox_node/static_map_slice"
    )
    for topic in "${topics[@]}"; do
        echo -n "   $topic: "
        ros2 topic list 2>/dev/null | grep -q "$topic"
        check_status_warn || echo "       (Nvblox initializing...)"
    done

    echo ""
    echo "10. Verifying Scan topic..."
    echo -n "    /scan: "
    ros2 topic list 2>/dev/null | grep -q "^/scan$"
    check_status

    echo ""
    echo "11. Verifying TF tree..."
    echo -n "    map -> odom: "
    timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation" && echo -e "${GREEN}[OK]${NC}" || echo -e "${YELLOW}[WAITING] (VSLAM initializing)${NC}"

    echo -n "    odom -> base_link: "
    timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q "Translation" && echo -e "${GREEN}[OK]${NC}" || echo -e "${YELLOW}[WAITING]${NC}"

    echo -n "    base_link -> camera_link: "
    timeout 3 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | grep -q "Translation" && echo -e "${GREEN}[OK]${NC}" || echo -e "${RED}[FAIL] (Static TF not published)${NC}"

    echo ""
    echo "12. Checking data rates (3 seconds each)..."
    echo "    Camera depth (target: ~30 Hz):"
    timeout 4 ros2 topic hz /camera/depth/image_rect_raw 2>&1 | grep "average rate" | head -1 || echo "      No data yet"

    echo "    Camera IMU (target: ~200 Hz):"
    timeout 4 ros2 topic hz /camera/imu 2>&1 | grep "average rate" | head -1 || echo "      No data yet"

    echo "    VSLAM odometry (target: ~30 Hz):"
    timeout 4 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | grep "average rate" | head -1 || echo "      No data (VSLAM not tracking)"

    echo ""
    echo "13. VSLAM tracking status..."
    echo -n "    vo_state: "
    status=$(timeout 2 ros2 topic echo /visual_slam/status --once 2>/dev/null | grep "vo_state" | head -1)
    if [ -n "$status" ]; then
        echo -e "${GREEN}$status${NC}"
    else
        echo -e "${YELLOW}No status yet (waiting for tracking)${NC}"
    fi

# =========================================
# TOPICS LIST
# =========================================
elif [ "$1" == "topics" ]; then
    echo ""
    echo -e "${BLUE}=== All Navigation-Related Topics ===${NC}"
    echo ""
    echo "Camera topics:"
    ros2 topic list 2>/dev/null | grep "^/camera" || echo "  (none)"
    echo ""
    echo "VSLAM topics:"
    ros2 topic list 2>/dev/null | grep "visual_slam" || echo "  (none)"
    echo ""
    echo "Nvblox topics:"
    ros2 topic list 2>/dev/null | grep "nvblox" || echo "  (none)"
    echo ""
    echo "Nav2 topics:"
    ros2 topic list 2>/dev/null | grep -E "plan|costmap|cmd_vel|goal" || echo "  (none)"
    echo ""
    echo "Other navigation topics:"
    ros2 topic list 2>/dev/null | grep -E "^/scan$|^/odom$|^/tf" || echo "  (none)"

# =========================================
# TF TREE
# =========================================
elif [ "$1" == "tf" ]; then
    echo ""
    echo -e "${BLUE}=== Generating TF Tree ===${NC}"
    echo ""
    echo "Saving TF tree to frames.pdf..."
    ros2 run tf2_tools view_frames
    echo ""
    echo "TF tree saved! Open frames.pdf to view."

else
    # Default: show usage
    echo ""
    echo -e "${BLUE}=== Quick Start Guide ===${NC}"
    echo ""
    echo "1. Build the package:"
    echo "   cd /workspaces/isaac_ros-dev"
    echo "   colcon build --packages-select your_robot_bringup"
    echo "   source install/setup.bash"
    echo ""
    echo "2. Launch the navigation stack:"
    echo "   ros2 launch your_robot_bringup master_bringup.launch.py enable_rviz:=true"
    echo ""
    echo "3. Run post-launch verification:"
    echo "   ./verify_system.sh verify"
    echo ""
    echo "4. Useful commands:"
    echo "   ./verify_system.sh topics  - List all relevant topics"
    echo "   ./verify_system.sh tf      - Generate TF tree PDF"
    echo ""
fi

echo ""
echo "========================================="
echo "Verification complete!"
echo ""
echo "Launch commands:"
echo "  Full stack:      ros2 launch your_robot_bringup master_bringup.launch.py"
echo "  With RViz:       ros2 launch your_robot_bringup master_bringup.launch.py enable_rviz:=true"
echo "  Perception only: ros2 launch your_robot_bringup master_bringup.launch.py enable_nav2:=false"
echo "  Odometry only:   ros2 launch your_robot_bringup master_bringup.launch.py enable_slam:=false"
echo "========================================="
