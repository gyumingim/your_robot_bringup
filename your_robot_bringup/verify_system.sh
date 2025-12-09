#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# System Verification Script for Isaac ROS Navigation Stack

set -e

echo "========================================="
echo "Isaac ROS Navigation System Verification"
echo "========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check function
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}"
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}"
        return 1
    fi
}

# Wait for topic
wait_for_topic() {
    local topic=$1
    local timeout=${2:-10}
    echo -n "Waiting for topic $topic..."
    
    for i in $(seq 1 $timeout); do
        ros2 topic list | grep -q "$topic" && check_status && return 0
        sleep 1
        echo -n "."
    done
    
    echo -e "${RED}✗ TIMEOUT${NC}"
    return 1
}

echo "1. Checking ROS2 environment..."
echo -n "   ROS_DISTRO: "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}NOT SET${NC}"
    echo "   Please source ROS2: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}$ROS_DISTRO${NC}"
fi

echo ""
echo "2. Checking RealSense camera..."
echo -n "   USB device: "
lsusb | grep -i "Intel" > /dev/null
check_status

echo -n "   realsense2_camera package: "
ros2 pkg list | grep -q "realsense2_camera"
check_status

echo ""
echo "3. Checking Isaac ROS packages..."
packages=("isaac_ros_visual_slam" "nvblox_ros" "robot_localization" "nav2_bringup")
for pkg in "${packages[@]}"; do
    echo -n "   $pkg: "
    ros2 pkg list | grep -q "$pkg"
    check_status
done

echo ""
echo "4. Checking nodes (after launch)..."
echo "   Run 'ros2 launch your_robot_bringup master_bringup.launch.py'"
echo "   Then run this script again with 'verify' argument"
echo ""

if [ "$1" == "verify" ]; then
    echo "5. Verifying running nodes..."
    
    nodes=("realsense2_camera_node" "visual_slam_node" "nvblox_node" "ekf_filter_node")
    for node in "${nodes[@]}"; do
        echo -n "   $node: "
        ros2 node list | grep -q "$node"
        check_status || echo "     (May be disabled)"
    done
    
    echo ""
    echo "6. Verifying topics..."
    
    # Camera topics
    echo "   Camera topics:"
    topics=(
        "/camera/depth/image_rect_raw"
        "/camera/color/image_raw"
        "/camera/imu"
        "/camera/infra1/image_rect_raw"
        "/camera/infra2/image_rect_raw"
    )
    for topic in "${topics[@]}"; do
        echo -n "     $topic: "
        ros2 topic list | grep -q "$topic"
        check_status
    done
    
    # VSLAM topics
    echo "   VSLAM topics:"
    topics=(
        "/visual_slam/tracking/odometry"
        "/visual_slam/tracking/vo_pose"
        "/visual_slam/status"
    )
    for topic in "${topics[@]}"; do
        echo -n "     $topic: "
        ros2 topic list | grep -q "$topic"
        check_status
    done
    
    # Nvblox topics
    echo "   Nvblox topics:"
    topics=(
        "/nvblox_node/mesh"
        "/nvblox_node/static_map_slice"
    )
    for topic in "${topics[@]}"; do
        echo -n "     $topic: "
        ros2 topic list | grep -q "$topic"
        check_status || echo "     (Check if nvblox initialized)"
    done
    
    # Scan topic
    echo "   Scan topic:"
    echo -n "     /scan: "
    ros2 topic list | grep -q "/scan"
    check_status
    
    echo ""
    echo "7. Verifying TF tree..."
    echo -n "   Checking TF: map -> odom: "
    timeout 2 ros2 run tf2_ros tf2_echo map odom > /dev/null 2>&1
    check_status || echo "     (VSLAM may not be initialized)"
    
    echo -n "   Checking TF: odom -> base_link: "
    timeout 2 ros2 run tf2_ros tf2_echo odom base_link > /dev/null 2>&1
    check_status || echo "     (VSLAM may not be initialized)"
    
    echo -n "   Checking TF: base_link -> camera_link: "
    timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link > /dev/null 2>&1
    check_status
    
    echo ""
    echo "8. Checking data rates..."
    echo "   Camera depth (expecting ~30 Hz):"
    timeout 3 ros2 topic hz /camera/depth/image_rect_raw 2>&1 | grep "average rate" || echo "     No data"
    
    echo "   Camera IMU (expecting ~200 Hz):"
    timeout 3 ros2 topic hz /camera/imu 2>&1 | grep "average rate" || echo "     No data"
    
    echo "   VSLAM odometry (expecting ~30 Hz):"
    timeout 3 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | grep "average rate" || echo "     No data (VSLAM may not be tracking)"
    
    echo "   Scan (expecting ~30 Hz):"
    timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" || echo "     No data"
    
    echo ""
    echo "9. Quick diagnostics..."
    echo -n "   VSLAM tracking status: "
    timeout 2 ros2 topic echo /visual_slam/status --once 2>&1 | grep "vo_state" || echo "No status"
    
fi

echo ""
echo "========================================="
echo "Verification complete!"
echo ""
echo "Next steps:"
echo "1. If all checks pass, try moving the camera slowly"
echo "2. Check RViz: ros2 launch your_robot_bringup rviz2.launch.py"
echo "3. Monitor topics: ros2 topic list"
echo "4. Check TF tree: ros2 run tf2_tools view_frames"
echo "========================================="