#!/bin/bash
# Debug script to find why EKF isn't publishing odom->base_link

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "EKF Debug Diagnostics"
echo "=========================================="
echo ""

echo "1. Checking if EKF node is running..."
if ros2 node list 2>/dev/null | grep -q ekf; then
    echo "   ✓ EKF node is running"
    ros2 node list | grep ekf
else
    echo "   ✗ EKF node is NOT running!"
    echo "   Available nodes:"
    ros2 node list
fi
echo ""

echo "2. Checking IMU topic..."
if timeout 2 ros2 topic list 2>/dev/null | grep -q "/imu/data"; then
    echo "   ✓ /imu/data topic exists"
    echo "   Checking IMU publishing rate..."
    timeout 3 ros2 topic hz /imu/data 2>&1 | head -3
else
    echo "   ✗ /imu/data topic does NOT exist!"
    echo "   Available topics:"
    ros2 topic list | grep -i imu
fi
echo ""

echo "3. Checking if EKF is subscribed to IMU..."
if ros2 node info /ekf_filter_node 2>/dev/null | grep -q "imu/data"; then
    echo "   ✓ EKF is subscribed to /imu/data"
else
    echo "   ✗ EKF is NOT subscribed to /imu/data"
    echo "   EKF subscriptions:"
    ros2 node info /ekf_filter_node 2>/dev/null | grep -A 10 "Subscribers:"
fi
echo ""

echo "4. Checking if odom frame exists..."
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo "   ✓ odom->base_link transform EXISTS"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5
else
    echo "   ✗ odom->base_link transform does NOT exist"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -3
fi
echo ""

echo "5. Checking EKF node info..."
ros2 node info /ekf_filter_node 2>&1 | head -30
echo ""

echo "6. Checking if EKF is publishing to /tf..."
if ros2 topic list 2>/dev/null | grep -q "/tf"; then
    echo "   ✓ /tf topic exists"
    echo "   Checking /tf publishing rate..."
    timeout 2 ros2 topic hz /tf 2>&1 | head -3
else
    echo "   ✗ /tf topic does NOT exist!"
fi
echo ""

echo "7. Checking EKF parameters..."
ros2 param list /ekf_filter_node 2>&1 | grep -E "(publish_tf|odom_frame|base_link_frame|imu0|frequency)" | head -10
echo ""

echo "8. Checking recent IMU messages..."
echo "   Last 3 IMU messages (if available):"
timeout 2 ros2 topic echo /imu/data --once 2>&1 | head -20
echo ""

echo "=========================================="
echo "Debug complete"
echo "=========================================="

