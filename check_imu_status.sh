#!/bin/bash

# Script to check IMU status and EKF transform publishing

echo "=========================================="
echo "IMU and EKF Diagnostic Check"
echo "=========================================="
echo ""

# Source ROS2 workspace
source install/setup.bash 2>/dev/null || true

echo "1. Checking IMU topic..."
if ros2 topic list 2>/dev/null | grep -q "imu"; then
    echo "   ✓ IMU topic found:"
    ros2 topic list 2>/dev/null | grep imu
    echo ""
    echo "   Checking IMU data rate..."
    timeout 2 ros2 topic hz /imu/data 2>/dev/null || echo "   ⚠ No data on /imu/data topic"
else
    echo "   ✗ No IMU topic found!"
    echo "   Note: IMU must be enabled in accessories.yaml (set bno055.active: true)"
fi
echo ""

echo "2. Checking EKF node..."
if ros2 node list 2>/dev/null | grep -q "ekf"; then
    echo "   ✓ EKF node found:"
    ros2 node list 2>/dev/null | grep ekf
else
    echo "   ✗ EKF node not running!"
fi
echo ""

echo "3. Checking odom->base_link transform..."
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -20; then
    echo "   ✓ Transform is being published"
else
    echo "   ✗ Transform not available!"
    echo "   This means EKF is not receiving IMU data or is not initialized"
fi
echo ""

echo "4. Checking all transforms..."
echo "   Available transforms:"
ros2 run tf2_ros tf2_monitor 2>/dev/null | head -30 || echo "   (No transforms available)"

echo ""
echo "=========================================="
echo "Diagnostic complete"
echo "=========================================="

