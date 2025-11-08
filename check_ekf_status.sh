#!/bin/bash

echo "=========================================="
echo "EKF Status Diagnostic"
echo "=========================================="
echo ""

source install/setup.bash 2>/dev/null || true

echo "1. Checking if EKF node is running..."
if ros2 node list 2>/dev/null | grep -q ekf; then
    echo "   ✓ EKF node is running"
    ros2 node list 2>/dev/null | grep ekf
    echo ""
    echo "   EKF node info:"
    ros2 node info /ekf_filter_node 2>/dev/null | head -30 || echo "   (Could not get node info)"
else
    echo "   ✗ EKF node is NOT running!"
fi
echo ""

echo "2. Checking if EKF is subscribing to IMU..."
if ros2 node info /ekf_filter_node 2>/dev/null | grep -q "imu/data"; then
    echo "   ✓ EKF is subscribed to /imu/data"
else
    echo "   ✗ EKF is NOT subscribed to /imu/data"
    echo "   Full subscriptions:"
    ros2 node info /ekf_filter_node 2>/dev/null | grep -A 5 "Subscribers:" || echo "   (Could not get subscriptions)"
fi
echo ""

echo "3. Checking if EKF is publishing odom->base_link transform..."
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -10; then
    echo "   ✓ Transform is being published"
else
    echo "   ✗ Transform NOT available!"
    echo "   This means EKF is not publishing odom->base_link"
fi
echo ""

echo "4. Checking EKF diagnostics topic..."
if ros2 topic list 2>/dev/null | grep -q "diagnostics"; then
    echo "   ✓ Diagnostics topic found"
    echo "   Latest EKF diagnostics:"
    timeout 2 ros2 topic echo /diagnostics --once 2>/dev/null | grep -A 20 "ekf_filter_node" || echo "   (No diagnostics data)"
else
    echo "   ⚠ No diagnostics topic (EKF might not be publishing diagnostics)"
fi
echo ""

echo "5. Checking if IMU data is available for EKF..."
if timeout 2 ros2 topic hz /imu/data 2>/dev/null | head -3; then
    echo "   ✓ IMU is publishing data"
else
    echo "   ✗ IMU is NOT publishing data!"
fi
echo ""

echo "6. Checking if cmd_vel is available (for EKF motion model)..."
if timeout 2 ros2 topic hz /cmd_vel 2>/dev/null | head -3; then
    echo "   ✓ cmd_vel topic exists"
else
    echo "   ⚠ cmd_vel topic not found (EKF uses this for position estimation)"
fi
echo ""

echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "If EKF is running but not publishing transforms:"
echo "  1. EKF needs IMU data + cmd_vel to initialize"
echo "  2. EKF might be waiting for initial data"
echo "  3. Check /tmp/ekf_debug.txt for debug output"
echo "=========================================="

