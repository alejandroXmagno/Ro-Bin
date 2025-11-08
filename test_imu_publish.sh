#!/bin/bash

echo "Testing IMU data publishing..."
echo ""

source install/setup.bash 2>/dev/null || true

echo "1. Checking if BNO055 node is running..."
if ros2 node list 2>/dev/null | grep -q bno055; then
    echo "   ✓ BNO055 node is running"
else
    echo "   ✗ BNO055 node is NOT running"
    exit 1
fi

echo ""
echo "2. Checking IMU topics..."
TOPICS=$(ros2 topic list 2>/dev/null | grep imu)
if [ -n "$TOPICS" ]; then
    echo "   Found topics:"
    echo "$TOPICS" | sed 's/^/     /'
else
    echo "   ✗ No IMU topics found"
    exit 1
fi

echo ""
echo "3. Waiting 5 seconds for data..."
echo "   (If you see data below, IMU is working)"
echo ""
timeout 5 ros2 topic echo /imu/data --once 2>&1 | head -20 || {
    echo ""
    echo "   ⚠ No data received on /imu/data"
    echo ""
    echo "4. Checking for warnings/errors in node info..."
    ros2 node info /bno055 2>&1 | grep -A 5 -i "publisher\|error\|warn" | head -20
}

echo ""
echo "Done."

