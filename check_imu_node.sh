#!/bin/bash
# Quick check for IMU node and topics

cd /home/stickykeys/rover_workspace
source install/setup.bash 2>/dev/null || true

echo "Checking IMU status..."
echo ""

echo "1. BNO055 Node:"
ros2 node list 2>&1 | grep -i bno || echo "   ✗ Not running"
echo ""

echo "2. IMU Topics:"
ros2 topic list 2>&1 | grep -i imu || echo "   ✗ No IMU topics"
echo ""

echo "3. IMU Device:"
if [ -e /dev/bno055 ]; then
    echo "   ✓ /dev/bno055 exists"
    ls -l /dev/bno055
else
    echo "   ✗ /dev/bno055 does NOT exist"
fi
echo ""

echo "4. Try to get IMU message (2 second timeout):"
timeout 2 ros2 topic echo /imu/data --once 2>&1 | head -5 || echo "   ✗ No messages"
echo ""

echo "5. Check if robot driver is running:"
if ros2 node list 2>&1 | grep -q "roverrobotics_driver"; then
    echo "   ✓ Robot driver running (IMU should be started by accessories.launch.py)"
else
    echo "   ✗ Robot driver NOT running"
    echo "   Start with: ros2 launch roverrobotics_driver mini.launch.py"
fi

