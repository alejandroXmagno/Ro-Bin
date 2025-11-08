#!/bin/bash
# Debug script to find why IMU isn't publishing

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "IMU Debug Diagnostics"
echo "=========================================="
echo ""

echo "1. Checking IMU node status..."
if ros2 node list 2>/dev/null | grep -q "bno055"; then
    echo "   ✓ BNO055 node is running"
    ros2 node list | grep bno055
else
    echo "   ✗ BNO055 node is NOT running!"
    echo "   Available nodes:"
    ros2 node list | head -10
fi
echo ""

echo "2. Checking IMU topics..."
if ros2 topic list 2>/dev/null | grep -q "/imu"; then
    echo "   ✓ IMU topics found:"
    ros2 topic list | grep imu
else
    echo "   ✗ No IMU topics found!"
fi
echo ""

echo "3. Checking IMU device files..."
echo "   Looking for /dev/bno055:"
if [ -e /dev/bno055 ]; then
    echo "   ✓ /dev/bno055 exists"
    ls -l /dev/bno055
else
    echo "   ✗ /dev/bno055 does NOT exist"
fi

echo "   Looking for /dev/ttyUSB*:"
USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -n "$USB_DEVICES" ]; then
    echo "   ✓ Found USB serial devices:"
    for dev in $USB_DEVICES; do
        echo "      $dev ($(ls -l $dev | awk '{print $3, $4}'))"
    done
else
    echo "   ✗ No /dev/ttyUSB* devices found"
fi
echo ""

echo "4. Checking IMU configuration..."
CONFIG_FILE="src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "   IMU active status:"
    grep -A 2 "bno055:" "$CONFIG_FILE" | grep "active:" | head -1
    echo "   IMU uart_port:"
    grep -A 5 "bno055:" "$CONFIG_FILE" | grep "uart_port:" | head -1
else
    echo "   ✗ Config file not found: $CONFIG_FILE"
fi
echo ""

echo "5. Checking if IMU node is subscribed to anything..."
if ros2 node list 2>/dev/null | grep -q "bno055"; then
    echo "   BNO055 node info:"
    ros2 node info /bno055 2>&1 | head -20
else
    echo "   ✗ Cannot check - BNO055 node not running"
fi
echo ""

echo "6. Checking for IMU messages (5 second timeout)..."
timeout 5 ros2 topic echo /imu/data --once 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "   ✓ IMU is publishing!"
else
    echo "   ✗ No IMU messages received"
fi
echo ""

echo "7. Checking robot driver status..."
if ros2 node list 2>/dev/null | grep -q "roverrobotics_driver"; then
    echo "   ✓ Robot driver is running"
    echo "   (IMU should be started by accessories.launch.py)"
else
    echo "   ✗ Robot driver is NOT running"
    echo "   IMU won't start without robot driver"
fi
echo ""

echo "=========================================="
echo "Troubleshooting Tips:"
echo "=========================================="
echo ""
echo "If IMU node is not running:"
echo "  1. Make sure robot driver is running: ros2 launch roverrobotics_driver mini.launch.py"
echo "  2. Check accessories.yaml - bno055.active should be 'true'"
echo "  3. Check uart_port matches actual device (/dev/bno055 or /dev/ttyUSB0)"
echo ""
echo "If IMU device doesn't exist:"
echo "  1. Check USB connection: lsusb | grep -i bno"
echo "  2. Check udev rules: ls -l /dev/bno055"
echo "  3. Try: sudo chmod 666 /dev/ttyUSB0 (if using /dev/ttyUSB0)"
echo ""
echo "If IMU node is running but not publishing:"
echo "  1. Check node output for errors: ros2 node info /bno055"
echo "  2. Check device permissions: ls -l /dev/bno055"
echo "  3. Try restarting robot driver"
echo ""

