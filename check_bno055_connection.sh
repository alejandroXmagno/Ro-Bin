#!/bin/bash

echo "=========================================="
echo "BNO055 IMU Connection Diagnostic"
echo "=========================================="
echo ""

# Check if device exists
if [ -e /dev/bno055 ]; then
    echo "✓ /dev/bno055 exists"
    echo "  -> $(readlink -f /dev/bno055)"
    echo "  Permissions: $(ls -l /dev/bno055 | awk '{print $1, $3, $4}')"
else
    echo "✗ /dev/bno055 does not exist!"
    echo "  Check udev rules: ls -la /etc/udev/rules.d/55-roverrobotics.rules"
    exit 1
fi
echo ""

# Check if device is accessible
if [ -r /dev/bno055 ] && [ -w /dev/bno055 ]; then
    echo "✓ Device is readable and writable"
else
    echo "⚠ Device permissions may be insufficient"
    echo "  Try: sudo chmod 666 /dev/bno055"
fi
echo ""

# Check if BNO055 node is running
echo "Checking BNO055 node status..."
if ros2 node list 2>/dev/null | grep -q bno055; then
    echo "✓ BNO055 node is running"
    ros2 node list 2>/dev/null | grep bno055
else
    echo "✗ BNO055 node is NOT running"
    echo "  Make sure 'bno055.active: true' in accessories.yaml"
fi
echo ""

# Check IMU topics
echo "Checking IMU topics..."
if ros2 topic list 2>/dev/null | grep -q imu; then
    echo "✓ IMU topics exist:"
    ros2 topic list 2>/dev/null | grep imu
    echo ""
    echo "Checking if data is being published..."
    timeout 2 ros2 topic hz /imu/data 2>/dev/null || echo "  ⚠ No data on /imu/data (node may be failing to connect)"
else
    echo "✗ No IMU topics found"
fi
echo ""

# Check USB device
echo "Checking USB device..."
USB_DEVICE=$(readlink -f /dev/bno055)
if [ -n "$USB_DEVICE" ]; then
    echo "Physical device: $USB_DEVICE"
    lsusb | grep -i "0403:6014" && echo "  ✓ FTDI device found (vendor 0403:6014)"
fi
echo ""

echo "=========================================="
echo "If BNO055 node is running but not publishing:"
echo "1. Check the terminal where mini.launch is running for error messages"
echo "2. Look for 'Failed to connect' or 'Permission denied' errors"
echo "3. Try: sudo chmod 666 /dev/bno055"
echo "4. Restart mini.launch after fixing permissions"
echo "=========================================="

