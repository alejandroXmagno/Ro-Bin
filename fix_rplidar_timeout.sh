#!/bin/bash

echo "=========================================="
echo "RPLidar Timeout Fix Script"
echo "=========================================="
echo ""

# Step 1: Kill all RPLidar processes
echo "[1/5] Killing stale RPLidar processes..."
pkill -9 -f rplidar_composition 2>/dev/null && echo "  ✓ Killed RPLidar processes" || echo "  ℹ No RPLidar processes found"
sleep 1

# Step 2: Find the device
echo ""
echo "[2/5] Locating RPLidar device..."
if [ -L /dev/rplidar ]; then
    DEVICE=$(readlink -f /dev/rplidar)
    echo "  ✓ Found RPLidar at: $DEVICE"
elif [ -e /dev/ttyUSB0 ]; then
    DEVICE=/dev/ttyUSB0
    echo "  ℹ Using /dev/ttyUSB0"
elif [ -e /dev/ttyUSB1 ]; then
    DEVICE=/dev/ttyUSB1
    echo "  ℹ Using /dev/ttyUSB1"
else
    echo "  ✗ ERROR: Cannot find RPLidar device!"
    echo ""
    echo "Troubleshooting steps:"
    echo "  1. Check if RPLidar is plugged in (USB)"
    echo "  2. Check if LED is spinning (power indicator)"
    echo "  3. Try different USB port"
    echo "  4. Check USB connection:"
    lsusb | grep -i "10c4:ea60" && echo "     ✓ USB device detected" || echo "     ✗ USB device NOT detected"
    echo "  5. Check available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "     No /dev/ttyUSB* devices found"
    echo ""
    exit 1
fi

# Step 3: Check and release device locks
echo ""
echo "[3/5] Checking for device locks..."
if command -v fuser >/dev/null 2>&1; then
    fuser -k "$DEVICE" 2>/dev/null && echo "  ✓ Released device lock" || echo "  ℹ No lock found"
else
    echo "  ℹ fuser not available, skipping"
fi

if command -v lsof >/dev/null 2>&1; then
    LOCKS=$(lsof "$DEVICE" 2>/dev/null)
    if [ -n "$LOCKS" ]; then
        echo "  ⚠ Device still locked by:"
        echo "$LOCKS"
        echo "  Attempting to kill processes..."
        lsof -t "$DEVICE" 2>/dev/null | xargs -r kill -9 2>/dev/null && echo "  ✓ Killed locking processes" || echo "  ✗ Failed to kill"
    else
        echo "  ✓ No active locks"
    fi
fi

sleep 1

# Step 4: Reset the device
echo ""
echo "[4/5] Resetting RPLidar device..."
# Try to read from device to reset it
timeout 1 cat "$DEVICE" > /dev/null 2>&1 && echo "  ✓ Device responded" || echo "  ℹ Device may need physical reset"

# Set proper permissions
if [ -w "$DEVICE" ]; then
    echo "  ✓ Device is writable"
else
    echo "  ⚠ Device is not writable, trying to fix..."
    sudo chmod 666 "$DEVICE" 2>/dev/null && echo "  ✓ Permissions fixed" || echo "  ✗ Cannot fix permissions (run as sudo?)"
fi

# Step 5: Verify readiness
echo ""
echo "[5/5] Verifying device readiness..."
if [ -c "$DEVICE" ] && [ -r "$DEVICE" ] && [ -w "$DEVICE" ]; then
    echo "  ✓ Device exists and has correct permissions"
    echo "  ✓ RPLidar should be ready!"
    echo ""
    echo "=========================================="
    echo "✓ Fix Complete!"
    echo "=========================================="
    echo ""
    echo "Now try launching the robot driver:"
    echo "  pkill -f roverrobotics_ros2_driver || true"
    echo "  fuser -kv /dev/ttyACM0 || true"
    echo "  source install/setup.bash"
    echo "  ros2 launch roverrobotics_driver mini.launch.py"
    echo ""
    exit 0
else
    echo "  ✗ Device still not ready"
    echo ""
    echo "=========================================="
    echo "Manual troubleshooting required:"
    echo "=========================================="
    echo ""
    echo "1. UNPLUG the RPLidar USB cable"
    echo "2. Wait 5 seconds"
    echo "3. PLUG it back in"
    echo "4. Wait for LED to spin"
    echo "5. Run this script again: ./fix_rplidar_timeout.sh"
    echo ""
    echo "If that doesn't work:"
    echo "  - Try a different USB port"
    echo "  - Check if the cable is damaged"
    echo "  - Verify RPLidar is getting power (LED spinning)"
    echo ""
    echo "Advanced diagnostics:"
    echo "  ./check_rplidar.sh"
    echo ""
    exit 1
fi

