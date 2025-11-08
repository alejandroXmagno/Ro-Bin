#!/bin/bash

echo "=== RPLidar Diagnostic Tool ==="
echo ""

# Check if device exists
if [ -L /dev/rplidar ]; then
    REAL_DEVICE=$(readlink -f /dev/rplidar)
    echo "✓ /dev/rplidar exists -> $REAL_DEVICE"
    
    if [ -c "$REAL_DEVICE" ]; then
        echo "✓ Device file exists and is a character device"
        
        # Check permissions
        PERMS=$(stat -c "%a" "$REAL_DEVICE" 2>/dev/null)
        echo "  Permissions: $PERMS"
        
        # Check if readable
        if [ -r "$REAL_DEVICE" ]; then
            echo "✓ Device is readable"
        else
            echo "✗ Device is NOT readable (check permissions)"
        fi
        
        # Check if writable
        if [ -w "$REAL_DEVICE" ]; then
            echo "✓ Device is writable"
        else
            echo "✗ Device is NOT writable (check permissions)"
        fi
    else
        echo "✗ Device file does not exist: $REAL_DEVICE"
    fi
else
    echo "✗ /dev/rplidar symlink does not exist"
    echo "  Available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null | head -5
fi

echo ""
echo "=== USB Device Information ==="
lsusb | grep -i "10c4:ea60" && echo "✓ RPLidar USB device detected" || echo "✗ RPLidar USB device NOT detected (check USB connection)"

echo ""
echo "=== Current Configuration ==="
if [ -f "src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml" ]; then
    echo "RPLidar active status:"
    grep -A 1 "rplidar:" src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml | grep "active" || echo "  (not found)"
    echo "RPLidar serial port:"
    grep "serial_port:" src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml | head -1 || echo "  (not found)"
else
    echo "✗ Configuration file not found"
fi

echo ""
echo "=== Process Check ==="
RPLIDAR_PROCESSES=$(pgrep -f rplidar_composition 2>/dev/null | wc -l)
if [ "$RPLIDAR_PROCESSES" -gt 0 ]; then
    echo "⚠ Warning: Found $RPLIDAR_PROCESSES RPLidar process(es) running:"
    ps aux | grep rplidar_composition | grep -v grep
    echo ""
    echo "  These may be preventing access to the device."
    echo "  To kill stale processes: pkill -f rplidar_composition"
else
    echo "✓ No RPLidar processes currently running"
fi

echo ""
echo "=== Recommendations ==="
if [ -c "$REAL_DEVICE" ] 2>/dev/null; then
    echo "1. Try unplugging and replugging the USB cable"
    echo "2. Ensure RPLidar is fully powered (check LED indicators)"
    echo "3. Check if another process is using the device:"
    echo "   lsof $REAL_DEVICE"
    echo "4. Try reading from device directly (may help reset it):"
    echo "   timeout 1 cat $REAL_DEVICE > /dev/null"
    echo "5. If issues persist, the RPLidar may need a firmware reset"
else
    echo "1. Check USB connection"
    echo "2. Verify udev rules are installed:"
    echo "   ls -la /etc/udev/rules.d/*rplidar*"
    echo "3. Reload udev rules if needed:"
    echo "   sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

