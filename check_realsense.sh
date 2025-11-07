#!/bin/bash

echo "=========================================="
echo "RealSense Camera Diagnostic Tool"
echo "=========================================="
echo ""

# Check if RealSense topics are available
echo "[1/3] Checking for RealSense topics..."
TOPICS=$(ros2 topic list 2>/dev/null | grep -i camera)

if [ -z "$TOPICS" ]; then
    echo "  ✗ No RealSense camera topics found!"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check if RealSense is connected (USB)"
    echo "  2. Check if RealSense driver is running:"
    echo "     ros2 node list | grep realsense"
    echo "  3. Start RealSense manually:"
    echo "     ros2 launch realsense2_camera rs_launch.py"
    echo ""
    exit 1
else
    echo "  ✓ Found RealSense topics:"
    echo "$TOPICS" | sed 's/^/    /'
    echo ""
fi

# Check specific streams
echo "[2/3] Checking available streams..."

check_topic() {
    TOPIC=$1
    NAME=$2
    if echo "$TOPICS" | grep -q "$TOPIC"; then
        # Check if publishing
        RATE=$(timeout 2 ros2 topic hz "$TOPIC" 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$RATE" ]; then
            echo "  ✓ $NAME: Publishing at ${RATE} Hz"
        else
            echo "  ⚠ $NAME: Topic exists but not publishing"
        fi
    else
        echo "  ✗ $NAME: Not available"
    fi
}

check_topic "/camera/camera/color/image_raw" "RGB Stream"
check_topic "/camera/camera/depth/image_rect_raw" "Depth Stream"
check_topic "/camera/camera/infra1/image_rect_raw" "Infrared 1 Stream"
check_topic "/camera/camera/infra2/image_rect_raw" "Infrared 2 Stream"

echo ""

# Check RealSense nodes
echo "[3/3] Checking RealSense nodes..."
NODES=$(ros2 node list 2>/dev/null | grep -i realsense)

if [ -z "$NODES" ]; then
    echo "  ✗ No RealSense nodes running"
    echo ""
    echo "To start RealSense camera:"
    echo "  ros2 launch realsense2_camera rs_launch.py"
    echo ""
else
    echo "  ✓ RealSense nodes running:"
    echo "$NODES" | sed 's/^/    /'
    echo ""
fi

echo "=========================================="
echo "Ready to view camera!"
echo "=========================================="
echo ""
echo "Run the viewer:"
echo "  ./view_realsense.sh"
echo ""

