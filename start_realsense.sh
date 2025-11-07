#!/bin/bash

# Start RealSense camera on physical robot

echo "=========================================="
echo "Starting RealSense Camera"
echo "=========================================="
echo ""

# Check if realsense2_camera package is installed
if ! ros2 pkg list | grep -q "realsense2_camera"; then
    echo "✗ ERROR: realsense2_camera package not found!"
    echo ""
    echo "Install it with:"
    echo "  sudo apt install ros-humble-realsense2-camera"
    echo ""
    echo "Or build from source if needed."
    echo ""
    exit 1
fi

echo "Launching RealSense camera..."
echo ""
echo "Streams available:"
echo "  - RGB: /camera/camera/color/image_raw"
echo "  - Depth: /camera/camera/depth/image_rect_raw"
echo "  - Infrared: /camera/camera/infra1/image_rect_raw"
echo "  - Infrared 2: /camera/camera/infra2/image_rect_raw"
echo ""
echo "To view the camera:"
echo "  Terminal 2: ./view_realsense.sh"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch RealSense with default parameters
# Adjust parameters as needed for your setup
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_infra1:=true \
    enable_infra2:=true \
    align_depth.enable:=true

