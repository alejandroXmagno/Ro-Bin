#!/bin/bash

# Script to find and kill ROS2 joystick control processes

echo "Checking for ROS2 joystick processes..."

# Find processes related to joystick/teleop
JOYSTICK_PIDS=$(ps aux | grep -iE "joy|joystick|teleop" | grep -v grep | grep -v "$0" | awk '{print $2}')

if [ -z "$JOYSTICK_PIDS" ]; then
    echo "No joystick processes found in process list."
else
    echo "Found joystick processes:"
    ps aux | grep -iE "joy|joystick|teleop" | grep -v grep | grep -v "$0"
    echo ""
    echo "Killing processes..."
    for pid in $JOYSTICK_PIDS; do
        echo "  Killing PID: $pid"
        kill -9 "$pid" 2>/dev/null
    done
    echo "Processes killed."
fi

# Check for ROS2 nodes (if ROS2 is sourced)
if command -v ros2 &> /dev/null; then
    echo ""
    echo "Checking for ROS2 joystick nodes..."
    JOYSTICK_NODES=$(ros2 node list 2>/dev/null | grep -iE "joy|teleop" || true)
    
    if [ -z "$JOYSTICK_NODES" ]; then
        echo "No joystick nodes found in ROS2."
    else
        echo "Found ROS2 joystick nodes:"
        echo "$JOYSTICK_NODES"
        echo ""
        echo "Note: ROS2 nodes should be terminated by killing their processes above."
        echo "If nodes persist, you may need to restart the ROS2 daemon."
    fi
else
    echo ""
    echo "ROS2 not found in PATH (may not be sourced)."
fi

echo ""
echo "Done!"

