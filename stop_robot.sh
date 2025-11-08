#!/bin/bash
# Emergency stop script - stops all robot movement

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "Stopping robot movement..."

# Publish zero velocity command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Also try stopping via explorer topic if it exists
ros2 topic pub --once /cmd_vel_explorer geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true

echo "Zero velocity command sent"
echo ""
echo "To permanently stop, you can:"
echo "  1. Kill the autonomous explorer: pkill -f autonomous_explorer"
echo "  2. Kill navigation stack: pkill -f nav2"
echo "  3. Or press Ctrl+C in the navigation terminal"

