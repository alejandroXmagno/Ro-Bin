#!/bin/bash
# Check if EKF is actually publishing transforms

cd /home/stickykeys/rover_workspace
source install/setup.bash 2>/dev/null || true

echo "Checking EKF transform publishing..."
echo ""

echo "1. Checking /tf topic rate:"
timeout 3 ros2 topic hz /tf 2>&1 | head -3
echo ""

echo "2. Checking for odom->base_link in /tf messages:"
timeout 3 ros2 topic echo /tf --once 2>&1 | grep -A 5 -B 5 "odom\|base_link" | head -20
echo ""

echo "3. Checking transform tree:"
timeout 5 ros2 run tf2_tools view_frames 2>&1 | grep -A 2 "base_link:" | head -5
echo ""

echo "4. Direct transform check:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5
echo ""

echo "5. Checking if odom frame exists:"
timeout 1 ros2 run tf2_ros tf2_echo odom odom 2>&1 | head -3 || echo "   odom frame doesn't exist (this is normal - odom is the root)"
echo ""

