#!/bin/bash
cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "  Opening RViz2 for SLAM Visualization   "
echo "=========================================="
echo ""
echo "LaserScan will show up as RED DOTS"
echo "Map will show up as GRAY areas"
echo ""
echo "If you don't see anything:"
echo "  1. Make sure Fixed Frame is set to 'chassis_link'"
echo "  2. LaserScan topic is '/scan'"
echo "  3. Size is 0.05m or larger"
echo ""

ros2 run rviz2 rviz2 -d rviz_slam.rviz


