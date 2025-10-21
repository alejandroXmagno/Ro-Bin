#!/bin/bash
cd /home/lechonka/rover_workspace
source install/setup.bash
echo "=========================================="
echo "  Starting SLAM Toolbox for Mapping      "
echo "=========================================="
echo ""
echo "SLAM Configuration:"
echo "  - Increased transform buffer: 60s"
echo "  - Increased scan buffer: 25"
echo "  - Transform timeout: 1.0s"
echo ""
echo "SLAM will:"
echo "  - Subscribe to /scan (LIDAR)"
echo "  - Subscribe to /odometry/wheels"
echo "  - Publish /map topic"
echo ""
echo "Watch the map build in RViz2!"
echo "Press Ctrl+C to stop"
echo ""
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file slam_config.yaml \
  -p use_sim_time:=true \
  -r scan:=/scan \
  -r odom:=/odometry/wheels


