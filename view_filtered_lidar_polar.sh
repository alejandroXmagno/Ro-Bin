#!/bin/bash

# View Filtered LiDAR in Polar Plot
# Visualizes /scan_filtered topic in a real-time polar plot

echo "Starting Filtered LiDAR Polar Plot Visualizer..."
echo "This will show only valid (non-filtered) LiDAR readings"
echo ""
echo "Make sure:"
echo "  1. Robot driver is running (mini.launch.py)"
echo "  2. Auto standoff filter is running (should start automatically)"
echo "  3. /scan_filtered topic is publishing"
echo ""
echo "Press Ctrl+C to stop"
echo ""

source install/setup.bash
ros2 run roverrobotics_input_manager visualize_filtered_lidar.py


