#!/bin/bash
# Start EKF node using robot_localizer.launch.py

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "Starting EKF (IMU-based)"
echo "=========================================="
echo ""
echo "This will start the EKF node that:"
echo "  - Uses IMU data for orientation"
echo "  - Publishes odom->base_link transform"
echo "  - Does NOT use wheel odometry"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Check if EKF is already running
if ros2 node list 2>/dev/null | grep -q ekf; then
    echo "⚠ EKF is already running!"
    echo "   Existing EKF nodes:"
    ros2 node list | grep ekf
    echo ""
    read -p "Kill existing EKF and start new one? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Killing existing EKF nodes..."
        pkill -f ekf_node
        pkill -f robot_localizer
        sleep 2
    else
        echo "Keeping existing EKF. Exiting."
        exit 0
    fi
fi

# Start EKF
echo "Starting EKF node..."
ros2 launch roverrobotics_driver robot_localizer.launch.py

