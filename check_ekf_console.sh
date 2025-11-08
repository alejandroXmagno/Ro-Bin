#!/bin/bash
# Check EKF console output for diagnostic messages

echo "=========================================="
echo "EKF Console Output Check"
echo "=========================================="
echo ""
echo "This will check for EKF diagnostic messages."
echo "Make sure EKF is running in another terminal."
echo ""
echo "Looking for EKF processes..."
ps aux | grep ekf_node | grep -v grep
echo ""
echo "To see EKF output, check the terminal where you started it,"
echo "or look for log files in: ~/.ros/log/"
echo ""
echo "Recent log files:"
ls -lt ~/.ros/log/*/ekf_node* 2>/dev/null | head -5 || echo "No log files found"
echo ""
echo "To see live EKF output, run:"
echo "  ros2 launch roverrobotics_driver robot_localizer.launch.py"
echo ""

