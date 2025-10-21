#!/bin/bash
cd /home/lechonka/rover_workspace
source install/setup.bash
echo "=========================================="
echo "  Keyboard Teleop - Control Your Rover   "
echo "=========================================="
echo ""
echo "Controls:"
echo "  i = Forward"
echo "  , = Backward"
echo "  j = Turn Left"
echo "  l = Turn Right"
echo "  k = Stop"
echo ""
echo "Press Ctrl+C to exit"
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard



