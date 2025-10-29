#!/bin/bash

# WASD Keyboard Controller for Rover
# Make sure the rover driver is running first!

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "  WASD Keyboard Controller for Rover     "
echo "=========================================="
echo ""
echo "Controls:"
echo "  W = Forward"
echo "  S = Backward"
echo "  A = Turn Left"
echo "  D = Turn Right"
echo "  Space = Stop"
echo "  Q = Quit"
echo ""
echo "Make sure your rover driver is running!"
echo "Press any key to start..."
echo ""

# Give user a moment to read
sleep 2

# Run the Python controller
python3 wasd_controller.py
