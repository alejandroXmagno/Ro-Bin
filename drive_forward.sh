#!/bin/bash

# Simple script to drive the rover forward for 1 second then stop
# Make sure the rover is running first!

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "  Simple Rover Movement Test"
echo "=========================================="
echo ""
echo "This will drive the rover forward for 1 second at low speed, then stop."
echo "Make sure your rover is running and connected!"
echo ""
echo "Press Ctrl+C to cancel..."
echo ""

# Give user a moment to read the message
sleep 2

# Run the Python script
python3 drive_forward.py

