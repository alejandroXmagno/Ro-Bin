#!/bin/bash
# Kill all EKF nodes and clean up

echo "Killing all EKF nodes..."

# Kill by process name
pkill -9 -f ekf_node
pkill -9 -f robot_localizer

# Wait a moment
sleep 2

# Check if any are still running
REMAINING=$(ros2 node list 2>/dev/null | grep -c ekf || echo "0")
if [ "$REMAINING" != "0" ]; then
    echo "⚠ Still $REMAINING EKF nodes running"
    echo "Trying harder..."
    # Kill by PID if we can find them
    ps aux | grep ekf_node | grep -v grep | awk '{print $2}' | xargs -r kill -9
    sleep 1
fi

# Final check
REMAINING=$(ros2 node list 2>/dev/null | grep -c ekf || echo "0")
if [ "$REMAINING" = "0" ]; then
    echo "✓ All EKF nodes killed"
else
    echo "⚠ Warning: $REMAINING EKF nodes may still be running"
    echo "You may need to restart the robot driver"
fi

