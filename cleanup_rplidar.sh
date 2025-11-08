#!/bin/bash

echo "=== Cleaning up stale RPLidar processes ==="

# Find and kill stale RPLidar processes
PIDS=$(pgrep -f rplidar_composition 2>/dev/null)

if [ -z "$PIDS" ]; then
    echo "✓ No RPLidar processes found"
else
    echo "Found RPLidar processes: $PIDS"
    for PID in $PIDS; do
        echo "  Killing PID $PID..."
        kill -9 "$PID" 2>/dev/null && echo "    ✓ Killed" || echo "    ✗ Failed"
    done
fi

# Wait a moment for processes to release resources
sleep 1

# Check if device is free
if lsof /dev/ttyUSB1 2>/dev/null | grep -q .; then
    echo ""
    echo "⚠ Warning: Device /dev/ttyUSB1 is still in use:"
    lsof /dev/ttyUSB1
    echo ""
    echo "You may need to:"
    echo "  1. Unplug and replug the USB cable"
    echo "  2. Kill the process manually: kill -9 <PID>"
else
    echo ""
    echo "✓ Device /dev/ttyUSB1 is now free"
    echo "✓ You can now launch the rover driver"
fi

