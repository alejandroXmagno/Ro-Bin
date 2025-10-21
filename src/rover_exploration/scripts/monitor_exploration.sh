#!/bin/bash

# Monitor the autonomous exploration status
# This script provides a dashboard of the exploration progress

echo "=========================================="
echo "   Rover Exploration Monitor Dashboard   "
echo "=========================================="
echo ""
echo "Monitoring exploration progress..."
echo "Press Ctrl+C to exit"
echo ""
echo "------------------------------------------"

# Function to check if a topic exists
topic_exists() {
    ros2 topic list 2>/dev/null | grep -q "^$1$"
    return $?
}

# Function to check if a node is running
node_exists() {
    ros2 node list 2>/dev/null | grep -q "$1"
    return $?
}

# Main monitoring loop
while true; do
    # Clear previous output (move cursor up)
    tput cuu 8 2>/dev/null || true
    tput ed 2>/dev/null || true
    
    echo "STATUS CHECK - $(date +%H:%M:%S)"
    echo "------------------------------------------"
    
    # Check SLAM
    if node_exists "slam_toolbox"; then
        echo "✓ SLAM:        Running"
    else
        echo "✗ SLAM:        Not running"
    fi
    
    # Check Navigation
    if node_exists "controller_server"; then
        echo "✓ Navigation:  Running"
    else
        echo "✗ Navigation:  Not running"
    fi
    
    # Check Exploration
    if node_exists "autonomous_explorer"; then
        echo "✓ Exploration: Running"
    else
        echo "✗ Exploration: Not running"
    fi
    
    # Check LIDAR
    if topic_exists "/scan"; then
        SCAN_RATE=$(ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$SCAN_RATE" ]; then
            echo "✓ LIDAR:       Publishing @ ${SCAN_RATE} Hz"
        else
            echo "~ LIDAR:       Topic exists (checking rate...)"
        fi
    else
        echo "✗ LIDAR:       No data"
    fi
    
    # Check Map
    if topic_exists "/map"; then
        echo "✓ Map:         Publishing"
    else
        echo "✗ Map:         Not available"
    fi
    
    # Check Robot velocity
    if topic_exists "/cmd_vel"; then
        echo "✓ Commands:    Active"
    else
        echo "✗ Commands:    Not available"
    fi
    
    echo "------------------------------------------"
    echo ""
    
    # Sleep before next update
    sleep 2
done



