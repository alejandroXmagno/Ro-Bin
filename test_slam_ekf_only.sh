#!/bin/bash
# Test EKF + SLAM without Nav2 - allows testing transforms without robot movement
# This starts EKF and SLAM but NOT Nav2, so robot won't move

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "EKF + SLAM Test (No Nav2, No Movement)"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start EKF node"
echo "  2. Start SLAM node"
echo "  3. Monitor transform publishing"
echo "  4. NOT start Nav2 (robot won't move)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Check if robot driver is running (check for node or topics)
if ! ros2 node list 2>/dev/null | grep -q "roverrobotics_driver" && ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "⚠ Robot driver not detected."
    echo "   Make sure roverrobotics_driver/mini.launch is running first."
    echo "   Run in another terminal: ros2 launch roverrobotics_driver mini.launch.py"
    echo ""
    read -p "Press Enter to continue anyway, or Ctrl+C to exit..."
else
    echo "✓ Robot driver detected"
fi

# Start SLAM (which includes EKF)
echo "Starting SLAM (includes EKF)..."
ros2 launch roverrobotics_driver slam_launch.py &
SLAM_PID=$!

echo "SLAM started (PID: $SLAM_PID)"
echo ""
echo "Waiting 10 seconds for EKF and SLAM to initialize..."
sleep 10

echo ""
echo "=========================================="
echo "Running Diagnostics (Refreshing every 3s)"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run diagnostics in a loop
while true; do
    echo ""
    echo "=========================================="
    echo "EKF + SLAM Diagnostics - $(date +%H:%M:%S)"
    echo "=========================================="
    echo ""
    
    echo "1. Node Status:"
    if ros2 node list 2>/dev/null | grep -q ekf; then
        echo "   ✓ EKF node running"
    else
        echo "   ✗ EKF node NOT running"
    fi
    if ros2 node list 2>/dev/null | grep -q slam; then
        echo "   ✓ SLAM node running"
    else
        echo "   ✗ SLAM node NOT running"
    fi
    echo ""
    
    echo "2. Transform Status:"
    echo "   odom->base_link:"
    TRANSFORM_OUTPUT=$(timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1)
    if echo "$TRANSFORM_OUTPUT" | grep -q "At time"; then
        echo "      ✓ EXISTS"
        echo "$TRANSFORM_OUTPUT" | grep -E "(Translation|Rotation)" | head -2 | sed 's/^/         /'
    else
        echo "      ✗ NOT FOUND"
        ERROR_MSG=$(echo "$TRANSFORM_OUTPUT" | grep -i "error\|invalid\|not exist" | head -1)
        if [ ! -z "$ERROR_MSG" ]; then
            echo "         $ERROR_MSG"
        fi
    fi
    
    echo "   map->base_link:"
    MAP_TRANSFORM=$(timeout 0.5 ros2 run tf2_ros tf2_echo map base_link 2>&1)
    if echo "$MAP_TRANSFORM" | grep -q "At time"; then
        echo "      ✓ EXISTS"
    else
        echo "      ✗ NOT FOUND (SLAM may not have initialized yet)"
    fi
    echo ""
    
    echo "3. Topic Status:"
    if timeout 1 ros2 topic hz /imu/data 2>&1 | grep -q "average rate"; then
        RATE=$(timeout 1 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -1)
        echo "   ✓ /imu/data: $RATE"
    else
        echo "   ✗ /imu/data: NOT PUBLISHING"
    fi
    
    if timeout 1 ros2 topic hz /scan_filtered 2>&1 | grep -q "average rate"; then
        RATE=$(timeout 1 ros2 topic hz /scan_filtered 2>&1 | grep "average rate" | head -1)
        echo "   ✓ /scan_filtered: $RATE"
    else
        echo "   ✗ /scan_filtered: NOT PUBLISHING"
    fi
    
    if timeout 1 ros2 topic hz /tf 2>&1 | grep -q "average rate"; then
        RATE=$(timeout 1 ros2 topic hz /tf 2>&1 | grep "average rate" | head -1)
        echo "   ✓ /tf: $RATE"
    else
        echo "   ✗ /tf: NOT PUBLISHING"
    fi
    echo ""
    
    echo "4. SLAM Message Filter Status:"
    # Check for SLAM message filter errors in recent logs
    echo "   (Check terminal running SLAM for 'Message Filter dropping' messages)"
    echo ""
    
    echo "=========================================="
    echo "Next update in 3 seconds..."
    echo "=========================================="
    
    sleep 3
done

