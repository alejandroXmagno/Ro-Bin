#!/bin/bash
# Standalone EKF test script - runs EKF and diagnostics without navigation
# This allows testing EKF transform publishing without the robot moving

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "Standalone EKF Test (No Navigation)"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start robot driver (if not running)"
echo "  2. Start EKF node"
echo "  3. Monitor EKF status and transforms"
echo "  4. NOT start navigation (robot won't move)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Check if robot driver is running (check for node or topics)
if ! ros2 node list 2>/dev/null | grep -q "roverrobotics_driver" && ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "⚠ Robot driver not detected. Make sure roverrobotics_driver/mini.launch is running first."
    echo "   Run: ros2 launch roverrobotics_driver mini.launch.py"
    echo ""
    read -p "Press Enter to continue anyway, or Ctrl+C to exit..."
else
    echo "✓ Robot driver detected"
fi

# Start EKF only (no SLAM, no Nav2)
echo "Starting EKF node..."
ros2 launch roverrobotics_driver robot_localizer.launch.py &
EKF_PID=$!

echo "EKF started (PID: $EKF_PID)"
echo ""
echo "Waiting 5 seconds for EKF to initialize..."
sleep 5

echo ""
echo "=========================================="
echo "Running Diagnostics..."
echo "=========================================="
echo ""

# Run diagnostics in a loop
while true; do
    echo ""
    echo "=========================================="
    echo "EKF Diagnostics (Refreshing every 2 seconds)"
    echo "Press Ctrl+C to stop"
    echo "=========================================="
    echo ""
    
    echo "1. EKF Node Status:"
    if ros2 node list 2>/dev/null | grep -q ekf; then
        echo "   ✓ EKF node is running"
        ros2 node list | grep ekf
    else
        echo "   ✗ EKF node is NOT running!"
    fi
    echo ""
    
    echo "2. IMU Data:"
    if timeout 1 ros2 topic hz /imu/data 2>&1 | grep -q "average rate"; then
        echo "   ✓ IMU is publishing"
        timeout 1 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -1
    else
        echo "   ✗ IMU is NOT publishing!"
    fi
    echo ""
    
    echo "3. Transform Status:"
    if timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
        echo "   ✓ odom->base_link transform EXISTS"
        timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "(At time|Translation|Rotation)" | head -3
    else
        echo "   ✗ odom->base_link transform does NOT exist"
        ERROR_MSG=$(timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -i "error\|invalid\|not exist" | head -1)
        if [ ! -z "$ERROR_MSG" ]; then
            echo "   Error: $ERROR_MSG"
        fi
    fi
    echo ""
    
    echo "4. EKF Subscriptions:"
    if ros2 node info /ekf_filter_node 2>/dev/null | grep -q "imu/data"; then
        echo "   ✓ EKF is subscribed to /imu/data"
    else
        echo "   ✗ EKF is NOT subscribed to /imu/data"
    fi
    echo ""
    
    echo "5. Recent IMU Message (last one):"
    timeout 1 ros2 topic echo /imu/data --once 2>&1 | grep -E "(header|orientation|angular_velocity)" | head -5
    echo ""
    
    echo "=========================================="
    echo "Next update in 2 seconds..."
    echo "=========================================="
    
    sleep 2
done

