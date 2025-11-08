#!/bin/bash
# Test EKF without starting navigation - just EKF and diagnostics
# This allows testing EKF transform publishing without any robot movement

cd /home/stickykeys/rover_workspace
source install/setup.bash

echo "=========================================="
echo "EKF-Only Test (No Navigation, No Movement)"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start EKF node only"
echo "  2. Monitor EKF status and transforms"
echo "  3. NOT start navigation (robot won't move)"
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

# Check if EKF is already running
if ros2 node list 2>/dev/null | grep -q ekf; then
    echo "⚠ EKF node is already running!"
    echo "   Not starting a new one to avoid conflicts"
    echo "   Using existing EKF node"
    EKF_PID=""
else
    # Start EKF only (no SLAM, no Nav2, no exploration)
    echo "Starting EKF node..."
    ros2 launch roverrobotics_driver robot_localizer.launch.py &
    EKF_PID=$!
    
    echo "EKF started (PID: $EKF_PID)"
    echo ""
    echo "Waiting 5 seconds for EKF to initialize..."
    sleep 5
fi

echo ""
echo "=========================================="
echo "Running Diagnostics (Refreshing every 2s)"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run diagnostics in a loop
while true; do
    echo ""
    echo "=========================================="
    echo "EKF Diagnostics - $(date +%H:%M:%S)"
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
    # First check if topic exists
    if ! ros2 topic list 2>/dev/null | grep -q "/imu/data"; then
        echo "   ✗ /imu/data topic does NOT exist!"
    else
        # Try to get a message (more reliable than hz for quick check)
        IMU_MSG=$(timeout 3 ros2 topic echo /imu/data --once 2>&1)
        if echo "$IMU_MSG" | grep -q "frame_id"; then
            echo "   ✓ IMU is publishing"
            # Try to get rate if possible
            RATE_CHECK=$(timeout 2 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -1)
            if [ ! -z "$RATE_CHECK" ]; then
                echo "      $RATE_CHECK"
            fi
        else
            echo "   ✗ IMU topic exists but no messages received"
            echo "   Checking if BNO055 node is running..."
            if ros2 node list 2>/dev/null | grep -q "bno055"; then
                echo "      ✓ BNO055 node is running"
                echo "      (May need a moment to start publishing)"
            else
                echo "      ✗ BNO055 node is NOT running"
            fi
        fi
    fi
    echo ""
    
    echo "3. Transform Status (odom->base_link):"
    TRANSFORM_OUTPUT=$(timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1)
    if echo "$TRANSFORM_OUTPUT" | grep -q "At time"; then
        echo "   ✓ Transform EXISTS"
        echo "$TRANSFORM_OUTPUT" | grep -E "(At time|Translation|Rotation)" | head -3 | sed 's/^/      /'
    else
        echo "   ✗ Transform does NOT exist"
        ERROR_MSG=$(echo "$TRANSFORM_OUTPUT" | grep -i "error\|invalid\|not exist" | head -1)
        if [ ! -z "$ERROR_MSG" ]; then
            echo "      Error: $ERROR_MSG"
        fi
    fi
    echo ""
    
    echo "4. EKF Subscriptions:"
    EKF_INFO=$(ros2 node info /ekf_filter_node 2>&1)
    if echo "$EKF_INFO" | grep -q "imu/data"; then
        echo "   ✓ EKF is subscribed to /imu/data"
        echo "$EKF_INFO" | grep "imu/data" | head -1 | sed 's/^/      /'
    else
        echo "   ✗ EKF is NOT subscribed to /imu/data"
    fi
    echo ""
    
    echo "5. EKF Parameters:"
    ros2 param get /ekf_filter_node publish_tf 2>&1 | grep -v "Getting" | sed 's/^/      /'
    ros2 param get /ekf_filter_node odom_frame 2>&1 | grep -v "Getting" | sed 's/^/      /'
    ros2 param get /ekf_filter_node base_link_frame 2>&1 | grep -v "Getting" | sed 's/^/      /'
    echo ""
    
    echo "6. Recent IMU Message:"
    IMU_MSG=$(timeout 1 ros2 topic echo /imu/data --once 2>&1)
    if echo "$IMU_MSG" | grep -q "header"; then
        echo "$IMU_MSG" | grep -E "(frame_id|orientation|angular_velocity)" | head -5 | sed 's/^/      /'
    else
        echo "      No IMU message received"
    fi
    echo ""
    
    echo "=========================================="
    echo "Next update in 2 seconds..."
    echo "=========================================="
    
    sleep 2
done

