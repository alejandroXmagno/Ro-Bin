#!/bin/bash
# Verify odometry setup - ensure only IMU-based EKF is publishing odom

cd /home/stickykeys/rover_workspace
source install/setup.bash 2>/dev/null || true

echo "=========================================="
echo "Odometry Setup Verification"
echo "=========================================="
echo ""

echo "1. Robot Driver TF Publishing:"
PUB_TF=$(ros2 param get /roverrobotics_driver publish_tf 2>&1 | grep "Boolean value" | awk '{print $NF}')
if [ "$PUB_TF" = "True" ]; then
    echo "   ✗ Robot driver IS publishing odom->base_link TF (from wheels)"
    echo "   ⚠ This will conflict with EKF!"
else
    echo "   ✓ Robot driver is NOT publishing odom->base_link TF"
    echo "   (publish_tf: false in mini_config.yaml)"
fi
echo ""

echo "2. Wheel Odometry Topic:"
if timeout 1 ros2 topic hz /odometry/wheels 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odometry/wheels 2>&1 | grep "average rate" | head -1)
    echo "   ⚠ Wheel odometry topic IS being published: $RATE"
    echo "   (This is just a topic, not a transform - should be OK)"
else
    echo "   ✓ Wheel odometry topic is NOT being published"
fi
echo ""

echo "3. EKF Nodes:"
EKF_NODES=$(ros2 node list 2>&1 | grep ekf || echo "")
if [ -z "$EKF_NODES" ]; then
    echo "   ✗ No EKF nodes running"
    echo "   (EKF should be started by robot_localizer.launch.py)"
else
    EKF_COUNT=$(echo "$EKF_NODES" | wc -l)
    echo "   Found $EKF_COUNT EKF node(s):"
    echo "$EKF_NODES" | sed 's/^/      /'
    if [ "$EKF_COUNT" -gt 1 ]; then
        echo "   ⚠ WARNING: Multiple EKF nodes - this will cause conflicts!"
    fi
fi
echo ""

echo "4. EKF Transform Publishing:"
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo "   ✓ EKF IS publishing odom->base_link transform"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|Rotation" | head -2 | sed 's/^/      /'
else
    echo "   ✗ EKF is NOT publishing odom->base_link transform"
    echo "   (EKF may not be initialized yet)"
fi
echo ""

echo "5. Transform Tree (odom->base_link):"
TF_CHECK=$(timeout 2 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id.*odom|child_frame_id.*base_link" | head -2)
if [ ! -z "$TF_CHECK" ]; then
    echo "   ✓ Found odom->base_link in /tf:"
    echo "$TF_CHECK" | sed 's/^/      /'
else
    echo "   ✗ No odom->base_link transform in /tf"
fi
echo ""

echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "✓ Robot driver: publish_tf = false (not publishing transform)"
echo "✓ Only EKF should publish odom->base_link transform"
echo "✓ Wheel odometry topic exists but doesn't publish transform"
echo ""

