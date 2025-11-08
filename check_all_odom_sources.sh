#!/bin/bash
# Check all odometry sources and transforms

cd /home/stickykeys/rover_workspace
source install/setup.bash 2>/dev/null || true

echo "=========================================="
echo "Checking All Odometry Sources"
echo "=========================================="
echo ""

echo "1. Odometry Topics:"
ros2 topic list 2>&1 | grep -i odom
echo ""

echo "2. Wheel Odometry (/odometry/wheels):"
if timeout 1 ros2 topic echo /odometry/wheels --once 2>&1 | grep -q "header"; then
    echo "   ✗ Wheel odometry IS being published"
    echo "   Frame: $(timeout 1 ros2 topic echo /odometry/wheels --once 2>&1 | grep frame_id | head -1)"
else
    echo "   ✓ Wheel odometry is NOT being published"
fi
echo ""

echo "3. EKF Filtered Odometry (/odometry/filtered):"
if timeout 1 ros2 topic echo /odometry/filtered --once 2>&1 | grep -q "header"; then
    echo "   ✓ EKF filtered odometry IS being published"
else
    echo "   ✗ EKF filtered odometry is NOT being published"
fi
echo ""

echo "4. Transform Sources (odom->base_link):"
echo "   Checking /tf for odom->base_link transforms..."
TF_OUTPUT=$(timeout 2 ros2 topic echo /tf --once 2>&1)
if echo "$TF_OUTPUT" | grep -q "frame_id.*odom.*base_link\|child_frame_id.*base_link"; then
    echo "   ✓ Transform found in /tf"
    echo "$TF_OUTPUT" | grep -E "frame_id|child_frame_id" | head -3
else
    echo "   ✗ No odom->base_link transform in /tf"
fi
echo ""

echo "5. EKF Nodes:"
EKF_NODES=$(ros2 node list 2>&1 | grep ekf || echo "")
EKF_COUNT=$(echo "$EKF_NODES" | grep -c ekf || echo "0")
if [ -z "$EKF_NODES" ]; then
    EKF_COUNT=0
fi
echo "   Found $EKF_COUNT EKF node(s)"
if [ "$EKF_COUNT" -gt 1 ]; then
    echo "   ⚠ WARNING: Multiple EKF nodes running!"
    echo "$EKF_NODES"
fi
echo ""

echo "6. Robot Driver TF Publishing:"
if ros2 param get /roverrobotics_driver publish_tf 2>&1 | grep -q "True"; then
    echo "   ✗ Robot driver IS publishing odom->base_link TF (from wheels)"
else
    echo "   ✓ Robot driver is NOT publishing odom->base_link TF"
fi
echo ""

echo "7. Direct Transform Check:"
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo "   ✓ odom->base_link transform EXISTS"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|Rotation" | head -2
else
    echo "   ✗ odom->base_link transform does NOT exist"
fi
echo ""

echo "=========================================="
echo "Summary:"
echo "=========================================="
echo "Wheel odometry topic exists but robot driver has publish_tf: false"
echo "Only EKF should be publishing odom->base_link transform"
echo ""

