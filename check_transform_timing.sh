#!/bin/bash

echo "=========================================="
echo "Transform Timing Diagnostic"
echo "=========================================="
echo ""

source install/setup.bash 2>/dev/null || true

echo "1. Checking transform availability..."
echo "   Testing odom->base_link transform:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10 || echo "   ✗ Transform not available"
echo ""

echo "2. Checking transform publishing rate..."
echo "   Listening to /tf for 3 seconds..."
timeout 3 ros2 topic hz /tf 2>&1 | grep -E "average rate|no new messages" || echo "   (Could not determine rate)"
echo ""

echo "3. Checking LiDAR scan rate..."
echo "   Listening to /scan_filtered for 3 seconds..."
timeout 3 ros2 topic hz /scan_filtered 2>&1 | grep -E "average rate|no new messages" || echo "   (Could not determine rate)"
echo ""

echo "4. Checking transform timestamps vs scan timestamps..."
echo "   Getting latest transform timestamp:"
LATEST_TF=$(timeout 1 ros2 topic echo /tf --once 2>/dev/null | grep -A 5 "child_frame_id: base_link" | grep "sec:" | head -1 | awk '{print $2}')
if [ ! -z "$LATEST_TF" ]; then
    echo "   Latest transform timestamp: $LATEST_TF"
else
    echo "   (Could not get transform timestamp)"
fi

echo ""
echo "5. Checking EKF node status..."
if ros2 node list 2>/dev/null | grep -q ekf; then
    echo "   ✓ EKF node is running"
    ros2 node info /ekf_filter_node 2>/dev/null | grep -E "Subscribers|Publishers" | head -4
else
    echo "   ✗ EKF node is NOT running!"
fi
echo ""

echo "6. Checking if transforms are being published..."
TF_COUNT=$(timeout 2 ros2 topic echo /tf --once 2>/dev/null | grep -c "child_frame_id" || echo "0")
if [ "$TF_COUNT" -gt "0" ]; then
    echo "   ✓ Transforms are being published ($TF_COUNT frames found)"
else
    echo "   ✗ No transforms found!"
fi
echo ""

echo "=========================================="

