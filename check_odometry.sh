#!/bin/bash

# Check Rover Odometry
# Diagnostic script to verify odometry messages are being received

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Rover Odometry Diagnostic${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if robot driver is running
echo "1. Checking robot driver..."
if ros2 node list 2>/dev/null | grep -q "roverrobotics_driver"; then
    echo -e "   ${GREEN}✓ Robot driver is running${NC}"
else
    echo -e "   ${RED}✗ Robot driver NOT running${NC}"
    echo -e "   ${YELLOW}Start with: ros2 launch roverrobotics_driver mini.launch.py${NC}"
    exit 1
fi

echo ""
echo "2. Checking odometry topics..."

# Check /odometry/wheels (used by hardware navigation)
if ros2 topic list 2>/dev/null | grep -q "^/odometry/wheels$"; then
    echo -e "   ${GREEN}✓ /odometry/wheels topic exists${NC}"
    
    # Check publish rate
    echo ""
    echo "3. Checking odometry publish rate..."
    echo -e "   ${CYAN}Measuring for 5 seconds...${NC}"
    RATE=$(timeout 5s ros2 topic hz /odometry/wheels 2>/dev/null | grep "average rate" | awk '{print $3}')
    
    if [ -n "$RATE" ]; then
        echo -e "   ${GREEN}✓ Publishing at ~${RATE} Hz${NC}"
        if (( $(echo "$RATE > 5.0" | bc -l) )); then
            echo -e "   ${GREEN}✓ Rate is good (should be ~15 Hz)${NC}"
        else
            echo -e "   ${YELLOW}⚠ Rate is low (expected ~15 Hz)${NC}"
        fi
    else
        echo -e "   ${RED}✗ No messages received in 5 seconds${NC}"
    fi
else
    echo -e "   ${RED}✗ /odometry/wheels topic NOT found${NC}"
fi

# Check /odometry/filtered (sometimes used)
echo ""
if ros2 topic list 2>/dev/null | grep -q "^/odometry/filtered$"; then
    echo -e "   ${CYAN}ℹ /odometry/filtered also available${NC}"
fi

echo ""
echo "4. Checking odometry message content..."
echo -e "   ${CYAN}Getting one message...${NC}"
echo ""

ODO_MSG=$(timeout 3s ros2 topic echo /odometry/wheels --once 2>/dev/null)

if [ -n "$ODO_MSG" ]; then
    echo -e "${GREEN}✓ Odometry message received!${NC}"
    echo ""
    echo "Sample data:"
    echo "$ODO_MSG" | head -20
    echo "..."
    echo ""
    
    # Check if position is changing
    echo "5. Checking if odometry is updating..."
    echo -e "   ${CYAN}Sampling position over 2 seconds...${NC}"
    
    POS1=$(timeout 2s ros2 topic echo /odometry/wheels --once 2>/dev/null | grep -A2 "position:" | grep "x:" | awk '{print $2}')
    sleep 1
    POS2=$(timeout 2s ros2 topic echo /odometry/wheels --once 2>/dev/null | grep -A2 "position:" | grep "x:" | awk '{print $2}')
    
    if [ "$POS1" != "$POS2" ]; then
        echo -e "   ${GREEN}✓ Position is updating (robot moving or updating)${NC}"
    else
        echo -e "   ${YELLOW}⚠ Position not changing (robot stationary or odometry frozen)${NC}"
        echo -e "   ${YELLOW}Try moving robot with WASD controller to test${NC}"
    fi
else
    echo -e "${RED}✗ No odometry message received${NC}"
    echo -e "${YELLOW}Possible issues:${NC}"
    echo -e "   - Robot driver not publishing odometry"
    echo -e "   - Wrong topic name configured"
    echo -e "   - Communication issue with robot"
fi

echo ""
echo "6. Checking TF frames (needed for navigation)..."

# Check if odom and base_link frames exist
TF_FRAMES=$(timeout 2s ros2 run tf2_ros tf2_echo odom base_link 2>&1)

if echo "$TF_FRAMES" | grep -q "At time"; then
    echo -e "   ${GREEN}✓ TF transform odom → base_link exists${NC}"
else
    echo -e "   ${RED}✗ TF transform odom → base_link NOT found${NC}"
    echo -e "   ${YELLOW}This is required for navigation!${NC}"
fi

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Summary${NC}"
echo -e "${CYAN}========================================${NC}"

# Final summary
echo ""
ros2 topic list 2>/dev/null | grep -E "odom|cmd_vel" | while read topic; do
    echo "  $topic"
done

echo ""
echo -e "${CYAN}Navigation Configuration:${NC}"
echo -e "  Controller reads: ${GREEN}/odometry/wheels${NC}"
echo -e "  Controller writes: ${GREEN}/cmd_vel${NC}"
echo ""

echo -e "${CYAN}Quick Tests:${NC}"
echo -e "  View odometry: ${GREEN}ros2 topic echo /odometry/wheels${NC}"
echo -e "  Check rate: ${GREEN}ros2 topic hz /odometry/wheels${NC}"
echo -e "  View TF tree: ${GREEN}ros2 run tf2_tools view_frames${NC}"
echo -e "  Test motors: ${GREEN}./wasd_controller.sh${NC}"
echo ""

