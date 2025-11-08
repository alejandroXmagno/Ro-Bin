#!/bin/bash

# Quick Navigation Test Script
# Tests if filtered LiDAR automatically starts

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Quick Navigation Test${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check for angles files
ANGLES_FILE=$(ls -t close_obstacles_*_unique_angles.txt 2>/dev/null | head -1)

if [ -z "$ANGLES_FILE" ]; then
    ANGLES_FILE=$(ls -t close_obstacles_*.json 2>/dev/null | head -1)
fi

echo "Checking for obstacle recordings..."
if [ -n "$ANGLES_FILE" ]; then
    echo -e "${GREEN}✓ Found: $ANGLES_FILE${NC}"
    echo ""
    echo "Test 1: Auto-detection"
    echo -e "  ${CYAN}Command: ./run_hardware_navigation.sh --explore${NC}"
    echo -e "  ${GREEN}Expected: Will automatically use $ANGLES_FILE${NC}"
    echo ""
    echo "Test 2: Explicit file"
    echo -e "  ${CYAN}Command: ./run_hardware_navigation.sh --angles $ANGLES_FILE --explore${NC}"
    echo -e "  ${GREEN}Expected: Will use specified file${NC}"
else
    echo -e "${YELLOW}✗ No angles files found${NC}"
    echo ""
    echo "To create one:"
    echo -e "  ${GREEN}./record_obstacles.sh${NC}"
    echo ""
    echo "Then navigation will automatically use it!"
fi

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Filtered LiDAR Status Check${NC}"
echo -e "${CYAN}========================================${NC}"

# Check if system is ready
if ros2 node list 2>/dev/null | grep -q "roverrobotics_driver"; then
    echo -e "${GREEN}✓ Robot driver running${NC}"
else
    echo -e "${YELLOW}✗ Robot driver not running${NC}"
fi

if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo -e "${GREEN}✓ /scan topic available${NC}"
else
    echo -e "${YELLOW}✗ /scan topic not available${NC}"
fi

if ros2 node list 2>/dev/null | grep -q "filtered_lidar_scanner"; then
    echo -e "${GREEN}✓ Filtered LiDAR already running${NC}"
    
    if ros2 topic list 2>/dev/null | grep -q "^/scan_filtered$"; then
        echo -e "${GREEN}✓ /scan_filtered topic available${NC}"
    else
        echo -e "${YELLOW}✗ /scan_filtered topic not available${NC}"
    fi
else
    echo -e "${CYAN}• Filtered LiDAR not running (will auto-start)${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Ready to test navigation!${NC}"
echo -e "${GREEN}========================================${NC}"

