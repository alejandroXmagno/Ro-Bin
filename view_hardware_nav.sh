#!/bin/bash

# View Hardware Navigation in RViz2
# Opens comprehensive visualization of LiDAR scans and path planning

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Hardware Navigation Visualization${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if navigation is running
if ! ros2 node list 2>/dev/null | grep -q "controller_server\|planner_server\|bt_navigator"; then
    echo -e "${YELLOW}Warning: Navigation stack not detected!${NC}"
    echo ""
    echo "Make sure you've started navigation first:"
    echo -e "  ${GREEN}./run_hardware_navigation.sh${NC}"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source the workspace
source install/setup.bash

echo -e "${GREEN}Opening RViz2 Navigation Visualization...${NC}"
echo ""
echo -e "${CYAN}Visualization Includes:${NC}"
echo "  🔴 RED points = Original LiDAR (/scan)"
echo "  🟢 GREEN points = Filtered LiDAR (/scan_filtered)"
echo "  🟢 GREEN line = Global path plan"
echo "  🟡 YELLOW line = Local path plan"
echo "  🗺️  Gray map = SLAM-generated map"
echo "  🎯 Orange marker = Current goal"
echo "  🤖 Robot model with footprint"
echo ""
echo -e "${CYAN}Tools Available:${NC}"
echo "  📍 '2D Pose Estimate' - Set robot's initial position"
echo "  🎯 '2D Goal Pose' - Send navigation goal"
echo "  📏 'Measure' - Measure distances"
echo ""

# Launch RViz
ros2 launch rover_exploration view_hardware_navigation.launch.py

echo ""
echo -e "${GREEN}RViz2 closed.${NC}"

