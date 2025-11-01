#!/bin/bash

# View LiDAR from Physical Robot
# This script launches RViz to visualize LiDAR data from the hardware robot

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}LiDAR Hardware Visualization${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

# Check if robot driver is running
if ! ros2 node list 2>/dev/null | grep -q "roverrobotics_driver"; then
    echo -e "${YELLOW}Warning: Robot driver not detected!${NC}"
    echo -e "${YELLOW}Make sure you've started the robot driver first:${NC}"
    echo ""
    echo -e "  ${GREEN}pkill -f roverrobotics_ros2_driver || true${NC}"
    echo -e "  ${GREEN}fuser -kv /dev/ttyACM0 || true${NC}"
    echo -e "  ${GREEN}source install/setup.bash${NC}"
    echo -e "  ${GREEN}ros2 launch roverrobotics_driver mini.launch.py${NC}"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source the workspace
source install/setup.bash

echo -e "${GREEN}Starting LiDAR visualization...${NC}"
echo ""

# Launch RViz for LiDAR visualization
ros2 launch rover_exploration view_lidar_hardware.launch.py

echo ""
echo -e "${GREEN}LiDAR visualization closed.${NC}"

