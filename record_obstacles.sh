#!/bin/bash

# Record Close Obstacles Script
# Records LiDAR readings less than 1 foot away over a 5-second period

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Close Obstacle Recorder${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

# Check if robot driver or simulation is running
if ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo -e "${YELLOW}Warning: /scan topic not found!${NC}"
    echo -e "${YELLOW}Make sure the robot driver or simulation is running.${NC}"
    echo ""
    echo "For physical robot:"
    echo -e "  ${GREEN}ros2 launch roverrobotics_driver mini.launch.py${NC}"
    echo ""
    echo "For simulation:"
    echo -e "  ${GREEN}ros2 launch rover_exploration simulation_autonomous_slam.launch.py${NC}"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source the workspace
source install/setup.bash

echo -e "${GREEN}Starting obstacle recording...${NC}"
echo "Duration: ${1:-5} seconds"
echo "Distance threshold: ${2:-0.3048}m (1 foot)"
echo ""

# Run the Python script
python3 record_close_obstacles.py "$@"

echo ""
echo -e "${GREEN}Recording complete!${NC}"

