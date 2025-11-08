#!/bin/bash

# Run Filtered LiDAR Scanner
# Filters out specific angles from LiDAR scans

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Filtered LiDAR Scanner${NC}"
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
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source the workspace
source install/setup.bash

# Get the angles file
ANGLES_FILE=""
if [ $# -eq 0 ]; then
    # Find the most recent unique angles file
    ANGLES_FILE=$(ls -t close_obstacles_*_unique_angles.txt 2>/dev/null | head -1)
    
    if [ -z "$ANGLES_FILE" ]; then
        # Try JSON files
        ANGLES_FILE=$(ls -t close_obstacles_*.json 2>/dev/null | head -1)
    fi
    
    if [ -z "$ANGLES_FILE" ]; then
        echo -e "${YELLOW}No obstacle recording files found.${NC}"
        echo ""
        echo "Usage: $0 <angles_file> [tolerance_degrees]"
        echo ""
        echo "Examples:"
        echo "  $0 close_obstacles_1762028491_unique_angles.txt"
        echo "  $0 close_obstacles_1762028491.json"
        echo "  $0 angles.txt 1.0  # with 1 degree tolerance"
        exit 1
    fi
    
    echo -e "${GREEN}Using most recent file: $ANGLES_FILE${NC}"
else
    ANGLES_FILE="$1"
fi

if [ ! -f "$ANGLES_FILE" ]; then
    echo -e "${YELLOW}Error: File not found: $ANGLES_FILE${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Starting filtered LiDAR scanner...${NC}"
echo "Input file: $ANGLES_FILE"
echo "Original scans: /scan"
echo "Filtered scans: /scan_filtered"
echo "Angle tolerance: ±2.0° (default)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the filtered scanner with 2 degree tolerance if no tolerance specified
if [ $# -eq 1 ]; then
    python3 filtered_lidar_scanner.py "$ANGLES_FILE" 2.0
else
    python3 filtered_lidar_scanner.py "$@"
fi

echo ""
echo -e "${GREEN}Filtered LiDAR scanner stopped.${NC}"

