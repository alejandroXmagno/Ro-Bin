#!/bin/bash

# Hardware Navigation Stack Runner
# Complete script to run autonomous navigation on physical robot with filtered LiDAR

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Hardware Navigation Stack${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Parse arguments
ENABLE_EXPLORATION=false
ANGLES_FILE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --explore)
            ENABLE_EXPLORATION=true
            shift
            ;;
        --angles)
            ANGLES_FILE="$2"
            shift 2
            ;;
        *)
            ANGLES_FILE="$1"
            shift
            ;;
    esac
done

# Find angles file if not specified
if [ -z "$ANGLES_FILE" ]; then
    # Find the most recent unique angles file
    ANGLES_FILE=$(ls -t close_obstacles_*_unique_angles.txt 2>/dev/null | head -1)
    
    if [ -z "$ANGLES_FILE" ]; then
        # Try JSON files
        ANGLES_FILE=$(ls -t close_obstacles_*.json 2>/dev/null | head -1)
    fi
    
    if [ -z "$ANGLES_FILE" ]; then
        echo -e "${YELLOW}No angles file specified and none found automatically.${NC}"
        echo -e "${YELLOW}The navigation will use unfiltered LiDAR.${NC}"
        echo ""
        read -p "Continue without filtered LiDAR? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo -e "${GREEN}Using angles file: $ANGLES_FILE${NC}"
    fi
fi

# Check prerequisites
echo -e "${CYAN}Checking prerequisites...${NC}"

# Check if robot driver is running
if ! ros2 node list 2>/dev/null | grep -q "roverrobotics_driver"; then
    echo -e "${RED}ERROR: Robot driver not running!${NC}"
    echo ""
    echo "Please start the robot driver first:"
    echo -e "${GREEN}  pkill -f roverrobotics_ros2_driver || true${NC}"
    echo -e "${GREEN}  fuser -kv /dev/ttyACM0 || true${NC}"
    echo -e "${GREEN}  source install/setup.bash${NC}"
    echo -e "${GREEN}  ros2 launch roverrobotics_driver mini.launch.py${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Robot driver running${NC}"

# Check if original /scan topic exists
if ! ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo -e "${RED}ERROR: /scan topic not found!${NC}"
    echo "Make sure LiDAR is enabled in accessories.yaml"
    exit 1
fi
echo -e "${GREEN}✓ LiDAR /scan topic available${NC}"

# Start filtered LiDAR if angles file exists
if [ -n "$ANGLES_FILE" ] && [ -f "$ANGLES_FILE" ]; then
    # Check if filtered scanner is already running
    if ros2 node list 2>/dev/null | grep -q "filtered_lidar_scanner"; then
        echo -e "${GREEN}✓ Filtered LiDAR scanner already running${NC}"
    else
        echo -e "${YELLOW}Starting filtered LiDAR scanner...${NC}"
        python3 filtered_lidar_scanner.py "$ANGLES_FILE" &
        FILTER_PID=$!
        sleep 3
        
        if ps -p $FILTER_PID > /dev/null; then
            echo -e "${GREEN}✓ Filtered LiDAR scanner started (PID: $FILTER_PID)${NC}"
        else
            echo -e "${RED}ERROR: Failed to start filtered LiDAR scanner${NC}"
            exit 1
        fi
    fi
    
    # Verify filtered topic
    sleep 2
    if ros2 topic list 2>/dev/null | grep -q "/scan_filtered"; then
        echo -e "${GREEN}✓ /scan_filtered topic available${NC}"
    else
        echo -e "${RED}ERROR: /scan_filtered topic not found${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}⚠ No filtered LiDAR - using original /scan${NC}"
fi

# Source workspace
source install/setup.bash

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Configuration:${NC}"
echo -e "${CYAN}========================================${NC}"
echo -e "  Robot Driver: ${GREEN}Running${NC}"
echo -e "  LiDAR Topic: ${GREEN}/scan_filtered${NC}"
if [ "$ENABLE_EXPLORATION" = true ]; then
    echo -e "  Autonomous Exploration: ${GREEN}Enabled${NC}"
else
    echo -e "  Autonomous Exploration: ${YELLOW}Disabled${NC}"
    echo -e "  ${YELLOW}(Use --explore flag to enable)${NC}"
fi
echo -e "${CYAN}========================================${NC}"
echo ""

echo -e "${GREEN}Starting navigation stack...${NC}"
echo ""
echo "You can:"
echo "  1. Use RViz to set navigation goals (2D Nav Goal)"
echo "  2. Watch the robot navigate autonomously"
echo "  3. Press Ctrl+C to stop"
echo ""

# Launch navigation stack
if [ "$ENABLE_EXPLORATION" = true ]; then
    ros2 launch rover_exploration hardware_navigation_stack.launch.py enable_exploration:=true
else
    ros2 launch rover_exploration hardware_navigation_stack.launch.py enable_exploration:=false
fi

# Cleanup on exit
echo ""
echo -e "${GREEN}Navigation stack stopped.${NC}"

if [ -n "$FILTER_PID" ]; then
    echo -e "${YELLOW}Stopping filtered LiDAR scanner...${NC}"
    kill $FILTER_PID 2>/dev/null
fi

