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

# Find angles file if not specified - ALWAYS try to find one
if [ -z "$ANGLES_FILE" ]; then
    # Find the most recent unique angles file
    ANGLES_FILE=$(ls -t close_obstacles_*_unique_angles.txt 2>/dev/null | head -1)
    
    if [ -z "$ANGLES_FILE" ]; then
        # Try JSON files
        ANGLES_FILE=$(ls -t close_obstacles_*.json 2>/dev/null | head -1)
    fi
    
    if [ -z "$ANGLES_FILE" ]; then
        echo -e "${YELLOW}⚠ No angles file found - navigation will use unfiltered LiDAR${NC}"
        echo -e "${YELLOW}Tip: Run './record_obstacles.sh' to create one${NC}"
        echo ""
        echo -e "${CYAN}Continuing with unfiltered LiDAR in 3 seconds...${NC}"
        sleep 3
    else
        echo -e "${GREEN}✓ Auto-detected angles file: $ANGLES_FILE${NC}"
    fi
fi

# Check prerequisites
echo -e "${CYAN}Checking prerequisites...${NC}"

# Check if robot driver is running (check for node or topics)
if ! ros2 node list 2>/dev/null | grep -q "roverrobotics_driver" && ! ros2 topic list 2>/dev/null | grep -qE "^/scan$|/scan_filtered"; then
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

# Check if LiDAR is available (either /scan or /scan_filtered)
# The auto standoff filter (integrated into mini.launch.py) subscribes to /scan and publishes /scan_filtered
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo -e "${GREEN}✓ LiDAR /scan topic available${NC}"
elif ros2 topic list 2>/dev/null | grep -q "/scan_filtered"; then
    echo -e "${GREEN}✓ LiDAR /scan_filtered topic available (auto filter running)${NC}"
else
    echo -e "${RED}ERROR: No LiDAR topics found!${NC}"
    echo "Make sure LiDAR is enabled in accessories.yaml and robot driver is running"
    exit 1
fi

# Check if auto standoff filter is running (integrated into mini.launch.py)
sleep 2
if ros2 node list 2>/dev/null | grep -q "auto_standoff_filter"; then
    echo -e "${GREEN}✓ Auto standoff filter running${NC}"
    
    # Verify filtered topic
    if ros2 topic list 2>/dev/null | grep -q "/scan_filtered"; then
        echo -e "${GREEN}✓ /scan_filtered topic available${NC}"
    else
        echo -e "${YELLOW}⚠ /scan_filtered not yet available, waiting...${NC}"
        sleep 3
        if ros2 topic list 2>/dev/null | grep -q "/scan_filtered"; then
            echo -e "${GREEN}✓ /scan_filtered topic now available${NC}"
        else
            echo -e "${YELLOW}⚠ /scan_filtered still not available - continuing anyway${NC}"
            echo -e "${YELLOW}  (Auto filter may still be recording standoffs)${NC}"
        fi
    fi
else
    echo -e "${YELLOW}⚠ Auto standoff filter not detected${NC}"
    echo -e "${YELLOW}  Make sure mini.launch.py is running${NC}"
    echo -e "${YELLOW}  Navigation will use /scan (unfiltered) if available${NC}"
fi

# Source workspace
source install/setup.bash

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Configuration:${NC}"
echo -e "${CYAN}========================================${NC}"
echo -e "  Robot Driver: ${GREEN}Running${NC}"
if [ -n "$ANGLES_FILE" ] && [ -f "$ANGLES_FILE" ]; then
    echo -e "  LiDAR Filtering: ${GREEN}Active (±2°)${NC}"
    echo -e "  LiDAR Topic: ${GREEN}/scan_filtered${NC}"
else
    echo -e "  LiDAR Filtering: ${YELLOW}Disabled${NC}"
    echo -e "  LiDAR Topic: ${YELLOW}/scan (unfiltered)${NC}"
fi
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

