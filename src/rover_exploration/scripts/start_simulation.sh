#!/bin/bash

# Start Autonomous Exploration in Gazebo Simulation
# This script launches the full SLAM-based autonomous exploration system in simulation

echo "=========================================="
echo " Rover Simulation Exploration Launcher   "
echo "=========================================="
echo ""

# Source the workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

echo "Sourcing workspace: $WORKSPACE_DIR"
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "Checking prerequisites..."

# Check if required nodes exist
if ! ros2 pkg list | grep -q "rover_exploration"; then
    echo "ERROR: rover_exploration package not found!"
    echo "Please build the workspace first:"
    echo "  cd $WORKSPACE_DIR"
    echo "  colcon build --packages-select rover_exploration"
    exit 1
fi

if ! ros2 pkg list | grep -q "roverrobotics_gazebo"; then
    echo "ERROR: roverrobotics_gazebo package not found!"
    exit 1
fi

echo "✓ All required packages found"
echo ""

# Default parameters
WORLD="warehouse"
SLAM_PARAMS=""
NAV_PARAMS=""
EXPLORATION_PARAMS=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)
            WORLD="$2"
            shift 2
            ;;
        --slam-params)
            SLAM_PARAMS="slam_params_file:=$2"
            shift 2
            ;;
        --nav-params)
            NAV_PARAMS="nav_params_file:=$2"
            shift 2
            ;;
        --exploration-params)
            EXPLORATION_PARAMS="exploration_params_file:=$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --world NAME               Gazebo world to load (default: warehouse)"
            echo "  --slam-params FILE         Custom SLAM parameters file"
            echo "  --nav-params FILE          Custom navigation parameters file"
            echo "  --exploration-params FILE  Custom exploration parameters file"
            echo "  --help                     Show this help message"
            echo ""
            echo "Available worlds:"
            echo "  - warehouse (default)"
            echo "  - maze"
            echo "  - office"
            echo ""
            echo "Example:"
            echo "  $0 --world maze"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Starting simulation exploration with:"
echo "  World: $WORLD"
[[ -n "$SLAM_PARAMS" ]] && echo "  SLAM params: $SLAM_PARAMS"
[[ -n "$NAV_PARAMS" ]] && echo "  Nav params: $NAV_PARAMS"
[[ -n "$EXPLORATION_PARAMS" ]] && echo "  Exploration params: $EXPLORATION_PARAMS"
echo ""

echo "=========================================="
echo "  Gazebo will launch in a moment...      "
echo "  Wait for 'Exploration started' message "
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop at any time"
echo ""
sleep 2

# Build launch command
LAUNCH_CMD="ros2 launch rover_exploration simulation_autonomous_slam.launch.py world:=$WORLD"
[[ -n "$SLAM_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $SLAM_PARAMS"
[[ -n "$NAV_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $NAV_PARAMS"
[[ -n "$EXPLORATION_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $EXPLORATION_PARAMS"

echo "Executing: $LAUNCH_CMD"
echo ""

eval $LAUNCH_CMD



