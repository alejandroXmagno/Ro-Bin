#!/bin/bash

# Start Autonomous Exploration on Real Robot
# This script launches the full SLAM-based autonomous exploration system

echo "=========================================="
echo "  Rover Autonomous Exploration Launcher  "
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

if ! ros2 pkg list | grep -q "roverrobotics_driver"; then
    echo "ERROR: roverrobotics_driver package not found!"
    exit 1
fi

echo "✓ All required packages found"
echo ""

# Check for custom parameters
USE_SIM_TIME="false"
SLAM_PARAMS=""
NAV_PARAMS=""
EXPLORATION_PARAMS=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --sim)
            USE_SIM_TIME="true"
            shift
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
            echo "  --sim                      Use simulation time"
            echo "  --slam-params FILE         Custom SLAM parameters file"
            echo "  --nav-params FILE          Custom navigation parameters file"
            echo "  --exploration-params FILE  Custom exploration parameters file"
            echo "  --help                     Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --nav-params my_nav_params.yaml"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Starting autonomous exploration with:"
echo "  Use sim time: $USE_SIM_TIME"
[[ -n "$SLAM_PARAMS" ]] && echo "  SLAM params: $SLAM_PARAMS"
[[ -n "$NAV_PARAMS" ]] && echo "  Nav params: $NAV_PARAMS"
[[ -n "$EXPLORATION_PARAMS" ]] && echo "  Exploration params: $EXPLORATION_PARAMS"
echo ""

echo "=========================================="
echo "  IMPORTANT: Robot will start moving!    "
echo "  Make sure the area is clear!           "
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop at any time"
echo ""
sleep 3

# Build launch command
LAUNCH_CMD="ros2 launch rover_exploration full_autonomous_slam.launch.py use_sim_time:=$USE_SIM_TIME"
[[ -n "$SLAM_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $SLAM_PARAMS"
[[ -n "$NAV_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $NAV_PARAMS"
[[ -n "$EXPLORATION_PARAMS" ]] && LAUNCH_CMD="$LAUNCH_CMD $EXPLORATION_PARAMS"

echo "Executing: $LAUNCH_CMD"
echo ""

eval $LAUNCH_CMD



