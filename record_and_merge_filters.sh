#!/bin/bash

# Quick script to record obstacles and merge with existing filters
# Usage: ./record_and_merge_filters.sh [duration] [distance_threshold]

set -e

DURATION=${1:-5}
DISTANCE=${2:-0.3048}

echo "=========================================="
echo "Recording Obstacles for LiDAR Filtering"
echo "=========================================="
echo "Duration: ${DURATION} seconds"
echo "Distance threshold: ${DISTANCE} meters"
echo ""
echo "Make sure:"
echo "  1. Robot driver is running (mini.launch.py)"
echo "  2. Only the obstacles you want to filter are within ${DISTANCE}m"
echo "  3. Move other objects away temporarily"
echo ""
echo "Press Ctrl+C now if you need to adjust setup..."
sleep 3

# Record obstacles
echo ""
echo "Recording obstacles..."
python3 record_close_obstacles.py ${DURATION} ${DISTANCE}

# Find the most recent recording
LATEST_JSON=$(ls -t close_obstacles_*.json 2>/dev/null | head -1)

if [ -z "$LATEST_JSON" ]; then
    echo "ERROR: No recording file found!"
    exit 1
fi

echo ""
echo "Found recording: $LATEST_JSON"

# Extract angles
echo "Extracting unique angles..."
python3 extract_unique_angles.py "$LATEST_JSON"

LATEST_ANGLES="${LATEST_JSON%.json}_unique_angles.txt"

if [ ! -f "$LATEST_ANGLES" ]; then
    echo "ERROR: Angle extraction failed!"
    exit 1
fi

echo "✓ Extracted angles to: $LATEST_ANGLES"
echo ""

# Check for existing filters
EXISTING_FILTER="close_obstacles_1762028491_unique_angles.txt"

if [ -f "$EXISTING_FILTER" ]; then
    echo "Found existing filter: $EXISTING_FILTER"
    echo ""
    echo "Options:"
    echo "  1) Merge with existing filter (recommended - filters both robot parts AND new obstacles)"
    echo "  2) Use only new filter (only filter the obstacles you just recorded)"
    echo "  3) Keep both separate (manual control)"
    echo ""
    read -p "Choose option (1/2/3): " CHOICE
    
    case $CHOICE in
        1)
            MERGED_FILTER="combined_filters_$(date +%s).txt"
            echo ""
            echo "Merging filters..."
            python3 merge_angle_filters.py "$MERGED_FILTER" "$EXISTING_FILTER" "$LATEST_ANGLES"
            echo ""
            echo "=========================================="
            echo "✓ DONE! Merged filter created:"
            echo "  $MERGED_FILTER"
            echo ""
            echo "To use it for navigation:"
            echo "  ./run_hardware_navigation.sh --angles $MERGED_FILTER --explore"
            echo "=========================================="
            ;;
        2)
            echo ""
            echo "=========================================="
            echo "✓ DONE! Using new filter only:"
            echo "  $LATEST_ANGLES"
            echo ""
            echo "To use it for navigation:"
            echo "  ./run_hardware_navigation.sh --angles $LATEST_ANGLES --explore"
            echo "=========================================="
            ;;
        3)
            echo ""
            echo "=========================================="
            echo "✓ DONE! Filters available:"
            echo "  Existing: $EXISTING_FILTER"
            echo "  New:      $LATEST_ANGLES"
            echo ""
            echo "To merge them manually:"
            echo "  ./merge_angle_filters.sh combined.txt $EXISTING_FILTER $LATEST_ANGLES"
            echo ""
            echo "To use one of them:"
            echo "  ./run_hardware_navigation.sh --angles $LATEST_ANGLES --explore"
            echo "=========================================="
            ;;
        *)
            echo "Invalid choice. Filters are available separately."
            ;;
    esac
else
    echo "=========================================="
    echo "✓ DONE! First filter created:"
    echo "  $LATEST_ANGLES"
    echo ""
    echo "To use it for navigation:"
    echo "  ./run_hardware_navigation.sh --angles $LATEST_ANGLES --explore"
    echo "=========================================="
fi

echo ""
echo "To visualize the filtering in RViz:"
echo "  Terminal 1: ./run_filtered_lidar.sh $LATEST_ANGLES"
echo "  Terminal 2: ./view_lidar.sh"

