#!/bin/bash

# Save the currently generated map from SLAM
# This script saves the map to a specified location

echo "=========================================="
echo "       Rover Map Saver Utility           "
echo "=========================================="
echo ""

# Default output location
OUTPUT_DIR="${HOME}/rover_maps"
MAP_NAME="explored_map"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --name)
            MAP_NAME="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --output-dir DIR   Directory to save the map (default: ~/rover_maps)"
            echo "  --name NAME        Name of the map file (default: explored_map)"
            echo "  --help             Show this help message"
            echo ""
            echo "The map will be saved as:"
            echo "  {output-dir}/{name}.yaml"
            echo "  {output-dir}/{name}.pgm"
            echo ""
            echo "Example:"
            echo "  $0 --name my_warehouse_map"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Check if the map topic is available
echo "Checking if map is being published..."
if ! ros2 topic list | grep -q "/map"; then
    echo "ERROR: /map topic not found!"
    echo "Make sure SLAM is running before saving the map."
    exit 1
fi

echo "✓ Map topic found"
echo ""

MAP_PATH="${OUTPUT_DIR}/${MAP_NAME}"

echo "Saving map to: $MAP_PATH"
echo ""
echo "This may take a few seconds..."
echo ""

# Save the map
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Map saved successfully!"
    echo "=========================================="
    echo ""
    echo "Files created:"
    echo "  - ${MAP_PATH}.yaml  (map metadata)"
    echo "  - ${MAP_PATH}.pgm   (map image)"
    echo ""
    echo "You can view the map with:"
    echo "  eog ${MAP_PATH}.pgm"
    echo ""
    echo "To use this map for localization:"
    echo "  Edit your launch file to use: ${MAP_PATH}.yaml"
else
    echo ""
    echo "ERROR: Failed to save map!"
    echo "Check that nav2_map_server is installed and the map topic is valid."
    exit 1
fi



