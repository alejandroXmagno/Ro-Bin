#!/bin/bash

# Extract Unique Angles from Obstacle Recording
# Convenience script to extract unique angles from JSON obstacle recordings

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

if [ $# -eq 0 ]; then
    echo -e "${YELLOW}Usage: $0 <json_file>${NC}"
    echo ""
    echo "Example:"
    echo "  $0 close_obstacles_1762028491.json"
    echo ""
    echo "Or process the most recent recording:"
    echo "  $0 \$(ls -t close_obstacles_*.json | head -1)"
    exit 1
fi

JSON_FILE="$1"

if [ ! -f "$JSON_FILE" ]; then
    echo -e "${YELLOW}Error: File not found: $JSON_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}Extracting unique angles from: $JSON_FILE${NC}"
echo ""

python3 extract_unique_angles.py "$JSON_FILE"

# If successful, show the output file
if [ $? -eq 0 ]; then
    OUTPUT_FILE="${JSON_FILE%.json}_unique_angles.txt"
    echo ""
    echo -e "${GREEN}Preview of output file:${NC}"
    echo ""
    head -30 "$OUTPUT_FILE"
    echo ""
    echo -e "${GREEN}Full output saved to: $OUTPUT_FILE${NC}"
fi

