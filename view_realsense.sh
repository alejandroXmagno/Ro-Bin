#!/bin/bash

# View RealSense camera streams on physical robot

echo "=========================================="
echo "RealSense Camera Viewer"
echo "=========================================="
echo ""
echo "Starting viewer..."
echo ""
echo "Controls:"
echo "  d - Toggle depth view"
echo "  i - Toggle infrared view"
echo "  c - Cycle depth colormap"
echo "  q or ESC - Quit"
echo ""

python3 view_realsense.py

