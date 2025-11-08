#!/bin/bash

# Stream RealSense camera to web browser
# View from any device on your network

echo "=========================================="
echo "RealSense Web Streamer"
echo "=========================================="
echo ""

# Check if Flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "Installing Flask..."
    pip3 install flask --user
    echo ""
fi

echo "Starting web stream..."
echo ""
echo "Once started, open a web browser to:"
echo "  http://<jetson-ip>:5000/"
echo ""

python3 stream_realsense_web.py


