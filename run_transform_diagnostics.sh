#!/bin/bash

# Simple script to run transform diagnostics in a separate terminal
# Usage: ./run_transform_diagnostics.sh

cd "$(dirname "$0")"
source install/setup.bash
ros2 launch roverrobotics_driver transform_diagnostics.launch.py

