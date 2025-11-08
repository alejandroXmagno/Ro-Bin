#!/bin/bash

# Simple script to run IMU monitor in a separate terminal
# Usage: ./run_imu_monitor.sh

cd "$(dirname "$0")"
source install/setup.bash
ros2 launch roverrobotics_driver imu_monitor.launch.py

