#!/bin/bash

# Test SLAM in Headless Gazebo Mode (WSL2 Compatible)
# This runs Gazebo server without GUI and you visualize in RViz2

echo "=========================================="
echo "  SLAM Testing - Headless Gazebo Mode    "
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Run Gazebo server (no GUI)"
echo "  2. Spawn the rover with LIDAR"
echo "  3. Start SLAM Toolbox"
echo "  4. You visualize in RViz2"
echo ""
echo "Press Ctrl+C to stop"
echo ""
sleep 2

# Set environment for WSL2 compatibility
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_INDIRECT=0

# Source workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null

echo "Starting Gazebo server (headless)..."
echo ""

# Start Gazebo server with empty world in background
ign gazebo -s -r empty.sdf &
GAZEBO_PID=$!
echo "Gazebo server PID: $GAZEBO_PID"
sleep 5

echo ""
echo "Spawning rover..."
# Spawn the robot
ros2 run ros_gz_sim create -world empty -file $(ros2 pkg prefix roverrobotics_description)/share/roverrobotics_description/urdf/mini.urdf -name mini_rover -x 0 -y 0 -z 0.2 &
SPAWN_PID=$!
sleep 3

echo ""
echo "Starting ROS-Gazebo bridges..."
# Bridge topics
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock &
sleep 2

echo ""
echo "Starting SLAM..."
# Start SLAM
ros2 launch roverrobotics_driver slam_launch.py use_sim_time:=true &
SLAM_PID=$!

echo ""
echo "=========================================="
echo "✓ System Ready!"
echo "=========================================="
echo ""
echo "Open RViz2 in another terminal:"
echo "  source install/setup.bash"
echo "  ros2 run rviz2 rviz2"
echo ""
echo "Add these displays in RViz2:"
echo "  - Map (/map)"
echo "  - LaserScan (/scan)"
echo "  - RobotModel"
echo "  - TF"
echo ""
echo "Move the robot with:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for Ctrl+C
trap "echo 'Stopping...'; kill $GAZEBO_PID $SPAWN_PID $SLAM_PID 2>/dev/null; pkill -P $$; exit" INT TERM
wait



