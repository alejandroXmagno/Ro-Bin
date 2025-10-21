#!/bin/bash

# Complete LIDAR SLAM Demo for Mini Rover in Gazebo
# This script launches everything needed for SLAM

echo "=========================================="
echo "  Mini Rover LIDAR SLAM Demo - WSL2      "
echo "=========================================="
echo ""

# Set WSL2 compatible environment
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_INDIRECT=0
export SVGA_VGPU10=0

cd /home/lechonka/rover_workspace
source install/setup.bash 2>/dev/null

echo "Step 1: Launching Gazebo with Mini Rover..."
ros2 launch roverrobotics_gazebo mini_gazebo.launch.py &
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"
sleep 10

echo ""
echo "Step 2: Launching SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true \
  -r scan:=/scan \
  -r odom:=/odometry/wheels &
SLAM_PID=$!
echo "  SLAM PID: $SLAM_PID"
sleep 3

echo ""
echo "=========================================="
echo "✓ System Ready!"
echo "=========================================="
echo ""
echo "Gazebo Window: You should see the rover!"
echo ""
echo "NEXT STEPS:"
echo ""
echo "1. Open RViz2 (in a NEW terminal):"
echo "   cd /home/lechonka/rover_workspace"
echo "   source install/setup.bash"
echo "   ros2 run rviz2 rviz2"
echo ""
echo "2. In RViz2, add these displays:"
echo "   - Click 'Add' button"
echo "   - Add 'LaserScan' -> Topic: /scan"
echo "   - Add 'Map' -> Topic: /map"
echo "   - Add 'RobotModel'"
echo "   - Change 'Fixed Frame' to 'odom'"
echo ""
echo "3. Move the robot (in ANOTHER terminal):"
echo "   cd /home/lechonka/rover_workspace"
echo "   source install/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "   Use keyboard:"
echo "     i = forward"
echo "     , = backward"
echo "     j = turn left"
echo "     l = turn right"
echo "     k = stop"
echo ""
echo "4. Watch the map build in RViz2 as you drive!"
echo ""
echo "Press Ctrl+C here to stop everything"
echo ""

# Keep running and handle Ctrl+C
trap "echo ''; echo 'Stopping...'; kill $GAZEBO_PID $SLAM_PID 2>/dev/null; pkill -f 'ign gazebo'; pkill -f slam_toolbox; exit" INT TERM
wait



