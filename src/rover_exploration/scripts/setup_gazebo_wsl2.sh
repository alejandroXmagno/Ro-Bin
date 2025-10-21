#!/bin/bash

# Setup Gazebo for WSL2
# This script sets environment variables needed for Gazebo to run in WSL2

echo "Setting up Gazebo environment for WSL2..."

# Force software rendering (no hardware acceleration)
export LIBGL_ALWAYS_SOFTWARE=1

# Override OpenGL version for compatibility
export MESA_GL_VERSION_OVERRIDE=3.3

# Disable indirect rendering
export LIBGL_ALWAYS_INDIRECT=0

# Disable SVGA 3D acceleration
export SVGA_VGPU10=0

# Ignition Gazebo resource path
export IGN_GAZEBO_RESOURCE_PATH=/usr/share/ignition/ignition-gazebo6

# Qt platform to avoid GUI issues
export QT_X11_NO_MITSHM=1

echo "✓ Environment configured for Gazebo in WSL2"
echo ""
echo "Environment variables set:"
echo "  LIBGL_ALWAYS_SOFTWARE=1"
echo "  MESA_GL_VERSION_OVERRIDE=3.3"
echo "  LIBGL_ALWAYS_INDIRECT=0"
echo "  SVGA_VGPU10=0"
echo ""
echo "You can now run:"
echo "  ros2 launch roverrobotics_gazebo mini_gazebo.launch.py"
echo ""
echo "Or for SLAM testing:"
echo "  ros2 launch rover_exploration simulation_autonomous_slam.launch.py"
echo ""



