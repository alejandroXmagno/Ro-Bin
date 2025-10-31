#!/bin/bash

echo "========================================="
echo "  Gazebo WSL2 Setup Script"
echo "========================================="
echo ""

# Backup .bashrc
echo "📦 Creating backup of ~/.bashrc..."
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
echo "✅ Backup created"
echo ""

# Add environment variables if not already present
if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc; then
    echo "📝 Adding environment variables to ~/.bashrc..."
    cat >> ~/.bashrc << 'EOF'

# ====================================
# Gazebo/Ignition Gazebo WSL2 Fix
# Software rendering for WSL2 compatibility
# ====================================
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export OGRE_RTT_MODE=Copy

EOF
    echo "✅ Environment variables added to ~/.bashrc"
else
    echo "✅ Environment variables already in ~/.bashrc (skipping)"
fi

echo ""
echo "========================================="
echo "  Setup Complete!"
echo "========================================="
echo ""
echo "⚠️  IMPORTANT NEXT STEPS:"
echo ""
echo "1. CLOSE this terminal window completely"
echo "2. OPEN A NEW terminal window"
echo "3. Run the following commands:"
echo ""
echo "   cd ~/rover_workspace"
echo "   source install/setup.bash"
echo "   ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py"
echo ""
echo "========================================="
echo ""
echo "📖 For more info, see: WSL2_GAZEBO_FIX.md"
echo ""

