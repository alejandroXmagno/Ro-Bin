# WSL2 Gazebo Fix Guide

This guide helps you fix Gazebo/Ignition Gazebo rendering issues in WSL2.

## The Problem

WSL2 has limited GPU/OpenGL support, causing Gazebo to crash with errors like:
```
OGRE EXCEPTION(9:UnimplementedException): in GL3PlusTextureGpu::copyTo
```

---

## ✅ Solution 1: Software Rendering (RECOMMENDED)

This forces Gazebo to use CPU-based software rendering instead of GPU.

### Step 1: Add Environment Variables

These are **already added to your `~/.bashrc`**, but here's what they do:

```bash
export LIBGL_ALWAYS_SOFTWARE=1        # Force software rendering
export GALLIUM_DRIVER=llvmpipe        # Use LLVM-based software rasterizer
export MESA_GL_VERSION_OVERRIDE=3.3   # Override OpenGL version
export OGRE_RTT_MODE=Copy            # Change Ogre render-to-texture mode
```

### Step 2: Open a NEW Terminal

**IMPORTANT:** Close your current terminal and open a **brand new** terminal window. This ensures the environment variables are loaded from `.bashrc`.

### Step 3: Launch Gazebo

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse
```

**Performance Note:** Software rendering is slower but more stable in WSL2.

---

## ✅ Solution 2: WSLg with X11 (If you have WSL 2.0+)

If you're on Windows 11 with WSLg (GUI support), you might need additional setup:

### Check WSL Version
```bash
wsl --version
```

### If WSLg is Available
```bash
# Add to ~/.bashrc
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=0
```

Then restart your terminal.

---

## ✅ Solution 3: Headless Mode (No GUI)

If you don't need to see Gazebo's GUI, run it in headless mode:

### Launch Gazebo Headless

```bash
cd ~/rover_workspace
source install/setup.bash

# Set headless environment
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Launch with headless flag
ign gazebo -s warehouse.sdf --headless-rendering
```

Or modify your launch file to include `headless:=true`.

---

## ✅ Solution 4: Use VcXsrv/X410 (External X Server)

If solutions 1-3 don't work, use an external X server on Windows.

### Step 1: Install VcXsrv on Windows

Download from: https://sourceforge.net/projects/vcxsrv/

### Step 2: Launch VcXsrv

- Start XLaunch
- Select "Multiple windows"
- Display number: 0
- **Important:** Check "Disable access control"

### Step 3: Configure WSL2

```bash
# Get Windows host IP
export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0.0

# Add to ~/.bashrc
echo 'export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '"'"'{print $2}'"'"'):0.0' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
```

### Step 4: Test

```bash
source ~/.bashrc
xclock  # Should show a clock window
```

### Step 5: Launch Gazebo

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py
```

---

## 🔍 Troubleshooting

### Check Current Environment Variables

```bash
echo "LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"
echo "GALLIUM_DRIVER=$GALLIUM_DRIVER"
echo "DISPLAY=$DISPLAY"
```

### Test OpenGL

```bash
glxinfo | grep "OpenGL version"
```

If this command fails, install:
```bash
sudo apt install mesa-utils
```

### Check Gazebo Version

```bash
ign gazebo --version
# or
gz sim --version
```

### Screen Size Warning

If you see "your 131072x1 screen size is bogus", this is usually harmless but indicates display issues.

**Fix:**
```bash
export DISPLAY=:0
xrandr  # Check available displays
```

---

## 📝 Quick Setup Script

Save this as `setup_gazebo_wsl2.sh`:

```bash
#!/bin/bash

echo "Setting up Gazebo for WSL2..."

# Backup .bashrc
cp ~/.bashrc ~/.bashrc.backup

# Add environment variables if not already present
if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc; then
    cat >> ~/.bashrc << 'EOF'

# Gazebo WSL2 Fix
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export OGRE_RTT_MODE=Copy
EOF
    echo "✅ Environment variables added to ~/.bashrc"
else
    echo "✅ Environment variables already in ~/.bashrc"
fi

echo ""
echo "🔄 Please close this terminal and open a NEW terminal"
echo "   Then run: ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py"
```

Make it executable:
```bash
chmod +x setup_gazebo_wsl2.sh
./setup_gazebo_wsl2.sh
```

---

## 🚀 Full Workflow with Fix Applied

### Terminal 1: Start Gazebo (in a NEW terminal!)

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse
```

**Wait for Gazebo to fully load** (may take 30-60 seconds with software rendering).

### Terminal 2: Start Navigation Stack

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration navigation_stack_only.launch.py
```

### In Gazebo: Press Play ▶️

The robot should start exploring!

---

## 🎯 What Solution Should I Use?

| Your Situation | Recommended Solution |
|----------------|---------------------|
| **Windows 11 + WSL2** | Try Solution 1 first, then Solution 2 |
| **Windows 10 + WSL2** | Try Solution 1 first, then Solution 4 |
| **Don't need GUI** | Solution 3 (headless mode) |
| **Solutions 1-3 fail** | Solution 4 (VcXsrv) |

---

## ⚠️ Common Mistakes

1. **Not opening a new terminal** after modifying `.bashrc`
   - Solution: Close terminal completely and open a fresh one

2. **Forgetting to source workspace**
   - Always run: `source install/setup.bash`

3. **Not waiting long enough**
   - Software rendering is slow - give it 60 seconds

4. **Running in same shell where you ran `source ~/.bashrc`**
   - Environment may not update properly - open NEW terminal

---

## 📊 Performance Comparison

| Mode | Startup Time | FPS | Stability |
|------|--------------|-----|-----------|
| **Hardware GPU** | Fast | 60+ | ❌ Crashes in WSL2 |
| **Software Rendering** | Slow | 10-30 | ✅ Stable |
| **Headless** | Medium | N/A | ✅ Very Stable |
| **VcXsrv** | Medium | 30-60 | ⚠️ Depends on config |

---

## 💡 Key Takeaway

**For most WSL2 users:** 

1. ✅ Environment variables are already in your `~/.bashrc`
2. ⚠️ **CLOSE your current terminal**
3. ⚠️ **OPEN A NEW TERMINAL**
4. ✅ Run Gazebo commands
5. 🎉 It should work!

---

## 🆘 Still Not Working?

If none of these solutions work:

### Option A: Use Docker
Run Gazebo in a Docker container with proper GPU passthrough.

### Option B: Native Linux
Consider dual-booting Ubuntu or using a VM with proper GPU passthrough.

### Option C: Use Gazebo on Windows
Install ROS2 natively on Windows (ROS2 Humble works on Windows).

### Option D: Remote Development
Use a cloud-based Linux VM or remote development server.

---

## 📞 Need More Help?

Check these resources:
- [WSL2 GUI Apps Documentation](https://docs.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
- [Gazebo WSL2 Known Issues](https://github.com/gazebosim/gz-sim/issues)
- [ROS2 WSL2 Guide](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)

