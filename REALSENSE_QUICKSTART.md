# Intel RealSense D435i Quick Start

Fast setup guide for integrating RealSense depth camera with your autonomous rover.

---

## ⚡ Quick Install (5 minutes)

```bash
# 1. Install RealSense SDK and ROS2 packages
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
  sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-depthimage-to-laserscan

# 2. Add user to video group (for camera permissions)
sudo usermod -a -G video $USER
# Log out and log back in after this!

# 3. Enable RealSense in robot URDF
# Edit your rover's URDF file (e.g., rover_2wd.urdf)
nano src/roverrobotics_ros2/roverrobotics_description/urdf/rover_2wd.urdf

# Add this line after other sensor includes (~line 20):
# <xacro:include filename="$(find roverrobotics_description)/urdf/accessories/realsense_d435i.urdf" />

# 4. Rebuild workspace
cd ~/rover_workspace
colcon build --symlink-install --allow-overriding roverrobotics_description
source install/setup.bash
```

---

## 🔌 Test Camera Connection

```bash
# Plug in RealSense camera, then:
lsusb | grep Intel
# Should show: Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i

# Test with viewer
realsense-viewer
```

---

## 🚀 Launch Options

### Option 1: Test RealSense Alone

```bash
# Terminal 1: Start robot
ros2 launch roverrobotics_driver zero2.launch.py

# Terminal 2: Start RealSense (after enabling in accessories.yaml)
ros2 launch roverrobotics_driver accessories.launch.py

# Terminal 3: View camera feed
ros2 run rqt_image_view rqt_image_view
```

### Option 2: Full Autonomous Exploration (Simulation)

```bash
# Terminal 1: Gazebo
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse

# Terminal 2: Navigation + RealSense
ros2 launch rover_exploration autonomous_slam_with_realsense.launch.py use_sim_time:=true
```

### Option 3: Full Autonomous Exploration (Hardware)

```bash
# Terminal 1: Robot driver
ros2 launch roverrobotics_driver zero2.launch.py

# Terminal 2: Navigation + RealSense
ros2 launch rover_exploration autonomous_slam_with_realsense.launch.py use_sim_time:=false
```

---

## 📊 Verify It's Working

```bash
# Check camera topics
ros2 topic list | grep camera

# Expected topics:
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /camera/depth/color/points
# /camera/scan  (converted laser scan)

# Check data rates
ros2 topic hz /camera/depth/image_rect_raw  # Should be ~15 Hz
ros2 topic hz /camera/scan                   # Should be ~30 Hz
```

---

## 🎛️ Configuration Files

| File | Purpose |
|------|---------|
| `src/rover_exploration/config/realsense_navigation.yaml` | Optimized for navigation |
| `src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml` | General RealSense config |
| `src/roverrobotics_ros2/roverrobotics_description/urdf/accessories/realsense_d435i.urdf` | Robot model |

---

## 🐛 Quick Troubleshooting

### Camera Not Found
```bash
# Check USB connection (must be USB 3.0)
lsusb | grep Intel

# Reset udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Permission Denied
```bash
# Make sure you're in video group and have logged out/in
groups | grep video

# If not in group:
sudo usermod -a -G video $USER
# Log out and log back in!
```

### Node Crashes
```bash
# Reset camera
rs-enumerate-devices -r

# Check if another process is using it
lsof /dev/video* | grep realsense
```

### Low Frame Rate
```bash
# Check USB speed (should be 5000M or 10000M, NOT 480M)
usb-devices | grep -A 20 "Intel.*RealSense" | grep Speed
```

---

## 📚 Full Documentation

For detailed setup, troubleshooting, and advanced configuration, see:
- **[REALSENSE_SETUP_GUIDE.md](REALSENSE_SETUP_GUIDE.md)** - Complete guide

---

## ✅ Quick Checklist

- [ ] RealSense SDK installed
- [ ] ROS2 packages installed
- [ ] Camera detected (`lsusb | grep Intel`)
- [ ] Added to video group and logged out/in
- [ ] URDF includes RealSense
- [ ] Workspace rebuilt
- [ ] Camera streams publishing
- [ ] Depth-to-laserscan working

---

**Status**: Ready to explore with 3D depth perception! 🎉

