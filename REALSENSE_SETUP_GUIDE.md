# Intel RealSense D435i Integration Guide

Complete guide for setting up and using the Intel RealSense D435i depth camera with your autonomous rover.

---

## 📋 Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Software Installation](#software-installation)
3. [Hardware Connection](#hardware-connection)
4. [Configuration](#configuration)
5. [Usage](#usage)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 Hardware Requirements

- **Intel RealSense D435i** depth camera
- **USB 3.0** port (USB 3.1/3.2 recommended for best performance)
- **USB 3.0 cable** (preferably < 2 meters for reliability)
- Mounting bracket for your rover (camera should face forward)

### Recommended Mounting Position

- **Height**: 10-30 cm above ground
- **Orientation**: Forward-facing, level with ground
- **Clear view**: No obstructions in front of camera
- **Stable mount**: Minimal vibration

---

## 💻 Software Installation

### Step 1: Install RealSense SDK

```bash
# Add Intel's repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
  sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### Step 2: Install ROS2 RealSense Package

```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

### Step 3: Install Depth-to-LaserScan Package

```bash
sudo apt install -y ros-humble-depthimage-to-laserscan
```

### Step 4: Verify Installation

```bash
# Connect your RealSense camera, then run:
realsense-viewer
```

You should see the RealSense Viewer GUI with your camera's streams.

---

## 🔌 Hardware Connection

### Physical Setup

1. **Connect** RealSense D435i to a **USB 3.0** port (blue port)
2. **Verify** connection:
   ```bash
   lsusb | grep Intel
   ```
   You should see: `Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i`

3. **Check device permissions**:
   ```bash
   ls -l /dev/video*
   ```
   If you see permission errors, add yourself to the `video` group:
   ```bash
   sudo usermod -a -G video $USER
   # Log out and log back in for this to take effect
   ```

### Test the Camera

```bash
# Start the RealSense node
ros2 run realsense2_camera realsense2_camera_node

# In another terminal, check topics
ros2 topic list | grep camera
```

You should see topics like:
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/depth/color/points`

---

## ⚙️ Configuration

### Adding RealSense to Robot URDF

The RealSense URDF has been created at:
`src/roverrobotics_ros2/roverrobotics_description/urdf/accessories/realsense_d435i.urdf`

**To enable it on your rover:**

1. **Edit your rover's URDF** (e.g., `rover_2wd.urdf`, `mini.urdf`, etc.):
   ```bash
   nano src/roverrobotics_ros2/roverrobotics_description/urdf/rover_2wd.urdf
   ```

2. **Add this line** after the other sensor includes (around line 20):
   ```xml
   <xacro:include filename="$(find roverrobotics_description)/urdf/accessories/realsense_d435i.urdf" />
   ```

3. **Rebuild** the workspace:
   ```bash
   cd ~/rover_workspace
   colcon build --packages-select roverrobotics_description
   source install/setup.bash
   ```

### Configuration Files

Two configuration files have been created:

1. **`src/rover_exploration/config/realsense_navigation.yaml`**
   - Optimized for autonomous navigation
   - Enables depth processing filters
   - Configured for obstacle avoidance

2. **Existing**: `src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml`
   - Set `active: true` under `realsense:` section to enable

---

## 🚀 Usage

### Option 1: Standalone RealSense Test

```bash
# Terminal 1: Start rover driver
ros2 launch roverrobotics_driver zero2.launch.py

# Terminal 2: Start RealSense
ros2 launch roverrobotics_driver accessories.launch.py

# Terminal 3: View depth data
ros2 run rqt_image_view rqt_image_view
```

In rqt_image_view, select `/camera/depth/image_rect_raw` or `/camera/color/image_raw`.

### Option 2: Full Autonomous Exploration with RealSense

```bash
# Terminal 1: Launch Gazebo simulation (or real robot driver)
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse

# Terminal 2: Launch autonomous exploration with RealSense
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration autonomous_slam_with_realsense.launch.py use_sim_time:=true
```

For **hardware** (real robot):
```bash
# Terminal 1: Launch robot driver
ros2 launch roverrobotics_driver zero2.launch.py

# Terminal 2: Launch autonomous exploration with RealSense
ros2 launch rover_exploration autonomous_slam_with_realsense.launch.py use_sim_time:=false
```

### Option 3: Add to Existing Launches

To use with your existing `navigation_stack_only.launch.py`:

1. **Edit** `src/rover_exploration/launch/navigation_stack_only.launch.py`
2. **Add** RealSense and depth-to-laserscan nodes (see example in `autonomous_slam_with_realsense.launch.py`)

---

## 🎯 Benefits for Autonomous Navigation

### Enhanced Obstacle Avoidance

- **3D depth perception** detects obstacles that 2D lidar might miss
- **Vertical obstacles** (overhangs, low-hanging objects) are detected
- **Dynamic obstacles** are tracked in real-time

### Improved Mapping

- **Denser maps** with combined lidar + depth data
- **Better coverage** in tight spaces
- **Reduced blind spots**

### Stuck Prevention

- **Early obstacle detection** cancels paths before collision
- **Better recovery** with 3D awareness
- **Improved frontier detection** for exploration

---

## 🔍 Monitoring RealSense Performance

### Check Data Flow

```bash
# Check if camera is publishing
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/scan  # Converted laser scan
```

Expected rates:
- Depth/Color: ~15 Hz
- Scan: ~30 Hz

### Visualize in RViz

```bash
ros2 run rviz2 rviz2
```

Add displays:
1. **Image** → `/camera/color/image_raw`
2. **DepthCloud** → `/camera/depth/image_rect_raw` + `/camera/depth/camera_info`
3. **LaserScan** → `/camera/scan`
4. **PointCloud2** → `/camera/depth/color/points`

---

## 🐛 Troubleshooting

### Camera Not Detected

**Problem**: `lsusb` doesn't show RealSense camera

**Solutions**:
1. Check USB connection (must be USB 3.0 - blue port)
2. Try a different USB port
3. Restart udev rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
4. Power cycle the camera (unplug/replug)

### Permission Denied

**Problem**: `Permission denied` when accessing `/dev/video*`

**Solution**:
```bash
sudo usermod -a -G video $USER
# Log out and log back in
```

### Node Crashes on Startup

**Problem**: `realsense2_camera_node` crashes immediately

**Solutions**:
1. Check if another process is using the camera:
   ```bash
   lsof /dev/video*
   ```
   Kill any conflicting processes

2. Reset the camera:
   ```bash
   rs-enumerate-devices -r
   ```

3. Check firmware version:
   ```bash
   rs-fw-update -l
   ```
   Update if needed:
   ```bash
   rs-fw-update -f
   ```

### Poor Depth Quality

**Problem**: Noisy or missing depth data

**Solutions**:
1. **Enable filters** in `realsense_navigation.yaml`:
   - Set `spatial_filter.enable: true`
   - Set `temporal_filter.enable: true`
   - Set `hole_filling_filter.enable: true`

2. **Adjust laser power**:
   - Increase `depth_module.laser_power` (max 150.0)

3. **Check lighting**:
   - RealSense works best in moderate lighting
   - Avoid direct sunlight or complete darkness

4. **Check emitter**:
   - Ensure `emitter_enabled: 1` in config
   - You should see a faint red glow from the camera

### Low Frame Rate

**Problem**: Camera running at < 10 fps

**Solutions**:
1. **Check USB connection**:
   ```bash
   usb-devices | grep -A 20 "Intel.*RealSense"
   ```
   Look for `Speed: 5000M` (USB 3.0) or `10000M` (USB 3.1/3.2)
   If you see `480M`, you're on USB 2.0 (too slow!)

2. **Reduce resolution** in `realsense_navigation.yaml`:
   ```yaml
   depth_module:
     profile: 424x240x15  # Lower resolution
   ```

3. **Disable unused streams**:
   ```yaml
   enable_infra1: false
   enable_infra2: false
   ```

### Conflicts with Other Cameras

**Problem**: Multiple cameras detected, wrong one being used

**Solution**: Specify camera serial number in `realsense_navigation.yaml`:
```yaml
realsense:
  ros__parameters:
    serial_no: '123456789012'  # Your camera's serial number
```

Find your serial number:
```bash
rs-enumerate-devices | grep "Serial Number"
```

### WSL2-Specific Issues

**Problem**: Camera not working in WSL2

**Solution**: WSL2 has limited USB support. Use **usbipd-win**:

1. **On Windows** (PowerShell as Administrator):
   ```powershell
   # Install usbipd-win
   winget install --interactive --exact dorssel.usbipd-win
   
   # List USB devices
   usbipd list
   
   # Attach RealSense (replace <BUSID> with your device's bus ID)
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```

2. **In WSL2**:
   ```bash
   lsusb | grep Intel
   ```

---

## 📊 Performance Tuning

### For Maximum Range

```yaml
depth_module:
  laser_power: 150.0  # Maximum power
  exposure: 10000     # Longer exposure
  profile: 640x480x15 # Higher resolution
```

### For Maximum Speed

```yaml
depth_module:
  profile: 424x240x30  # Lower resolution, higher FPS
  laser_power: 100.0   # Lower power
decimation_filter:
  enable: true         # Reduce point cloud density
```

### For Best Quality

```yaml
depth_module:
  profile: 640x480x15
  laser_power: 150.0
spatial_filter:
  enable: true
  filter_smooth_alpha: 0.5
temporal_filter:
  enable: true
  filter_smooth_alpha: 0.4
hole_filling_filter:
  enable: true
  holes_fill: 1
```

---

## 🎓 Next Steps

1. **Tune Nav2 costmap** to use camera scan data
2. **Adjust exploration parameters** for depth-aware navigation
3. **Test in different environments** (indoor, outdoor, varied lighting)
4. **Monitor performance** and adjust filters as needed

---

## 📚 Additional Resources

- [RealSense ROS2 Wrapper Documentation](https://github.com/IntelRealSense/realsense-ros)
- [Intel RealSense SDK Documentation](https://dev.intelrealsense.com/docs)
- [Depth-to-LaserScan Package](https://github.com/ros-perception/depthimage_to_laserscan)
- [Nav2 with Depth Cameras](https://navigation.ros.org/)

---

## ✅ Verification Checklist

- [ ] RealSense SDK installed (`realsense-viewer` works)
- [ ] ROS2 packages installed (`ros-humble-realsense2-camera`)
- [ ] Camera detected (`lsusb | grep Intel`)
- [ ] Camera streams publishing (`ros2 topic list | grep camera`)
- [ ] URDF includes RealSense model
- [ ] Configuration file updated
- [ ] Autonomous exploration launch file includes RealSense
- [ ] Depth-to-laserscan conversion working
- [ ] RViz visualization shows depth data
- [ ] Robot successfully navigates using depth data

---

**Need Help?** Check the Troubleshooting section or consult the RealSense ROS2 documentation.

**Status**: ✅ RealSense integration complete and ready for testing!

