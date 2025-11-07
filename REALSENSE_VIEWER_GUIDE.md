# RealSense Camera Viewer Guide

View what the RealSense camera sees on your physical robot in real-time!

---

## Quick Start

### Check if RealSense is Working

```bash
./check_realsense.sh
```

This will show:
- ✅ Available camera topics
- ✅ Publishing status and frame rates
- ✅ Running RealSense nodes

---

### View Camera Streams

**If RealSense is already running** (e.g., with robot driver):

```bash
# Just view it
./view_realsense.sh
```

**If RealSense needs to be started separately:**

```bash
# Terminal 1: Start RealSense
./start_realsense.sh

# Terminal 2: View camera
./view_realsense.sh
```

---

## What You'll See

The viewer shows multiple streams side-by-side:

| Stream | What It Shows | Use Case |
|--------|---------------|----------|
| **RGB** | Color camera view | Normal vision, object recognition |
| **Depth** | Distance to objects | Obstacle detection, 3D mapping |
| **Infrared 1** | IR camera 1 | Low-light vision, stereo depth |
| **Infrared 2** | IR camera 2 | Low-light vision, stereo depth |

### Depth View Features

- **Color-coded depth**: Closer objects = warmer colors (red), farther = cooler (blue)
- **Green crosshair**: Points at center of image
- **Distance reading**: Shows depth at center in meters
- **Real-time**: Updates at ~30 FPS

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| `d` | Toggle depth view ON/OFF |
| `i` | Toggle infrared view ON/OFF |
| `c` | Cycle depth colormap (Jet → Hot → Cool → Rainbow → Viridis) |
| `q` or `ESC` | Quit viewer |

---

## Common Issues

### "No RealSense camera topics found!"

**Solution 1: Check USB connection**
```bash
# Check if USB device is detected
lsusb | grep -i intel
# Should show: "Intel Corp. RealSense ..."
```

**Solution 2: Start RealSense manually**
```bash
./start_realsense.sh
```

**Solution 3: Install RealSense package**
```bash
sudo apt install ros-humble-realsense2-camera
```

---

### "cv_bridge not found!"

```bash
# Install required packages
sudo apt install ros-humble-cv-bridge python3-opencv
```

---

### Topics have different names

If your RealSense uses different topic names, edit `view_realsense.py`:

```python
# Common alternative topic names:
# RGB: /camera/color/image_raw
# Depth: /camera/depth/image_rect_raw
# Infrared: /camera/infra1/image_rect_raw

# Find your actual topics:
ros2 topic list | grep camera
```

Then update the topic names in the script (lines 26-47).

---

### Low FPS or laggy

**Option 1: Reduce resolution** (edit `start_realsense.sh`):
```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.profile:=640x480x30  # Lower resolution
```

**Option 2: Disable unnecessary streams**:
```bash
# Press 'i' to turn off infrared view
# Only show RGB + Depth
```

---

## Integration with Robot

### Option 1: RealSense with Robot Driver

If your `mini.launch.py` already includes RealSense:

```bash
# Terminal 1: Start robot (includes RealSense)
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: View camera
./view_realsense.sh
```

### Option 2: Separate Launch

If RealSense is not in robot driver:

```bash
# Terminal 1: Robot driver
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: RealSense camera
./start_realsense.sh

# Terminal 3: View camera
./view_realsense.sh
```

---

## Use Cases

### 1. Object Detection / Inspection

```bash
# View RGB stream to see what robot sees
./view_realsense.sh

# Press 'd' to turn off depth
# Focus on RGB stream for object identification
```

### 2. Obstacle Distance Measurement

```bash
./view_realsense.sh

# Point robot at obstacle
# Look at depth view - green crosshair shows distance
# Example: "Center: 1.23m" = obstacle is 1.23 meters away
```

### 3. Navigation Testing

```bash
# Terminal 1: Run navigation
./run_hardware_navigation.sh --explore

# Terminal 2: Watch what camera sees
./view_realsense.sh

# Toggle depth ('d') to see how robot perceives obstacles
```

### 4. Low-Light Operation

```bash
./view_realsense.sh

# Press 'i' to show infrared streams
# Infrared works in dark/low-light conditions
# Useful for night operation or dark environments
```

---

## Advanced: Recording Camera Data

To record RealSense data for later analysis:

```bash
# Record all camera topics
ros2 bag record -o realsense_recording \
    /camera/camera/color/image_raw \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/camera_info

# Stop recording with Ctrl+C

# Playback later
ros2 bag play realsense_recording/

# View during playback
./view_realsense.sh
```

---

## Troubleshooting Commands

| Problem | Command |
|---------|---------|
| Check RealSense status | `./check_realsense.sh` |
| List camera topics | `ros2 topic list \| grep camera` |
| Check topic rate | `ros2 topic hz /camera/camera/color/image_raw` |
| View topic info | `ros2 topic info /camera/camera/color/image_raw` |
| Check RealSense node | `ros2 node list \| grep realsense` |
| Test USB connection | `lsusb \| grep -i intel` |

---

## Quick Reference

```bash
# Check if working
./check_realsense.sh

# Start RealSense
./start_realsense.sh

# View camera
./view_realsense.sh

# Controls while viewing:
#   d - Toggle depth
#   i - Toggle infrared
#   c - Change colormap
#   q - Quit
```

---

## Technical Details

- **Viewer**: Python + OpenCV + cv_bridge
- **Update Rate**: ~30 FPS display
- **Depth Encoding**: 16-bit unsigned int (millimeters)
- **RGB Encoding**: 8-bit BGR
- **Window**: Resizable, starts at 1280x720

---

## Files Created

- `view_realsense.py` - Main viewer script
- `view_realsense.sh` - Quick launch script
- `check_realsense.sh` - Diagnostic tool
- `start_realsense.sh` - Launch RealSense camera
- `REALSENSE_VIEWER_GUIDE.md` - This guide

---

Enjoy viewing your robot's perspective! 📷🤖

