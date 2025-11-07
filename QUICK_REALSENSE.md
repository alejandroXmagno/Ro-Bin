# Quick Reference: RealSense Camera Viewer

## 🎥 View Camera in 3 Steps

```bash
# 1. Check if working
./check_realsense.sh

# 2. View camera
./view_realsense.sh

# 3. Use controls:
#    d = depth on/off
#    i = infrared on/off
#    c = change colors
#    q = quit
```

---

## 📋 Common Commands

| Task | Command |
|------|---------|
| **Check status** | `./check_realsense.sh` |
| **View camera** | `./view_realsense.sh` |
| **Start RealSense** | `./start_realsense.sh` |
| **Check topics** | `ros2 topic list \| grep camera` |
| **Check frame rate** | `ros2 topic hz /camera/camera/color/image_raw` |

---

## 🎮 Keyboard Controls

| Key | Action |
|-----|--------|
| `d` | Toggle depth view |
| `i` | Toggle infrared view |
| `c` | Cycle colormap (Jet/Hot/Cool/Rainbow/Viridis) |
| `q` | Quit |
| `ESC` | Quit |

---

## 🖼️ What You See

- **RGB** (left): Normal color camera view
- **Depth** (middle): Distance to objects
  - 🔴 Red = Close
  - 🟡 Yellow = Medium
  - 🔵 Blue = Far
  - Green crosshair shows distance at center
- **Infrared** (right): Low-light vision (toggle with `i`)

---

## ⚠️ Troubleshooting

| Problem | Solution |
|---------|----------|
| "No topics found" | 1. Check USB: `lsusb \| grep Intel`<br>2. Start RealSense: `./start_realsense.sh` |
| "cv_bridge not found" | `sudo apt install ros-humble-cv-bridge python3-opencv` |
| Low FPS / Laggy | Press `i` to turn off infrared streams |
| Black screen | Check if topics are publishing: `./check_realsense.sh` |

---

## 🚀 Full Setup

### With Robot Driver (if RealSense included)

```bash
# Terminal 1: Start robot
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: View camera
./view_realsense.sh
```

### Standalone RealSense

```bash
# Terminal 1: Start RealSense
./start_realsense.sh

# Terminal 2: View camera
./view_realsense.sh
```

---

## 📏 Reading Depth

The depth view shows a **green crosshair** in the center with distance reading:

```
Center: 1.23m
```

This means the object in the center is **1.23 meters away**.

Point the camera at objects to measure their distance!

---

## 💡 Use Cases

1. **Check what robot sees** - Use RGB stream
2. **Measure obstacle distance** - Use depth + crosshair
3. **Night operation** - Use infrared streams (press `i`)
4. **Navigation testing** - Watch depth while driving
5. **Object detection** - RGB for color/shape recognition

---

## 📚 More Info

See `REALSENSE_VIEWER_GUIDE.md` for complete documentation.

---

**Quick tip:** Run with robot navigation to see what the robot sees while it navigates! 🤖📷

