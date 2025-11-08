# How to View RealSense Camera (Connected to Jetson)

The RealSense is physically connected to your Jetson. Here are **4 ways** to view what it sees:

---

## 🌐 Method 1: Web Browser (EASIEST! 🎉)

**View from ANY device** - laptop, phone, tablet, etc.

### On the Jetson:

```bash
# Terminal 1: Start robot
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Start web stream
./stream_realsense_web.sh
```

### On Your Laptop/Phone/Tablet:

Open a web browser and go to:
```
http://<jetson-ip>:5000/
```

**That's it!** You'll see RGB + Depth camera in your browser!

### Features:
- ✅ Works from **any device** on your network
- ✅ View from your **laptop** while robot drives
- ✅ View from your **phone/tablet** 
- ✅ No SSH or X11 needed
- ✅ Multiple people can view at once
- ✅ Shows RGB and Depth side-by-side

**Example URLs:**
- Combined view: `http://192.168.1.100:5000/`
- RGB only: `http://192.168.1.100:5000/rgb`
- Depth only: `http://192.168.1.100:5000/depth`

---

## 🎯 Method 2: RViz (Best for Navigation)

**Best when you want** camera + robot position + LiDAR all in one view.

### On the Jetson:

```bash
# Start robot
ros2 launch roverrobotics_driver mini.launch.py
```

### On Your Laptop (or Jetson):

```bash
# Make sure ROS_DOMAIN_ID matches Jetson
export ROS_DOMAIN_ID=0  # or whatever Jetson uses

# Launch RViz
./view_realsense_rviz.sh

# OR use the combined navigation viewer:
./view_hardware_nav.sh
```

### Features:
- ✅ Camera feeds in RViz panels
- ✅ See robot position on map
- ✅ See LiDAR data
- ✅ See navigation paths
- ✅ Professional robotics visualization
- ✅ Works over ROS network

---

## 💻 Method 3: OpenCV Viewer (X11 Forwarding)

**If you're SSH'd into the Jetson** and have X11 forwarding enabled.

### From Your Laptop:

```bash
# SSH with X11 forwarding
ssh -X stickykeys@<jetson-ip>

# Navigate to workspace
cd /home/stickykeys/rover_workspace
source install/setup.bash

# View camera (window appears on your laptop!)
./view_realsense.sh
```

### Features:
- ✅ High-quality OpenCV display
- ✅ Keyboard controls (d/i/c/q)
- ✅ Depth with crosshair distance
- ✅ Multiple colormap options
- ✅ ~30 FPS display

### Requirements:
- SSH with `-X` flag
- X11 server on your laptop (built-in on Linux/Mac)
- Windows: Install Xming or VcXsrv

---

## 🔍 Method 4: Command Line Check

**Quick check** if camera is working and see topic info.

```bash
# On Jetson or laptop (if ROS network configured):

# Check status
./check_realsense.sh

# List topics
ros2 topic list | grep camera

# Check frame rate
ros2 topic hz /camera/camera/color/image_raw

# View raw data
ros2 topic echo /camera/camera/color/image_raw
```

---

## 📊 Comparison Table

| Method | Where to Run | View From | Best For | Difficulty |
|--------|-------------|-----------|----------|------------|
| **Web Browser** | Jetson | Any device | Monitoring while driving | ⭐ Easy |
| **RViz** | Laptop | Laptop | Navigation + camera | ⭐⭐ Medium |
| **OpenCV** | Jetson (SSH) | Laptop | Development/testing | ⭐⭐⭐ Medium |
| **CLI** | Either | Terminal | Quick checks | ⭐ Easy |

---

## 🚀 Quick Start (Recommended)

**For most people, use the Web Browser method:**

```bash
# 1. On Jetson - Start robot
ros2 launch roverrobotics_driver mini.launch.py

# 2. On Jetson - Start web stream (in another terminal)
cd /home/stickykeys/rover_workspace
source install/setup.bash
./stream_realsense_web.sh

# 3. On your laptop - Open browser
# Go to: http://<jetson-ip>:5000/

# Done! 🎉
```

---

## 💡 Pro Tips

### Find Your Jetson's IP Address:

```bash
# On Jetson:
hostname -I
# or
ip addr show | grep "inet "
```

### View While Running Navigation:

```bash
# Jetson Terminal 1: Navigation
./run_hardware_navigation.sh --explore

# Jetson Terminal 2: Web stream
./stream_realsense_web.sh

# Laptop: Open http://<jetson-ip>:5000/
# Watch what robot sees while it navigates!
```

### Multiple Viewers:

You can run multiple viewing methods at the same time:
- Web browser for monitoring
- RViz for detailed navigation view
- All from different devices!

---

## ⚠️ Troubleshooting

### Web Browser Method

**"Can't connect to http://jetson-ip:5000/"**
- Check Jetson IP: `hostname -I` on Jetson
- Check firewall: `sudo ufw allow 5000`
- Make sure both devices are on same network
- Try `http://localhost:5000` if on Jetson directly

**"Flask not found"**
```bash
pip3 install flask --user
```

### RViz Method

**"No image topics in RViz"**
- Check ROS_DOMAIN_ID matches between Jetson and laptop
- Check topics: `ros2 topic list | grep camera`
- In RViz, manually set topic to `/camera/camera/color/image_raw`

### X11 Method

**"cannot open display"**
```bash
# Make sure you SSH'd with -X:
ssh -X stickykeys@<jetson-ip>

# Check X11 forwarding is working:
echo $DISPLAY  # Should show something like "localhost:10.0"
```

---

## 📦 Files Created

- `stream_realsense_web.py` - Web streamer
- `stream_realsense_web.sh` - Quick launch
- `view_realsense_rviz.sh` - RViz viewer
- `view_realsense.py` - OpenCV viewer
- `view_realsense.sh` - OpenCV quick launch
- `check_realsense.sh` - Diagnostic tool

---

## 🎬 Example Session

```bash
# === ON JETSON ===
# Terminal 1
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2
./stream_realsense_web.sh
# Shows: "Access from: http://192.168.1.100:5000/"

# === ON YOUR LAPTOP ===
# Open web browser
# Go to: http://192.168.1.100:5000/
# See camera feed! 📷

# === OPTIONAL: Also open RViz ===
# On laptop terminal
export ROS_DOMAIN_ID=0
./view_realsense_rviz.sh
# Now you have browser + RViz both showing camera!
```

---

**TL;DR:** Use the **Web Browser** method (`./stream_realsense_web.sh`) - it's the easiest! 🎉


