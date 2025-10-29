# 🚀 Quick Start - LIDAR SLAM in Gazebo

## ✅ What's Already Running:
1. **Gazebo** - Your rover simulation ✓
2. **LIDAR** - Publishing on `/scan` ✓  
3. **Odometry** - Publishing on `/odometry/wheels` ✓
4. **RViz2** - Visualization tool (just launched) ✓

## 🎮 Move the Robot RIGHT NOW:

Open a **NEW terminal** and run:

```bash
cd /home/stickykeys/rover_workspace
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Keyboard Controls:
- **`i`** = Move Forward
- **`,`** = Move Backward  
- **`j`** = Turn Left
- **`l`** = Turn Right
- **`k`** = Stop
- **`q`/`z`** = Increase/Decrease speed

## 🗺️ Start SLAM Mapping:

In **ANOTHER new terminal**:

```bash
cd /home/stickykeys/rover_workspace
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true \
  -r scan:=/scan \
  -r odom:=/odometry/wheels
```

## 📺 Configure RViz2:

In the RViz2 window that opened:

1. **Change Fixed Frame:**
   - In left panel, find "Fixed Frame"
   - Change from "map" to **"odom"**

2. **Add LaserScan Display:**
   - Click "Add" button (bottom left)
   - Choose "By topic" tab
   - Find `/scan` → `LaserScan`
   - Click OK
   - You should see red dots (LIDAR data)!

3. **Add Map Display (once SLAM starts):**
   - Click "Add" again
   - Choose "By topic"
   - Find `/map` → `Map`
   - Click OK

4. **Add RobotModel:**
   - Click "Add"
   - Choose "By display type"
   - Find "RobotModel"
   - Click OK

## 🎯 What You'll See:

- **Gazebo Window**: 3D simulation of rover
- **RViz2 Window**: 
  - Red dots = LIDAR scan  
  - Gray map = Areas explored
  - Robot model in center
- **Terminal**: Keyboard control feedback

## 🚗 Drive and Map:

1. Use the teleop terminal to drive the rover around
2. Watch RViz2 as the LIDAR scans appear
3. Once SLAM starts, watch the map build in real-time!
4. The robot creates a 2D map of the environment

## 🛑 To Stop Everything:

Press `Ctrl+C` in each terminal window

## 🔧 Troubleshooting:

### Can't see LIDAR data in RViz2?
```bash
# Check if scan is publishing:
ros2 topic hz /scan
# Should show ~10-30 Hz
```

### Robot not moving?
```bash
# Check cmd_vel topic:
ros2 topic echo /cmd_vel
# Should show velocity commands when you press keys
```

### No map appearing?
```bash
# Check if SLAM is running:
ros2 node list | grep slam
# Should show: /slam_toolbox

# Check if map is publishing:
ros2 topic list | grep map
# Should show: /map
```

---

**You're all set! Start driving and watch your rover map the environment! 🗺️🤖**



