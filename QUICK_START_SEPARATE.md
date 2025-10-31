# Quick Start: Separate Gazebo + Navigation Launch

This guide shows you how to launch Gazebo and navigation nodes separately.

---

## 🚀 Quick Commands

### Step 1: Open Terminal 1 - Start Gazebo

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse
```

**Wait** for Gazebo window to open and robot to spawn (~30-60 seconds on WSL2).

---

### Step 2: Open Terminal 2 - Start Navigation

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration navigation_stack_only.launch.py
```

**Wait** for navigation nodes to initialize (~10 seconds). Look for:
```
[autonomous_explorer]: Autonomous Explorer initialized
```

---

### Step 3: Press Play in Gazebo

Click the **Play button (▶️)** in Gazebo's bottom-left corner.

Watch the robot explore autonomously! 🎉

---

## 🌍 World Options

Choose different environments:

```bash
# Warehouse (default - good for testing)
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse

# Maze
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=maze

# Empty world
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=empty
```

---

## 🔧 Troubleshooting

### "Package not found" Error

**Problem:**
```
package 'rover_exploration' not found
```

**Solution:**
```bash
cd ~/rover_workspace
source install/setup.bash
```

---

### "Launch file not found" Error

**Problem:**
```
file 'navigation_stack_only.launch.py' was not found
```

**Solution:** Rebuild the package
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration
source install/setup.bash
```

---

### Gazebo Crashes (Ogre Exception)

**Problem:**
```
OGRE EXCEPTION(9:UnimplementedException)
```

**Solution:** See `WSL2_GAZEBO_FIX.md` for detailed fixes.

**Quick fix:**
1. Close terminal
2. Open NEW terminal
3. The environment variables should now work

---

### Robot Doesn't Move

**Possible causes:**

1. **Gazebo is paused** → Press Play ▶️
2. **Navigation not ready** → Wait 10-15 seconds after launching
3. **No map yet** → Wait for SLAM to build a map (15-20 seconds)

**Check if nodes are running:**
```bash
ros2 node list
```

Should see:
- `/slam_toolbox`
- `/controller_server`
- `/planner_server`
- `/bt_navigator`
- `/autonomous_explorer`

---

## 📊 Monitoring

### View Map in RViz

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 run rviz2 rviz2
```

Add displays:
- **Map** → `/map`
- **LaserScan** → `/scan`
- **RobotModel** → `/robot_description`
- **TF** → All frames

---

### Watch Topics

```bash
# See laser scan data
ros2 topic echo /scan

# See robot velocity commands
ros2 topic echo /cmd_vel

# See odometry
ros2 topic echo /odometry/filtered
```

---

### Check Node Status

```bash
# List all nodes
ros2 node list

# Get info about a specific node
ros2 node info /autonomous_explorer

# Check topics
ros2 topic list
```

---

## 🛑 Stopping Everything

### Stop Navigation Only

In Terminal 2 (navigation), press `Ctrl+C`

Gazebo keeps running, you can restart navigation later.

### Stop Everything

1. Terminal 2: `Ctrl+C` (navigation)
2. Terminal 1: `Ctrl+C` (Gazebo)

---

## ⚙️ Customization

### Change Exploration Parameters

Edit: `src/rover_exploration/config/exploration_params.yaml`

```yaml
autonomous_explorer:
  ros__parameters:
    exploration_radius: 5.0          # Search radius (meters)
    goal_timeout: 30.0               # Goal timeout (seconds)
    obstacle_distance_threshold: 0.5  # Min obstacle distance
```

After editing:
```bash
colcon build --packages-select rover_exploration
source install/setup.bash
# Restart navigation in Terminal 2
```

---

## 📝 Complete Workflow Example

```bash
# ============================================
# TERMINAL 1: Gazebo
# ============================================
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse

# Wait for Gazebo to fully load...

# ============================================
# TERMINAL 2: Navigation (open new terminal)
# ============================================
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration navigation_stack_only.launch.py

# Wait for "Autonomous Explorer initialized"...

# ============================================
# TERMINAL 3: RViz (optional, open new terminal)
# ============================================
cd ~/rover_workspace
source install/setup.bash
ros2 run rviz2 rviz2
# Configure displays for /map, /scan, etc.

# ============================================
# Back to Gazebo window: Press Play ▶️
# ============================================
```

---

## 🆚 Separate vs All-in-One

### Separate Launch (This Guide)
✅ More control  
✅ Easier debugging  
✅ Can restart navigation without restarting Gazebo  
❌ Requires 2 terminals  

```bash
# Terminal 1
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py

# Terminal 2
ros2 launch rover_exploration navigation_stack_only.launch.py
```

### All-in-One Launch
✅ Simpler (1 command)  
✅ Faster to start  
❌ Less control  
❌ Must restart everything to change settings  

```bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py
```

---

## 💡 Tips

1. **Always source workspace** in each new terminal
2. **Wait for initialization** - give nodes time to start
3. **Use separate terminals** - easier to see logs
4. **Monitor with RViz** - helps visualize what's happening
5. **Start with warehouse world** - good for testing

---

## 📚 More Documentation

- **Full setup guide:** `SEPARATE_LAUNCH_GUIDE.md`
- **WSL2 fixes:** `WSL2_GAZEBO_FIX.md`
- **SLAM details:** `SLAM_EXPLORATION_SUMMARY.md`
- **Installation:** `INSTALLATION_VERIFICATION.md`

---

## 🎯 Most Common Issue

**Problem:** Commands work in one terminal but not another

**Solution:** You must run `source install/setup.bash` in **EVERY new terminal**

Consider adding to `~/.bashrc`:
```bash
echo 'source ~/rover_workspace/install/setup.bash' >> ~/.bashrc
```

Then every new terminal will automatically source your workspace!

