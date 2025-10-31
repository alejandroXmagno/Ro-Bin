# Separate Gazebo and Navigation Launch Guide

This guide explains how to launch Gazebo simulation and navigation nodes **separately** for more control.

## Why Launch Separately?

- **More control**: Start Gazebo, inspect the environment, then start navigation
- **Debugging**: Easier to restart just the navigation stack without restarting Gazebo
- **Flexibility**: Can pause Gazebo, modify things, then start navigation when ready

---

## Method 1: Separate Launch (Recommended for Testing)

### Step 1: Start Gazebo Simulation

Open a terminal and launch Gazebo with the rover:

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse
```

**Available worlds:**
- `warehouse` - Large warehouse environment (default)
- `maze` - Maze environment
- `empty` - Empty world for testing

**Wait for Gazebo to fully load.** You should see:
- Gazebo window open
- Rover spawned in the world
- Robot sitting still (paused or ready)

### Step 2: Launch Navigation Stack

Open a **new terminal** (keep Gazebo running) and launch the navigation nodes:

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration navigation_stack_only.launch.py
```

This launches:
- ✅ **SLAM Toolbox** - For mapping
- ✅ **Nav2 Stack** - For path planning and control
- ✅ **Autonomous Explorer** - For frontier exploration (starts after 10 second delay)

You'll see lots of log output. Wait for:
```
[autonomous_explorer]: Autonomous Explorer initialized
[autonomous_explorer]: Waiting for map and pose data...
```

### Step 3: Press Play in Gazebo

1. Go to the Gazebo window
2. Press the **Play** button (▶️) at the bottom left
3. Watch the robot start exploring autonomously!

### Step 4: Monitor with RViz (Optional)

Open another terminal to visualize:

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 run rviz2 rviz2
```

Add displays:
- **Map** → Topic: `/map`
- **LaserScan** → Topic: `/scan`
- **RobotModel** → Description Topic: `/robot_description`
- **TF** → For coordinate frames

---

## Method 2: All-in-One Launch (Original)

If you prefer to launch everything at once:

```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py world:=warehouse
```

This launches Gazebo AND navigation together. Press play in Gazebo to start.

---

## Comparison

| Feature | Separate Launch | All-in-One Launch |
|---------|----------------|-------------------|
| **Control** | High - start navigation when ready | Medium - everything starts together |
| **Debugging** | Easy - restart just navigation | Harder - must restart everything |
| **Setup Time** | Slower (2 commands) | Faster (1 command) |
| **Use Case** | Testing, debugging, demos | Quick experiments |

---

## Stopping and Restarting

### To Stop Navigation Only:
1. In the navigation terminal, press `Ctrl+C`
2. Gazebo keeps running
3. Can restart navigation with step 2 above

### To Stop Everything:
1. Press `Ctrl+C` in navigation terminal
2. Press `Ctrl+C` in Gazebo terminal
3. Wait for clean shutdown

### To Restart Navigation:
```bash
# Don't close Gazebo!
# In the navigation terminal:
ros2 launch rover_exploration navigation_stack_only.launch.py
```

---

## Customization

### Change Exploration Parameters

Edit `src/rover_exploration/config/exploration_params.yaml`:

```yaml
autonomous_explorer:
  ros__parameters:
    exploration_radius: 5.0          # How far to explore (meters)
    goal_timeout: 30.0               # Time before giving up on goal
    obstacle_distance_threshold: 0.5  # Min distance to obstacles
```

Then restart navigation (keep Gazebo running).

### Change World

Restart Gazebo with a different world:

```bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=maze
```

---

## Troubleshooting

### "Navigation stack not ready"

**Solution**: Wait longer. SLAM and Nav2 take 5-10 seconds to initialize.

### Robot doesn't move after pressing play

**Possible causes:**
1. Gazebo is paused - press Play ▶️
2. Navigation nodes crashed - check terminal output
3. No map data yet - wait for SLAM to initialize (10-15 seconds)

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

### "Could not find random free space goal"

**Cause**: Map is too small or no free space detected yet.

**Solution**: Wait 20-30 seconds for SLAM to build a larger map.

---

## Advanced: Manual Navigation Goals

Want to send goals manually instead of autonomous exploration?

### Option 1: Disable Autonomous Explorer

Comment out the autonomous explorer in `navigation_stack_only.launch.py`:

```python
# ld.add_action(delayed_exploration)  # Comment this line
```

### Option 2: Use RViz to Send Goals

1. Open RViz
2. Add **2D Goal Pose** tool
3. Click on map to send navigation goals
4. Robot will navigate to your clicked point!

---

## Summary

**For most users:**
```bash
# Terminal 1: Start Gazebo
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=warehouse

# Terminal 2: Start Navigation
ros2 launch rover_exploration navigation_stack_only.launch.py

# Press Play in Gazebo → Watch it explore! 🚀
```

**For quick tests:**
```bash
# One command
ros2 launch rover_exploration simulation_autonomous_slam.launch.py
```

