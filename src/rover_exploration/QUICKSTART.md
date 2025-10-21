# Quick Start Guide - Rover Autonomous Exploration

Get your rover exploring autonomously in under 5 minutes!

## 🚀 Fast Track

### Step 1: Build the Package (First time only)

```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration
source install/setup.bash
```

### Step 2: Test in Simulation

```bash
# Option A: Use the helper script
./src/rover_exploration/scripts/start_simulation.sh

# Option B: Direct command
ros2 launch rover_exploration simulation_autonomous_slam.launch.py
```

Wait ~10 seconds, then watch the rover explore autonomously in Gazebo!

### Step 3: Run on Real Robot

**IMPORTANT**: Make sure the area is clear and you're ready to stop the robot!

```bash
# Option A: Use the helper script
./src/rover_exploration/scripts/start_exploration.sh

# Option B: Direct command
ros2 launch rover_exploration full_autonomous_slam.launch.py
```

The rover will:
1. ✓ Initialize LIDAR and sensors
2. ✓ Start building a map with SLAM
3. ✓ Begin autonomous exploration
4. ✓ Avoid obstacles automatically
5. ✓ Explore all reachable areas

## 📊 Monitor Progress

### Open RViz to visualize

In a new terminal:
```bash
source ~/rover_workspace/install/setup.bash
ros2 run rviz2 rviz2
```

Add displays:
- Map (`/map`)
- LaserScan (`/scan`)
- RobotModel
- Path (`/plan`)
- Local & Global Costmaps

### Or use the monitoring script

```bash
./src/rover_exploration/scripts/monitor_exploration.sh
```

## 💾 Save Your Map

When exploration is complete (or whenever you want):

```bash
# Option A: Use the helper script
./src/rover_exploration/scripts/save_map.sh --name my_awesome_map

# Option B: Direct command
ros2 run nav2_map_server map_saver_cli -f ~/my_awesome_map
```

Your map will be saved as:
- `my_awesome_map.yaml` (metadata)
- `my_awesome_map.pgm` (image)

## 🛑 Stop the Robot

Press `Ctrl+C` in the launch terminal, or:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## ⚙️ Quick Tweaks

### Make it explore further
Edit `src/rover_exploration/config/exploration_params.yaml`:
```yaml
exploration_radius: 8.0  # Increase from 6.0
```

### Make it safer (more cautious)
Edit `src/rover_exploration/config/nav2_params_exploration.yaml`:
```yaml
vx_max: 0.3  # Slower speed
inflation_radius: 0.6  # Larger safety margin
```

### Make it faster (less safe)
```yaml
vx_max: 0.5  # Faster speed
inflation_radius: 0.35  # Smaller safety margin
```

After changes, rebuild:
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration
source install/setup.bash
```

## 🐛 Quick Fixes

### "Package not found"
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration
source install/setup.bash
```

### "LIDAR not working"
Check accessories configuration:
```bash
cat src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml
```
Make sure `rplidar: active: true`

### "Robot not moving"
1. Check LIDAR: `ros2 topic hz /scan`
2. Check Nav2: `ros2 node list | grep controller`
3. Check map: `ros2 topic hz /map`

### "Robot hitting obstacles"
Increase safety margins in `config/nav2_params_exploration.yaml`:
```yaml
inflation_radius: 0.7
repulsion_weight: 3.0
```

## 📚 Learn More

See the full [README.md](README.md) for:
- Detailed architecture
- All configuration options
- Advanced usage
- Troubleshooting guide

## 🎯 Common Use Cases

### Explore a warehouse
```bash
# In simulation with warehouse world
./src/rover_exploration/scripts/start_simulation.sh --world warehouse
```

### Map your office
```bash
# On real robot
./src/rover_exploration/scripts/start_exploration.sh
# Let it run until complete, then save
./src/rover_exploration/scripts/save_map.sh --name office_map
```

### Test different parameters
```bash
# Copy and modify params
cp src/rover_exploration/config/exploration_params.yaml my_params.yaml
# Edit my_params.yaml with your changes
# Launch with custom params
ros2 launch rover_exploration full_autonomous_slam.launch.py exploration_params_file:=$(pwd)/my_params.yaml
```

## 🏁 You're Ready!

Your rover can now autonomously explore and map any environment. Have fun! 🤖

For questions or issues, see [README.md](README.md#troubleshooting) troubleshooting section.



