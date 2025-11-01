# Hardware Navigation Quick Reference

## 🚀 Quick Start

```bash
# Terminal 1: Robot Driver
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Navigation (auto-starts filtered LiDAR)
./run_hardware_navigation.sh

# OR with autonomous exploration:
./run_hardware_navigation.sh --explore
```

## 📋 Common Commands

```bash
# Start with specific angles file
./run_hardware_navigation.sh --angles my_angles.txt

# Start with exploration
./run_hardware_navigation.sh --explore

# Both
./run_hardware_navigation.sh --angles angles.txt --explore

# Emergency stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 🎮 Manual Control Mode

```bash
# Terminal 3: Launch RViz for manual goals
ros2 launch nav2_bringup rviz_launch.py

# In RViz:
# 1. "2D Pose Estimate" - Set robot's starting position
# 2. "2D Nav Goal" - Click where you want robot to go
```

## 📊 System Components

| Component | Status Check | Purpose |
|-----------|--------------|---------|
| Robot Driver | `ros2 node list \| grep roverrobotics_driver` | Motor control |
| Filtered LiDAR | `ros2 topic hz /scan_filtered` | Obstacle detection |
| SLAM | `ros2 node list \| grep slam_toolbox` | Mapping |
| Nav2 | `ros2 node list \| grep controller_server` | Navigation |
| Explorer | `ros2 node list \| grep autonomous_explorer` | Exploration |

## 🔌 Key Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/scan_filtered` | → Nav2 | Filtered LiDAR input |
| `/cmd_vel` | Nav2 → Robot | Motor commands |
| `/odometry/wheels` | Robot → Nav2 | Position feedback |
| `/map` | SLAM → Nav2 | Current map |
| `/goal_pose` | You → Nav2 | Navigation goals |

## ⚙️ Configuration Files

- **Nav2 params**: `src/rover_exploration/config/nav2_params_hardware_filtered.yaml`
- **Exploration params**: `src/rover_exploration/config/exploration_params.yaml`
- **SLAM params**: `src/roverrobotics_driver/config/slam_configs/mapper_params_online_async.yaml`

## 🎯 Speed Adjustment

Edit `nav2_params_hardware_filtered.yaml`:

```yaml
# Faster (less safe)
max_vel_x: 0.25          # default: 0.15
max_vel_theta: 1.0       # default: 0.8

# Slower (more safe)
max_vel_x: 0.10
max_vel_theta: 0.5
```

## 🛡️ Safety Adjustment

```yaml
# More cautious
inflation_radius: 0.7         # default: 0.5
cost_scaling_factor: 6.0      # default: 5.0

# More aggressive
inflation_radius: 0.3
cost_scaling_factor: 4.0
```

## 📈 Monitoring

```bash
# Watch motor commands
ros2 topic echo /cmd_vel

# Watch explorer status
ros2 topic echo /autonomous_explorer/status

# Check scan rate
ros2 topic hz /scan_filtered

# Monitor all nodes
ros2 node list

# View TF tree
ros2 run tf2_tools view_frames
```

## 🆘 Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot doesn't move | Check `/cmd_vel`: `ros2 topic echo /cmd_vel` |
| No filtered scan | Check filter running: `ros2 node list \| grep filtered` |
| Gets stuck | Reduce filtered angles or increase tolerance |
| Too slow | Increase `max_vel_x` in nav params |
| Hits obstacles | Increase `inflation_radius` |
| Won't plan path | Check map: `ros2 topic echo /map --once` |

## 💾 Save Map

```bash
# While navigation is running
ros2 run nav2_map_server map_saver_cli -f my_map

# Creates: my_map.yaml and my_map.pgm
```

## 🔄 Typical Workflow

```mermaid
1. Start Robot → 2. Start Navigation → 3. Set Goals/Explore → 4. Monitor → 5. Stop
```

```bash
# 1. Start Robot
ros2 launch roverrobotics_driver mini.launch.py

# 2. Start Navigation
./run_hardware_navigation.sh

# 3. Set goals in RViz OR use --explore

# 4. Monitor
ros2 topic echo /autonomous_explorer/status

# 5. Stop
Ctrl+C
```

## 📁 Project Files

```
/home/stickykeys/rover_workspace/
├── run_hardware_navigation.sh          # Main launcher
├── run_filtered_lidar.sh                # LiDAR filter
├── record_obstacles.sh                  # Record angles
├── extract_angles.sh                    # Extract unique angles
├── wasd_controller.sh                   # Manual control
└── src/rover_exploration/
    ├── launch/
    │   └── hardware_navigation_stack.launch.py
    └── config/
        ├── nav2_params_hardware_filtered.yaml
        └── exploration_params.yaml
```

## 🎓 Learning Path

1. **Test manually** - Use WASD controller first
2. **Record obstacles** - Run `./record_obstacles.sh`
3. **Filter LiDAR** - Test filtered scanner
4. **Manual navigation** - Set goals in RViz
5. **Autonomous** - Enable exploration mode

## 📖 Full Documentation

- **Comprehensive Guide**: `HARDWARE_NAVIGATION_GUIDE.md`
- **LiDAR Filtering**: `FILTERED_LIDAR_GUIDE.md`
- **Obstacle Recording**: `OBSTACLE_RECORDING_GUIDE.md`
- **Hardware Setup**: `LIDAR_HARDWARE_GUIDE.md`
- **Main README**: `README.md`

## ⚡ Pro Tips

- Start with slow speeds (`max_vel_x: 0.1`)
- Test in open space first
- Monitor `/cmd_vel` to see what Nav2 is commanding
- Save successful maps for reuse
- Adjust filtered angles as environment changes
- Keep emergency stop command ready

