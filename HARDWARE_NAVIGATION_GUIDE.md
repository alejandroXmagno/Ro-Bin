# Hardware Navigation Guide

Complete guide for running autonomous navigation on the physical Rover Mini robot with filtered LiDAR.

## Overview

This system integrates:
1. **Physical robot driver** - Controls motors via `/cmd_vel`
2. **Filtered LiDAR** - Uses `/scan_filtered` to ignore specific angles
3. **SLAM** - Real-time mapping with SLAM Toolbox
4. **Nav2** - Path planning and obstacle avoidance
5. **Autonomous Explorer** - Optional autonomous exploration

## Prerequisites

### Hardware Requirements
- Rover Mini robot with motors
- RPLidar S2 (enabled in accessories.yaml)
- Sufficient battery charge
- Clear operating space (3m x 3m minimum)

### Software Requirements
- Robot driver running (`mini.launch.py`)
- LiDAR enabled and publishing to `/scan`
- Filtered angles recorded (from obstacle recording)

## Quick Start

### Option 1: All-in-One Script (Recommended)

```bash
# Terminal 1: Start robot driver
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Start everything automatically
./run_hardware_navigation.sh
```

The script will:
- Check prerequisites
- Auto-find and load angles file
- Start filtered LiDAR scanner
- Launch navigation stack
- Display status

### Option 2: Manual Control

For more control over each component:

```bash
# Terminal 1: Robot driver
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Filtered LiDAR
./run_filtered_lidar.sh close_obstacles_1762028491_unique_angles.txt

# Terminal 3: Navigation stack
source install/setup.bash
ros2 launch rover_exploration hardware_navigation_stack.launch.py
```

## Usage Modes

### Mode 1: Manual Goal Setting

Use RViz to set navigation goals:

```bash
# Terminal 1: Robot + navigation (as above)

# Terminal 2: Launch RViz
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py

# In RViz:
# 1. Use "2D Pose Estimate" to set initial position
# 2. Use "2D Nav Goal" to set destination
# 3. Watch robot navigate autonomously!
```

### Mode 2: Autonomous Exploration

Let the robot explore and map automatically:

```bash
./run_hardware_navigation.sh --explore
```

The robot will:
- Build a map as it explores
- Find frontiers (unexplored areas)
- Navigate to them systematically
- Avoid obstacles using filtered LiDAR
- Continue until the area is mapped

### Mode 3: Custom Angles

Use specific obstacle angles:

```bash
./run_hardware_navigation.sh --angles my_custom_angles.txt --explore
```

## Configuration

### Nav2 Parameters

The hardware navigation uses: `config/nav2_params_hardware_filtered.yaml`

Key settings for physical robot:
```yaml
use_sim_time: False              # Hardware timing
max_vel_x: 0.15                  # Conservative speed (m/s)
max_vel_theta: 0.8               # Turn rate (rad/s)
inflation_radius: 0.5            # Obstacle buffer (m)
scan topic: /scan_filtered       # Filtered LiDAR
odom_topic: /odometry/wheels     # Wheel odometry
```

### Speed Adjustment

To make the robot faster/slower, edit `nav2_params_hardware_filtered.yaml`:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.25        # Increase for faster (default: 0.15)
      max_vel_theta: 1.0     # Increase for faster turns (default: 0.8)
```

**Warning**: Higher speeds reduce reaction time for obstacles!

### Safety Parameters

Adjust safety margins:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.6     # Larger = more cautious (default: 0.5)
        cost_scaling_factor: 6.0  # Higher = stronger repulsion (default: 5.0)
```

## ROS2 Topics

### Inputs (Hardware → Navigation)
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | Original LiDAR data |
| `/scan_filtered` | LaserScan | Filtered LiDAR (used by Nav2) |
| `/odometry/wheels` | Odometry | Wheel odometry from robot |
| `/tf` | TF2 | Transform tree |

### Outputs (Navigation → Hardware)
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Motor commands to robot |
| `/map` | OccupancyGrid | SLAM-generated map |
| `/plan` | Path | Planned path |
| `/local_costmap/costmap` | Costmap | Obstacle map |

### Monitoring Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/autonomous_explorer/status` | String | Explorer status |
| `/goal_pose` | PoseStamped | Current navigation goal |
| `/amcl_pose` | PoseStamped | Robot localization |

## Monitoring and Control

### Check System Status

```bash
# List all running nodes
ros2 node list

# Should see:
# - /roverrobotics_driver
# - /filtered_lidar_scanner
# - /slam_toolbox
# - /controller_server
# - /planner_server
# - /bt_navigator
# - /autonomous_explorer (if enabled)

# Check topics
ros2 topic list | grep -E "scan|cmd_vel|map"

# Monitor motor commands
ros2 topic echo /cmd_vel

# Monitor navigation status
ros2 topic echo /autonomous_explorer/status
```

### Visualization

```bash
# Launch RViz with Nav2 visualization
ros2 launch nav2_bringup rviz_launch.py

# Or launch filtered LiDAR visualization
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

### Emergency Stop

Press `Ctrl+C` in the navigation terminal, or:

```bash
# Publish zero velocity
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Or kill navigation
pkill -f hardware_navigation_stack
```

## Troubleshooting

### Robot Doesn't Move

**Check motor commands:**
```bash
ros2 topic echo /cmd_vel
```
Should show non-zero values when navigating.

**Check robot driver:**
```bash
ros2 node list | grep roverrobotics_driver
```

**Check odometry:**
```bash
ros2 topic echo /odometry/wheels
```

### Poor Navigation Performance

**Too cautious:**
- Decrease `inflation_radius` (default: 0.5 → 0.3)
- Increase speed limits

**Too aggressive:**
- Increase `inflation_radius` (default: 0.5 → 0.7)
- Decrease speed limits
- Increase `cost_scaling_factor`

**Gets stuck:**
- Check for filtered angles blocking all paths
- Reduce number of filtered angles
- Increase tolerance in filtered scanner

### SLAM Issues

**Map not building:**
```bash
# Check SLAM is running
ros2 node list | grep slam_toolbox

# Check scan topic
ros2 topic hz /scan_filtered

# Restart SLAM if needed
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

**Poor localization:**
- Drive robot around more to build better map
- Ensure LiDAR has good features to track
- Check TF tree: `ros2 run tf2_tools view_frames`

### Filtered LiDAR Issues

**Too many points filtered:**
```bash
# Check filter statistics in filtered_lidar_scanner terminal
# If >10% filtered, reduce angle list or increase tolerance

# Run with larger tolerance
./run_filtered_lidar.sh angles.txt 2.0  # ±2 degrees
```

**Not filtering enough:**
```bash
# Record new obstacles
./record_obstacles.sh 10

# Extract and use new angles
./extract_angles.sh $(ls -t close_obstacles_*.json | head -1)
./run_filtered_lidar.sh $(ls -t *_unique_angles.txt | head -1)
```

## Safety Guidelines

### Before Running

1. **Clear the area** - Remove obstacles, people, pets
2. **Check battery** - Ensure sufficient charge
3. **Test LiDAR** - `ros2 topic hz /scan` should show ~10 Hz
4. **Test motors manually** - Use WASD controller first
5. **Start slow** - Use low speeds initially

### During Operation

1. **Stay nearby** - Be ready to emergency stop
2. **Monitor status** - Watch terminal output
3. **Watch for stuck** - Robot trying same path repeatedly
4. **Check odometry** - Wheels should turn freely
5. **Monitor battery** - Stop before critically low

### After Running

1. **Stop navigation** - Ctrl+C or emergency stop
2. **Verify stopped** - Robot should be stationary
3. **Save map** - If you want to reuse it later
4. **Review logs** - Check for errors or warnings

## Advanced Usage

### Save and Load Maps

**Save current map:**
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

**Use saved map:**
```bash
ros2 launch rover_exploration hardware_navigation_stack.launch.py \
  map:=my_map.yaml
```

### Custom Navigation Goals via Code

```python
from geometry_msgs.msg import PoseStamped

# Publish goal
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 2.0
goal.pose.position.y = 1.0
goal.pose.orientation.w = 1.0

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "$goal"
```

### Integration with Other Systems

The navigation system publishes standard ROS2 topics, so you can:
- Subscribe to `/cmd_vel` to monitor commands
- Subscribe to `/map` for the current map
- Publish to `/goal_pose` to send navigation goals
- Subscribe to odometry for position tracking

## Performance Tips

### For Faster Exploration

```yaml
# In nav2_params_hardware_filtered.yaml
max_vel_x: 0.25              # Faster speed
update_frequency: 10.0       # Faster costmap updates
controller_frequency: 30.0   # Faster control loop
```

### For Better Mapping

```yaml
# Drive slower for better SLAM
max_vel_x: 0.1
max_vel_theta: 0.5

# Increase costmap range
obstacle_max_range: 3.5
```

### For Tighter Spaces

```yaml
# Smaller robot footprint padding
footprint_padding: 0.01

# Smaller inflation
inflation_radius: 0.3

# More aggressive turns
max_vel_theta: 1.2
```

## Files Created

- `hardware_navigation_stack.launch.py` - Main launch file
- `nav2_params_hardware_filtered.yaml` - Hardware-specific Nav2 config
- `run_hardware_navigation.sh` - Convenience startup script
- `HARDWARE_NAVIGATION_GUIDE.md` - This guide

## Next Steps

1. **Test in safe area** - Start with open space
2. **Tune parameters** - Adjust for your environment
3. **Build maps** - Create maps of different areas
4. **Integrate** - Add your own navigation logic
5. **Experiment** - Try different exploration strategies

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Rover Robotics Docs](https://github.com/RoverRobotics)
- Project guides: `FILTERED_LIDAR_GUIDE.md`, `LIDAR_HARDWARE_GUIDE.md`

