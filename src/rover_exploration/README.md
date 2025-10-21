# Rover Autonomous Exploration Package

This package provides autonomous SLAM-based exploration for the Rover Robotics Mini 2WD platform. The rover can navigate any area with obstacles and explore all parts of it autonomously using frontier-based and random exploration strategies.

## Features

- **LIDAR SLAM**: Real-time mapping using slam_toolbox
- **Autonomous Navigation**: Nav2-based navigation with obstacle avoidance
- **Intelligent Exploration**: Hybrid frontier-based and random exploration
- **Safety Features**: Emergency stop on obstacle proximity, goal timeout, recovery behaviors
- **Configurable**: Easily tune exploration radius, obstacle avoidance sensitivity, and more

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Autonomous Explorer Node                  │
│  - Frontier detection                                        │
│  - Random goal generation                                    │
│  - Goal prioritization                                       │
│  - Safety monitoring                                         │
└──────────────┬────────────────────────────┬─────────────────┘
               │                            │
               ▼                            ▼
      ┌────────────────┐          ┌─────────────────┐
      │  Nav2 Stack    │          │  SLAM Toolbox   │
      │  - Path plan   │          │  - Mapping      │
      │  - Controller  │          │  - Localization │
      │  - Costmaps    │          └─────────────────┘
      └────────┬───────┘
               │
               ▼
      ┌─────────────────┐
      │  Robot Driver   │
      │  - Motor ctrl   │
      │  - LIDAR        │
      └─────────────────┘
```

## Prerequisites

- ROS 2 (Humble or later)
- Nav2 navigation stack
- slam_toolbox
- Rover Robotics ROS 2 packages

## Installation

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/rover_workspace/src
# Package should already be here if you're reading this!
```

2. Install dependencies:
```bash
cd ~/rover_workspace
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration
source install/setup.bash
```

## Usage

### Real Robot

1. **Make sure your LIDAR is configured** in the accessories.yaml file:
```bash
# Check that rplidar is set to active: true
cat src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml
```

2. **Launch the full autonomous system**:
```bash
ros2 launch rover_exploration full_autonomous_slam.launch.py
```

The rover will:
- Start up with LIDAR enabled
- Begin SLAM mapping
- Initialize navigation
- Start autonomous exploration after 10 seconds

3. **Monitor progress**:

Open RViz2 to visualize the map being built:
```bash
ros2 run rviz2 rviz2 -d src/roverrobotics_ros2/roverrobotics_description/rviz/navigation.rviz
```

Watch the console output for exploration status:
```bash
# In another terminal
ros2 topic echo /autonomous_explorer/status
```

### Simulation (Gazebo)

Test in simulation before deploying to the real robot:

```bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py
```

This will:
- Launch Gazebo with the 2WD rover
- Start SLAM and navigation
- Begin autonomous exploration

You can change the world:
```bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py world:=maze
```

## Configuration

### Exploration Parameters

Edit `config/exploration_params.yaml` to tune exploration behavior:

```yaml
autonomous_explorer:
  ros__parameters:
    exploration_radius: 6.0              # How far to search for goals (meters)
    frontier_threshold: 8                # Sensitivity of frontier detection
    min_frontier_size: 5                 # Minimum frontier cluster size
    goal_timeout: 45.0                   # Abandon goal after this time (seconds)
    obstacle_distance_threshold: 0.4     # Emergency stop distance (meters)
    random_exploration_probability: 0.3  # Probability of random vs frontier exploration
```

### Navigation Parameters

Edit `config/nav2_params_exploration.yaml` to tune navigation behavior:

Key parameters for obstacle avoidance:
- `inflation_radius`: How far around obstacles to avoid (default: 0.45m local, 0.5m global)
- `cost_scaling_factor`: How strongly to repel from obstacles (default: 4.0)
- `ObstaclesCritic.repulsion_weight`: Controller's obstacle avoidance strength (default: 2.0)
- `vx_max`: Maximum forward velocity (default: 0.4 m/s)

For more aggressive avoidance, increase:
- `inflation_radius` to 0.6-0.8
- `cost_scaling_factor` to 5.0-6.0
- `repulsion_weight` to 3.0-4.0

For faster exploration (less safe):
- Increase `vx_max` to 0.5-0.6
- Decrease `inflation_radius` to 0.3-0.4
- Decrease `repulsion_weight` to 1.0-1.5

## Exploration Strategies

The node uses two complementary strategies:

### 1. Frontier-Based Exploration
- Finds boundaries between known and unknown space
- Prioritizes closest frontiers
- Ensures systematic coverage
- Best for complete mapping

### 2. Random Exploration
- Generates random goals in free space
- Helps escape local minima
- Adds variety to exploration patterns
- Good for discovering disconnected areas

The `random_exploration_probability` parameter controls the balance between these strategies.

## Troubleshooting

### Robot doesn't move
- Check that Nav2 is running: `ros2 node list | grep controller_server`
- Verify LIDAR is publishing: `ros2 topic hz /scan`
- Check for errors: `ros2 node info /autonomous_explorer`

### Robot gets stuck
- The node has automatic recovery - it will cancel goals and find new ones
- If stuck repeatedly, reduce `vx_max` or increase `inflation_radius`
- Check for narrow passages in the environment

### No frontiers found
- If exploration radius is too small, increase `exploration_radius`
- Check that the map is being updated: `ros2 topic hz /map`
- Reduce `min_frontier_size` to detect smaller frontiers

### Crashes into obstacles
- Increase `inflation_radius` in nav2_params_exploration.yaml
- Increase `obstacle_distance_threshold` for earlier emergency stops
- Check LIDAR data quality: `ros2 topic echo /scan`

## Stopping the Robot

To safely stop autonomous exploration:

```bash
# Ctrl+C in the launch terminal
# Or send a stop command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Saving the Map

After exploration is complete, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This creates two files:
- `my_map.yaml` - Map metadata
- `my_map.pgm` - Map image

## Advanced Usage

### Using Saved Maps

Once you have a map, you can use it for localization instead of SLAM:

1. Save your map (see above)
2. Edit the navigation launch to use your map
3. Launch with localization mode

### Custom Exploration Patterns

You can modify `rover_exploration/autonomous_explorer.py` to implement custom exploration strategies:
- Coverage path planning
- Semantic exploration (target specific features)
- Multi-robot coordination
- Battery-aware exploration

## Parameters Reference

### Exploration Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exploration_radius` | float | 6.0 | Maximum distance for goal search (m) |
| `frontier_threshold` | int | 8 | Min unknown neighbors for frontier |
| `min_frontier_size` | int | 5 | Min frontier cluster size |
| `goal_timeout` | float | 45.0 | Time before abandoning goal (s) |
| `obstacle_distance_threshold` | float | 0.4 | Emergency stop distance (m) |
| `random_exploration_probability` | float | 0.3 | Probability of random exploration |

## Topics

### Subscribed Topics
- `/map` (nav_msgs/OccupancyGrid) - SLAM map
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/scan` (sensor_msgs/LaserScan) - LIDAR data

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist) - Emergency stop commands
- `/navigate_to_pose` (action) - Navigation goals

## Contributing

To improve this package:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly in simulation
4. Submit a pull request

## License

Apache 2.0

## Authors

- Autonomous Exploration System for Rover Robotics Mini 2WD
- Based on ROS 2 Nav2 and slam_toolbox

## Support

For issues and questions:
- Check the troubleshooting section above
- Review Nav2 documentation: https://navigation.ros.org/
- Review slam_toolbox documentation: https://github.com/SteveMacenski/slam_toolbox



