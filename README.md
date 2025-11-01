# Rover Workspace Setup

## Initial Setup

1. Copy the template file to create your local setup file:
   ```bash
   cp setup_rover.bash.template setup_rover.bash
   ```

2. The script automatically detects paths, but you can customize if needed:
   - **ROS Distribution**: Set `ROS_DISTRO` environment variable (default: `humble`)
     ```bash
     export ROS_DISTRO=foxy  # or your installed distribution
     ```
   - **ROS Installation Path**: Set `ROS_INSTALL_PATH` if installed elsewhere
     ```bash
     export ROS_INSTALL_PATH=/path/to/ros
     ```
   - **Workspace Location**: Set `WORKSPACE_PATH` if workspace is elsewhere
     ```bash
     export WORKSPACE_PATH=/path/to/workspace
     ```

3. The `setup_rover.bash` file is git-ignored so you can customize it without affecting the repository.

## Launching Autonomous Exploration

This is for launching the autonomous exploration in empty (custom) gazebo simulation.

# Terminal 1: Launch everything
cd ~/rover_workspace
source setup_rover.bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py world:=empty.sdf

# Wait ~15-20 seconds, then in Terminal 2: Unpause
ign service -s /world/empty_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 1000 --req 'pause: false'


############################

# Controlling the Robot

terminal 1 (run the kill commands EVERY TIME else it wont work)
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

terminal 2
./wasd_controller.sh

# Viewing LiDAR Data from Physical Robot

The LiDAR is now enabled by default in the accessories configuration.
When you run the robot driver (mini.launch.py), it will automatically start publishing LiDAR data on the `/scan` topic.

To visualize the LiDAR data in RViz:

terminal 1 - Start the robot driver (if not already running)
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

terminal 2 - Launch RViz to view LiDAR
./view_lidar.sh

You can also view LiDAR data with:
ros2 topic echo /scan                    # View raw scan data
ros2 topic hz /scan                      # Check scan frequency
ros2 launch rover_exploration view_lidar_hardware.launch.py  # Alternative RViz launch

## Recording Close Obstacles

To record which LiDAR positions detect obstacles less than 1 foot away over a 5-second period:

```bash
# With robot already running (mini.launch.py), in a new terminal:
./record_obstacles.sh

# Or with custom duration and distance:
./record_obstacles.sh 10 0.5    # 10 seconds, 0.5 meters threshold

# Or run directly:
python3 record_close_obstacles.py [duration_seconds] [distance_meters]
```

This will:
- Record all LiDAR readings closer than 1 foot (0.3048m) for 5 seconds
- Display statistics: closest/farthest/average distances
- Show angular distribution (which directions have obstacles)
- Categorize by direction (front, left, right, rear)
- Save detailed JSON data to a timestamped file

### Extract Unique Angles

To extract unique angles from a recording and save them as an array:

```bash
# Extract from specific file
./extract_angles.sh close_obstacles_1762028491.json

# Or extract from most recent recording
./extract_angles.sh $(ls -t close_obstacles_*.json | head -1)

# Or run Python script directly
python3 extract_unique_angles.py close_obstacles_1762028491.json
```

This creates a text file with:
- Unique angles in degrees (array format)
- Unique angles in radians (array format)
- Angle statistics (min, max, range, average)
- Distribution showing most frequent angles

### Filter LiDAR to Ignore Specific Angles

To run the LiDAR scanner while ignoring the recorded angles (useful for filtering out static obstacles or robot parts):

```bash
# Terminal 1: Robot driver (already running)
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Start filtered LiDAR scanner
./run_filtered_lidar.sh close_obstacles_1762028491_unique_angles.txt

# Terminal 3 (optional): View both original and filtered scans
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

The filtered scanner will:
- Subscribe to `/scan` (original LiDAR data)
- Filter out the specified angles by setting them to infinity
- Publish filtered data to `/scan_filtered`
- Show statistics on how many points were filtered

You can use `/scan_filtered` with navigation and SLAM algorithms to ignore known obstacles.

## Autonomous Navigation on Physical Robot

Run autonomous navigation on the physical robot using filtered LiDAR:

### Quick Start (All-in-One Script)

```bash
# Terminal 1: Start robot driver
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Start navigation stack (automatically starts filtered LiDAR)
./run_hardware_navigation.sh

# OR with autonomous exploration enabled:
./run_hardware_navigation.sh --explore

# OR specify custom angles file:
./run_hardware_navigation.sh --angles close_obstacles_1762028491_unique_angles.txt --explore
```

### Manual Setup (3 Terminals)

If you prefer more control:

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

# OR with autonomous exploration:
ros2 launch rover_exploration hardware_navigation_stack.launch.py enable_exploration:=true
```

### What This Does

The hardware navigation system:
- ✅ Uses **physical robot motors** (via `/cmd_vel` topic)
- ✅ Uses **filtered LiDAR** (`/scan_filtered`) to ignore specific angles
- ✅ Runs **SLAM** to build a map in real-time
- ✅ Runs **Nav2** for path planning and obstacle avoidance
- ✅ Optionally runs **autonomous exploration** to explore the environment
- ✅ All configured for hardware (use_sim_time: false)

