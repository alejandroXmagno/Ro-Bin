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

## Viewing RealSense Camera

The RealSense is connected to the Jetson. View what it sees using one of these methods:

### Method 1: Web Browser (Easiest - Works from Any Device!)

```bash
# On Jetson: Start robot
ros2 launch roverrobotics_driver mini.launch.py

# On Jetson: Start web stream
./stream_realsense_web.sh

# On your laptop/phone/tablet: Open browser to:
# http://<jetson-ip>:5000/
```

View from **any device** on your network! Perfect for monitoring from your laptop while robot drives around.

### Method 2: RViz (Best for Navigation)

```bash
# On Jetson: Start robot
ros2 launch roverrobotics_driver mini.launch.py

# On your laptop: Open RViz
./view_realsense_rviz.sh
```

Shows camera feeds + robot position + LiDAR in one view.

### Method 3: OpenCV Viewer (If SSH'd with X11)

```bash
# SSH to Jetson with X11 forwarding
ssh -X stickykeys@<jetson-ip>

# On Jetson: View camera
./view_realsense.sh
```

**Keyboard controls:**
- `d` - Toggle depth view
- `i` - Toggle infrared view
- `c` - Cycle depth colormap
- `q` - Quit

### Method 4: Check Topics

```bash
# Check if RealSense is working
./check_realsense.sh

# View raw data
ros2 topic echo /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw
```

**Recommended:** Use **Method 1 (Web Browser)** for easiest remote viewing!

See `REALSENSE_VIEWER_GUIDE.md` for detailed guide and troubleshooting.

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

# Terminal 2: Start filtered LiDAR scanner (±2° tolerance by default)
./run_filtered_lidar.sh close_obstacles_1762028491_unique_angles.txt

# Terminal 3 (optional): View both original and filtered scans
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

The filtered scanner will:
- Subscribe to `/scan` (original LiDAR data)
- Filter out the specified angles **±2 degrees** (for robustness)
- Publish filtered data to `/scan_filtered`
- Show statistics on how many points were filtered

**Angle tolerance:** Default is ±2.0° to catch angles close to recorded ones. Adjust with:
```bash
./run_filtered_lidar.sh angles.txt 1.0   # ±1 degree (stricter)
./run_filtered_lidar.sh angles.txt 3.0   # ±3 degrees (looser)
```

You can use `/scan_filtered` with navigation and SLAM algorithms to ignore known obstacles.

### Filter Multiple Types of Obstacles (e.g., Robot Parts + Trash Bin Stands)

If you need to filter multiple types of obstacles (like robot parts AND external stands):

**Quick Method (Automated):**
```bash
# Terminal 1: Start robot driver
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Record and merge (interactive)
./record_and_merge_filters.sh
# This will:
# 1. Record new obstacles
# 2. Extract angles
# 3. Ask if you want to merge with existing filters
# 4. Give you the command to use the merged filter
```

**Manual Method:**
```bash
# 1. Record the new obstacles (e.g., trash bin stands behind LiDAR)
./record_obstacles.sh 5 0.3048

# 2. Extract angles from the new recording
./extract_angles.sh close_obstacles_NEW_TIMESTAMP.json

# 3. Merge with existing filter
./merge_angle_filters.sh combined_filters.txt \
    close_obstacles_1762028491_unique_angles.txt \
    close_obstacles_NEW_TIMESTAMP_unique_angles.txt

# 4. Use merged filter for navigation
./run_hardware_navigation.sh --angles combined_filters.txt --explore
```

**Example Use Case - Filtering Trash Bin Stands:**
If you have 4 stands behind the LiDAR holding up a trash bin that cause navigation issues:
1. Position robot so LiDAR sees only those stands
2. Run `./record_and_merge_filters.sh`
3. Choose option 1 to merge with existing filters
4. Use the merged filter for navigation

See `FILTER_TRASH_BIN_STANDS.md` for detailed guide.

## Autonomous Navigation on Physical Robot

Run autonomous navigation on the physical robot using filtered LiDAR:

### Quick Start (All-in-One Script)

```bash
# Terminal 1: Start robot driver
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Start navigation with automatic filtered LiDAR
./run_hardware_navigation.sh --explore
```

**That's it!** The script will:
- ✅ Auto-detect your most recent obstacle recording
- ✅ Start filtered LiDAR with ±2° tolerance
- ✅ Use `/scan_filtered` for navigation
- ✅ Enable autonomous exploration

**Advanced options:**
```bash
# Manual goals only (no exploration)
./run_hardware_navigation.sh

# Specify custom angles file
./run_hardware_navigation.sh --angles my_angles.txt --explore

# Without any angles file (unfiltered)
./run_hardware_navigation.sh --explore
# (Will warn and continue with /scan after 3 seconds)
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
- ✅ **TUNED TO PREVENT JITTER AND STUCK BEHAVIOR** (see NAVIGATION_FIXES.md)

### Visualization with RViz2

To see LiDAR scans and path planning in action:

```bash
# Terminal 1: Robot + Navigation (already running)
./run_hardware_navigation.sh --explore

# Terminal 2: Open RViz2 visualization
./view_hardware_nav.sh
```

**What you'll see in RViz:**
- 🔴 **RED points** - Original LiDAR scan (`/scan`)
- 🟢 **GREEN points** - Filtered LiDAR scan (`/scan_filtered`)
- 🟢 **GREEN line** - Global path plan
- 🟡 **YELLOW line** - Local path plan  
- 🗺️ **Gray map** - SLAM-generated map
- 🎯 **Orange marker** - Current navigation goal
- 🤖 **Robot model** - Your rover with footprint
- **Blue/purple overlay** - Local costmap (obstacle inflation)

**Tools:**
- Use "2D Goal Pose" to manually send the robot to a location
- Use "2D Pose Estimate" to correct robot localization
- Use "Measure" to measure distances on the map

