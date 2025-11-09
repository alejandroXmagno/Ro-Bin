# Rover Workspace - Autonomous Navigation with Person Detection

This workspace contains a ROS2-based autonomous rover that can navigate, explore, and interact with waving people using BlazePose detection.

## Features

- **Autonomous Navigation**: Frontier-based exploration with Nav2
- **Person Detection**: Real-time BlazePose detection via Intel RealSense D435i camera
- **Person Tracking**: Approaches waving people, waits 10 seconds, then continues exploring
- **Obstacle Avoidance**: Two-level collision detection with stuck recovery
- **Simulation**: Full Gazebo simulation with human actors

## Setup

```bash
cd ~/rover_workspace
source setup_rover.bash
```

## Running the Simulation

### Terminal 1: Launch Gazebo with Rover
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=depot.sdf
```

**Important**: Press the **PLAY ▶️** button in Gazebo to start the simulation!

### Terminal 2: Spawn Waving People
```bash
cd ~/rover_workspace
python3 spawn_person.py
```

This spawns 4 people with waving animations at different locations.

### Terminal 3: Launch Person Detection (BlazePose)
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 launch rover_exploration person_detection.launch.py use_sim_time:=true
```

### Terminal 4: Launch Autonomous Navigation
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 launch rover_exploration navigation_stack_only.launch.py use_sim_time:=true
```

## Expected Behavior

1. **Exploration**: Robot explores the environment autonomously
2. **Detection**: When a waving person is detected via camera
3. **Approach**: Robot cancels current goal and navigates towards the person
4. **Wait**: Robot stops 1 foot away and waits 10 seconds
5. **Resume**: Robot leaves and searches for more waving people
6. **Repeat**: Process continues for all waving people in the environment

## Monitoring

### View Camera Feed
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

### View Detection Results
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 topic echo /person_detection
```

### Monitor Navigation
```bash
cd ~/rover_workspace
source setup_rover.bash
ros2 topic echo /odom
```

## Configuration Files

- **Navigation**: `src/rover_exploration/config/nav2_params_exploration.yaml`
- **Exploration**: `src/rover_exploration/config/exploration_params.yaml`
- **Camera URDF**: `src/roverrobotics_ros2/roverrobotics_description/urdf/accessories/realsense_d435i.urdf`

## Troubleshooting

### Gazebo Doesn't Start (WSL2)
Make sure environment variables are set:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export OGRE_RTT_MODE=Copy
```

These are already set in `setup_rover.bash`.

### Camera Not Publishing
Check that Gazebo is running and the rover is spawned:
```bash
ros2 topic list | grep camera
```

### People Not Spawning
Make sure Gazebo is fully loaded before running `spawn_person.py`. Check Gazebo logs for errors.

### Robot Gets Stuck
The autonomous explorer includes stuck detection and recovery. Check parameters in `exploration_params.yaml`:
- `obstacle_distance_threshold`: 0.8m (warning level)
- `critical_obstacle_threshold`: 0.5m (immediate stop)

## Building

```bash
cd ~/rover_workspace
colcon build --symlink-install
source install/setup.bash
```

### Build Specific Package
```bash
colcon build --packages-select rover_exploration --symlink-install
```

## License

See individual package licenses for details.
