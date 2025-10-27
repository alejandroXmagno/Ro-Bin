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
