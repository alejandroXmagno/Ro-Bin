# This is for launching the autonomous exploration in empty (custom) gazebo simulation.

# Terminal 1: Launch everything
cd ~/rover_workspace
source setup_rover.bash
ros2 launch rover_exploration simulation_autonomous_slam.launch.py world:=empty.sdf

# Wait ~15-20 seconds, then in Terminal 2: Unpause
ign service -s /world/empty_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 1000 --req 'pause: false'
