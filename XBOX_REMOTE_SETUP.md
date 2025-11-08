# Xbox Controller Remote Setup Guide

This guide explains how to use an Xbox controller connected to your local computer to control the robot on a Jetson over SSH.

## Problem
When SSH'd into a Jetson, joystick devices connected to your local machine are not accessible. The controller needs to be read on the local machine and the data sent over the network.

## Solution
Run `joy_linux_node` on your **local machine** (where the controller is connected) and configure ROS2 networking so the Jetson can receive the `/joy` topic.

## Setup Instructions

### 1. On Your LOCAL Machine (where Xbox controller is connected)

**a. Make sure ROS2 is installed and sourced:**
```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 distro
```

**b. Set ROS2 domain ID (must match Jetson):**
```bash
export ROS_DOMAIN_ID=0  # Use any number, but SAME on both machines
```

**c. Install joy_linux package if not already installed:**
```bash
sudo apt install ros-humble-joy-linux  # Replace 'humble' with your distro
```

**d. Run the remote joystick publisher:**
```bash
ros2 run roverrobotics_input_manager remote_joystick_publisher.py
```

The script will:
- Auto-detect available joystick devices
- Check permissions and warn if needed
- Start `joy_linux_node` to publish `/joy` topic

You should see it publishing `/joy` topic. Keep this running.

**If you need to specify a different device:**
```bash
ros2 run roverrobotics_input_manager remote_joystick_publisher.py --ros-args -p device_name:=/dev/input/js1
```

### 2. On the JETSON (via SSH)

**a. Source the workspace:**
```bash
cd ~/rover_workspace
source install/setup.bash
```

**b. Set ROS2 domain ID (MUST match local machine):**
```bash
export ROS_DOMAIN_ID=0  # MUST be same as local machine!
```

**c. Run the remote teleop launch file:**
```bash
ros2 launch roverrobotics_driver mini_teleop_xbox_remote.launch.py
```

This version does NOT try to start `joy_linux_node` locally - it expects `/joy` to come from the network.

### 3. Verify Network Connection

On the Jetson, check if `/joy` topic is available:
```bash
ros2 topic list | grep joy
ros2 topic echo /joy  # Should show joystick data
```

## Troubleshooting

### No joystick data received on Jetson

1. **Check ROS_DOMAIN_ID matches** on both machines
2. **Check firewall** - ROS2 uses UDP multicast, may need to allow it
3. **Check network** - both machines must be on same network/subnet
4. **Verify joy_linux_node is running** on local machine:
   ```bash
   ros2 topic echo /joy  # Should show data on local machine
   ```

### Permission denied accessing joystick

On local machine, the script will now auto-detect devices and warn about permissions:

**Quick fix (temporary):**
```bash
sudo chmod 666 /dev/input/js0  # Replace js0 with your device
```

**Permanent fix (recommended):**
```bash
# Add user to input group
sudo usermod -a -G input $USER
# Then logout and login again (or reboot)
```

After fixing permissions, the script will show which devices it found and can access.

### Controller not detected

On local machine:
```bash
ls -la /dev/input/js*  # List joystick devices
# Test with:
jstest /dev/input/js0  # If jstest is installed
```

## Alternative: Direct joy_linux_node

If the wrapper script doesn't work, run joy_linux_node directly on local machine:
```bash
ros2 run joy_linux joy_linux_node
```

This does the same thing - publishes `/joy` topic that can be received over network.

