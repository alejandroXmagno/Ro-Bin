# LiDAR Hardware Setup and Usage Guide

## Overview

This guide explains how to use the RPLidar S2 sensor on the physical Rover Mini robot. The LiDAR has been configured to automatically start when you launch the robot driver.

## What Was Changed

### 1. Enabled LiDAR in Configuration
**File:** `src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml`

Changed `active: false` to `active: true` for the RPLidar configuration. This means the LiDAR will now automatically start when you run the robot driver.

### 2. Created Visualization Launch File
**File:** `src/rover_exploration/launch/view_lidar_hardware.launch.py`

A new launch file that starts RViz with a custom configuration optimized for viewing LiDAR data from the physical robot.

### 3. Created RViz Configuration
**File:** `src/rover_exploration/config/lidar_view.rviz`

Custom RViz configuration that includes:
- LaserScan display showing LiDAR data
- RobotModel display showing the robot's URDF
- TF display for coordinate frames
- Grid for spatial reference

### 4. Created Convenience Script
**File:** `view_lidar.sh`

A simple bash script that checks if the robot driver is running and launches the LiDAR visualization.

### 5. Updated Package Configuration
**File:** `src/rover_exploration/setup.py`

Updated to include `.rviz` files in the installation process.

## Usage Instructions

### Basic Setup (Two Terminals)

**Terminal 1 - Start the Robot Driver:**
```bash
cd ~/rover_workspace
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py
```

**Terminal 2 - View LiDAR Data:**
```bash
cd ~/rover_workspace
./view_lidar.sh
```

### Alternative Methods

#### Method 1: Using the launch file directly
```bash
source install/setup.bash
ros2 launch rover_exploration view_lidar_hardware.launch.py
```

#### Method 2: View raw LiDAR data in terminal
```bash
source install/setup.bash
ros2 topic echo /scan
```

#### Method 3: Check LiDAR scan frequency
```bash
source install/setup.bash
ros2 topic hz /scan
```

#### Method 4: List all LiDAR-related topics
```bash
source install/setup.bash
ros2 topic list | grep -i scan
```

## LiDAR Configuration Details

### Hardware Settings
- **Device:** RPLidar S2
- **Serial Port:** `/dev/rplidar`
- **Baudrate:** 1000000
- **Frame ID:** `lidar_link`
- **Scan Mode:** Standard
- **Scan Frequency:** 10 Hz

### ROS2 Topic
- **Topic Name:** `/scan`
- **Message Type:** `sensor_msgs/msg/LaserScan`
- **QoS:** Best Effort (typical for sensor data)

## Troubleshooting

### LiDAR Node Not Starting

If the LiDAR node doesn't start, check:

1. **Device permissions:**
   ```bash
   ls -l /dev/rplidar
   # Should show readable/writable permissions for your user
   ```

2. **Add user to dialout group (if needed):**
   ```bash
   sudo usermod -aG dialout $USER
   # Then log out and log back in
   ```

3. **Check if device exists:**
   ```bash
   ls -l /dev/rplidar
   # If it doesn't exist, try:
   ls -l /dev/ttyUSB*
   ```

4. **Update serial port in config:**
   If your LiDAR is on a different port, edit:
   ```bash
   nano src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml
   ```
   Change `serial_port: "/dev/rplidar"` to the correct port.

### No Data in RViz

If RViz shows no LiDAR data:

1. **Check if scan topic is publishing:**
   ```bash
   ros2 topic hz /scan
   ```

2. **Check for error messages:**
   Look at the terminal where you ran `mini.launch.py` for any error messages from the rplidar node.

3. **Verify TF frames:**
   ```bash
   ros2 run tf2_tools view_frames
   # This creates a PDF showing the TF tree
   ```

4. **Check RViz Fixed Frame:**
   In RViz, make sure the "Fixed Frame" in Global Options is set to `base_link` or `lidar_link`.

### Permission Denied on /dev/ttyACM0

This is normal - the robot's main controller is on `/dev/ttyACM0`, and the LiDAR is on `/dev/rplidar`. The kill commands in the instructions handle this.

### LiDAR Spinning But No Data

If the LiDAR is physically spinning but not publishing data:

1. Check the serial connection and baudrate
2. Verify the rplidar node is running:
   ```bash
   ros2 node list | grep rplidar
   ```
3. Check node logs:
   ```bash
   ros2 node info /rplidar
   ```

## Using LiDAR with Other Applications

### With SLAM (Mapping)

To use the LiDAR for SLAM on the physical robot, you'll need to create a launch file similar to `full_autonomous_slam.launch.py` but with `use_sim_time: false`.

Example launch command structure:
```bash
ros2 launch rover_exploration physical_robot_slam.launch.py use_sim_time:=false
```

### With Navigation

The LiDAR data on `/scan` can be used directly with Nav2 for obstacle avoidance and navigation. Just ensure `use_sim_time` is set to `false` in all navigation parameters.

### Recording LiDAR Data

To record LiDAR data for later analysis:
```bash
ros2 bag record /scan /tf /tf_static
```

To play it back:
```bash
ros2 bag play <bag_file>
```

## Testing the LiDAR

### Quick Test Procedure

1. Start the robot driver (Terminal 1)
2. Check if LiDAR topic is publishing:
   ```bash
   ros2 topic hz /scan
   ```
   You should see approximately 10 Hz.

3. View a single scan message:
   ```bash
   ros2 topic echo /scan --once
   ```

4. Launch RViz to visualize:
   ```bash
   ./view_lidar.sh
   ```

5. In RViz, you should see:
   - Red dots/squares showing detected obstacles
   - The robot model in the center
   - TF frames showing coordinate transformations

## Hardware Specifications

### RPLidar S2
- **Range:** 0.15m - 10m
- **Scan Rate:** 10 Hz (configurable)
- **Angular Resolution:** ~0.5 degrees
- **Samples per Scan:** ~640
- **Field of View:** 360 degrees
- **Interface:** USB/Serial

## Next Steps

Now that LiDAR is working, you can:

1. **Test navigation:** Use the LiDAR data for obstacle avoidance while driving with the WASD controller
2. **Build maps:** Run SLAM to create maps of your environment
3. **Autonomous exploration:** Use the autonomous exploration package with the physical robot
4. **Record data:** Collect LiDAR datasets for later analysis or testing

## Additional Resources

- [RPLidar ROS2 Package Documentation](https://github.com/Slamtec/rplidar_ros)
- [ROS2 LaserScan Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [RViz User Guide](https://github.com/ros2/rviz/blob/ros2/README.md)
- [Nav2 Documentation](https://navigation.ros.org/)

