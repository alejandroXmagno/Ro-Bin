# EKF Debugging Guide

## Problem
The EKF is not publishing `odom->base_link` transforms, causing SLAM to drop messages with "queue is full" errors.

## Solution: Test EKF Without Robot Movement

Since the robot moves when navigation is running, use these scripts to test EKF without movement:

### 1. Test EKF Only (No Navigation)
```bash
./test_ekf_only.sh
```
- Starts ONLY EKF node
- No SLAM, no Nav2, no movement
- Shows real-time diagnostics
- Press Ctrl+C to stop

### 2. Test EKF + SLAM (No Nav2)
```bash
./test_slam_ekf_only.sh
```
- Starts EKF and SLAM
- No Nav2, so robot won't move
- Monitors transform publishing
- Press Ctrl+C to stop

### 3. Emergency Stop
```bash
./stop_robot.sh
```
- Immediately sends zero velocity command
- Stops robot movement
- Use if robot starts moving unexpectedly

### 4. Full Diagnostics
```bash
./debug_ekf.sh
```
- Comprehensive EKF diagnostics
- Checks all components
- One-time check (not continuous)

## Running Navigation Without Movement

### Option 1: Don't Enable Exploration
```bash
./run_hardware_navigation.sh
# (without --explore flag)
```
- Starts Nav2 but no autonomous exploration
- Robot only moves if you send goals via RViz
- You can test EKF while Nav2 is running (but be ready to stop it)

### Option 2: Stop Robot Before Testing
1. Start navigation: `./run_hardware_navigation.sh --explore`
2. In another terminal, immediately run: `./stop_robot.sh`
3. Kill autonomous explorer: `pkill -f autonomous_explorer`
4. Now you can test EKF while Nav2 is running

## What to Look For

### EKF Should Show:
- ✓ EKF node running
- ✓ Subscribed to `/imu/data`
- ✓ IMU publishing at ~100 Hz
- ✓ `odom->base_link` transform exists
- ✓ `/tf` topic publishing

### If Transform Doesn't Exist:
1. Check EKF console output for errors
2. Verify IMU is publishing: `ros2 topic echo /imu/data`
3. Check EKF parameters: `ros2 param list /ekf_filter_node`
4. Look for initialization messages in EKF output

## Key Changes Made

1. **Enabled yaw orientation from IMU** - EKF needs at least one absolute measurement to initialize
2. **Enabled debug logging** - `print_diagnostics: true` and `debug: true` (console output)
3. **Created diagnostic scripts** - Test EKF without robot movement

## Next Steps

1. Run `./test_ekf_only.sh` to verify EKF is working
2. Check EKF console output for initialization messages
3. Once EKF is publishing transforms, test with SLAM using `./test_slam_ekf_only.sh`
4. If transforms are working, restart full navigation stack

