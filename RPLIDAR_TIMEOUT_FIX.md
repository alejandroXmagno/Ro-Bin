# RPLidar Timeout Error Fix

## The Error You're Seeing

```
[rplidar_composition-2] [ERROR] [1762548250.720116555] [rplidar]: 
  Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!
[ERROR] [rplidar_composition-2]: process has died [pid 10055, exit code 255]
```

## Quick Fix (Try This First!)

```bash
# Stop everything
pkill -f roverrobotics_ros2_driver
pkill -f rplidar_composition

# Run the fix script
./fix_rplidar_timeout.sh

# Then restart the robot
source install/setup.bash
ros2 launch roverrobotics_driver mini.launch.py
```

---

## If Quick Fix Doesn't Work

### Method 1: Physical Reset (Most Reliable)

1. **Stop all processes:**
   ```bash
   pkill -f roverrobotics_ros2_driver
   pkill -f rplidar_composition
   ```

2. **UNPLUG the RPLidar USB cable**
   - Wait 5 seconds
   
3. **PLUG it back in**
   - Wait for the LED to start spinning (indicates power)
   
4. **Restart the robot driver:**
   ```bash
   pkill -f roverrobotics_ros2_driver || true
   fuser -kv /dev/ttyACM0 || true
   source install/setup.bash
   ros2 launch roverrobotics_driver mini.launch.py
   ```

### Method 2: Check Device Permissions

```bash
# Run diagnostic
./check_rplidar.sh

# Look for permission issues
# If device is not writable, fix with:
sudo chmod 666 /dev/ttyUSB0  # or /dev/ttyUSB1 or /dev/rplidar
```

### Method 3: Check for Stale Processes

```bash
# Check what's using the device
./cleanup_rplidar.sh

# Or manually check:
lsof /dev/ttyUSB* 2>/dev/null
ps aux | grep rplidar
```

---

## Common Causes

| Cause | Symptom | Fix |
|-------|---------|-----|
| **Stale process** | Device locked by old process | `pkill -9 -f rplidar_composition` |
| **No power** | LED not spinning | Check USB connection, try different port |
| **Permissions** | Device not readable/writable | `sudo chmod 666 /dev/ttyUSB0` |
| **Wrong device** | Looking at wrong USB port | Check `/dev/rplidar` or `/dev/ttyUSB*` |
| **USB hub issue** | Intermittent connection | Plug directly into computer USB port |
| **Cable damage** | Random disconnects | Try different USB cable |

---

## Verify It's Working

After fixing, you should see:

```bash
# Check if scan data is being published
ros2 topic hz /scan

# Should output something like:
# average rate: 10.000
#   min: 0.099s max: 0.101s std dev: 0.00050s window: 10
```

If `/scan` topic is publishing, your RPLidar is working! 🎉

---

## Prevention

To avoid this issue in the future:

1. **Always kill processes cleanly:**
   ```bash
   pkill -f roverrobotics_ros2_driver || true
   pkill -f rplidar_composition || true
   ```

2. **Check before launching:**
   ```bash
   ./check_rplidar.sh
   ```

3. **Use the cleanup script:**
   ```bash
   ./cleanup_rplidar.sh
   ```

---

## Still Not Working?

### Advanced Diagnostics

```bash
# Check USB device detection
lsusb | grep -i "10c4:ea60"
# Should see: "Silicon Labs CP210x UART Bridge"

# Check available serial devices
ls -la /dev/ttyUSB*

# Check udev rules
ls -la /etc/udev/rules.d/*rplidar*

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Hardware Check

1. **LED Indicator:**
   - Spinning = Good (powered and ready)
   - Not spinning = Bad (no power or fault)

2. **USB Connection:**
   - Try different USB port
   - Try direct connection (not through hub)
   - Check cable for damage

3. **Power:**
   - RPLidar needs USB 5V power
   - Some USB ports may not provide enough current
   - Try a powered USB hub

---

## Quick Reference Commands

| Task | Command |
|------|---------|
| Fix timeout | `./fix_rplidar_timeout.sh` |
| Diagnose issue | `./check_rplidar.sh` |
| Clean up processes | `./cleanup_rplidar.sh` |
| Check if working | `ros2 topic hz /scan` |
| View scan data | `ros2 topic echo /scan` |
| Kill all processes | `pkill -f rplidar && pkill -f roverrobotics` |

---

## Once Fixed, Continue With Filtering

After RPLidar is working, you can continue with filtering the trash bin stands:

```bash
# Terminal 1: Robot driver (should be working now)
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Record obstacles
./record_and_merge_filters.sh
```

See `QUICK_FILTER_STANDS.md` for details.

