# Filtering Out Trash Bin Stands from LiDAR

## Problem
The 4 stands behind the LiDAR that hold up the trash bin are causing the robot to freak out during navigation. We need to filter them out from the LiDAR scan data.

## Solution: Record and Filter the Stand Angles

### Step 1: Record the Trash Bin Stand Angles

Position your robot so the LiDAR can see the 4 stands behind it, then record their angles:

```bash
# Terminal 1: Start robot driver (if not already running)
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Record obstacles for 5 seconds
# This will record all obstacles within 1 foot (0.3048m)
./record_obstacles.sh

# OR with custom settings (10 seconds, anything within 2 feet):
./record_obstacles.sh 10 0.6096
```

This creates a file like: `close_obstacles_1234567890.json`

**Important:** Make sure the robot is positioned so ONLY the trash bin stands are detected as close obstacles. Move other objects away temporarily if needed.

### Step 2: Extract Unique Angles from Recording

```bash
# Extract angles from the recording you just made
# Replace the filename with your actual recording
./extract_angles.sh close_obstacles_1234567890.json
```

This creates: `close_obstacles_1234567890_unique_angles.txt`

### Step 3: Merge with Existing Filters (Optional)

If you want to keep filtering the existing obstacles (robot parts) AND the new trash bin stands:

```bash
# Merge the old filter (robot parts) with new filter (trash bin stands)
./merge_angle_filters.sh combined_robot_and_stands.txt \
    close_obstacles_1762028491_unique_angles.txt \
    close_obstacles_1234567890_unique_angles.txt
```

OR just use the new trash bin stands filter alone:

```bash
# Just filter the trash bin stands
cp close_obstacles_1234567890_unique_angles.txt trash_bin_stands_filter.txt
```

### Step 4: Run Navigation with Filtered LiDAR

Now run your robot with the filtered LiDAR:

```bash
# Terminal 1: Robot driver (already running from Step 1)
# ...keep it running...

# Terminal 2: Start navigation with filtered LiDAR
# Option A: Use merged filter (robot parts + trash bin stands)
./run_hardware_navigation.sh --angles combined_robot_and_stands.txt --explore

# Option B: Use only trash bin stands filter
./run_hardware_navigation.sh --angles trash_bin_stands_filter.txt --explore

# Option C: Auto-detect most recent filter (looks for latest recording)
./run_hardware_navigation.sh --explore
```

### Step 5: Visualize (Optional)

To see what's being filtered in real-time:

```bash
# In a third terminal
./view_hardware_nav.sh
```

In RViz you'll see:
- 🔴 **RED points** - Original LiDAR scan (includes the stands)
- 🟢 **GREEN points** - Filtered LiDAR scan (stands removed!)

## Troubleshooting

### "I recorded but it's filtering too much"

The default tolerance is ±2°. Try reducing it:

```bash
# Use ±1 degree tolerance (stricter filtering)
./run_filtered_lidar.sh trash_bin_stands_filter.txt 1.0
```

### "The stands are still showing up"

1. Check if you recorded the right angles:
   ```bash
   # View the recorded angles
   cat trash_bin_stands_filter.txt
   ```
   
2. The stands should be at angles around ±150° to ±180° (rear of robot)
   
3. Try increasing the tolerance:
   ```bash
   # Use ±3 degree tolerance (looser filtering)
   ./run_filtered_lidar.sh trash_bin_stands_filter.txt 3.0
   ```

### "I want to start fresh"

1. Record only the trash bin stands (remove other obstacles temporarily)
2. Create a new filter from scratch
3. Test it first with `./run_filtered_lidar.sh` and `./view_lidar.sh` before using for navigation

## Quick Commands Summary

```bash
# 1. Record obstacles
./record_obstacles.sh 5 0.3048

# 2. Extract angles
./extract_angles.sh close_obstacles_TIMESTAMP.json

# 3. (Optional) Merge with existing
./merge_angle_filters.sh combined.txt old_filter.txt new_filter.txt

# 4. Run navigation with filter
./run_hardware_navigation.sh --angles combined.txt --explore

# 5. Visualize
./view_hardware_nav.sh
```

## What Angles to Expect for "Behind the LiDAR"

If the stands are truly **behind** the LiDAR (rear of robot), you should see angles around:
- **150° to 180°** (rear-right)
- **-150° to -180°** (rear-left)

Your current filter (`close_obstacles_1762028491_unique_angles.txt`) shows:
- **-17.96° to -15.56°** (front-right area)
- **+17.16° to +18.16°** (front-left area)

So the trash bin stands should be at completely different angles!

## Advanced: Manual Angle Entry

If you know exactly which angles to filter, you can create a custom filter file:

```bash
cat > custom_filter.txt << 'EOF'
================================================================================
CUSTOM ANGLE FILTER - Trash Bin Stands
================================================================================

UNIQUE ANGLES IN DEGREES (sorted):
--------------------------------------------------------------------------------
[
  -175.0,
  -165.0,
  165.0,
  175.0
]

UNIQUE ANGLES IN RADIANS (sorted):
--------------------------------------------------------------------------------
[
  -3.0543261909900763,
  -2.8797932657906435,
  2.8797932657906435,
  3.0543261909900763
]
EOF

# Use the custom filter
./run_hardware_navigation.sh --angles custom_filter.txt --explore
```

