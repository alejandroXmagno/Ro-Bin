# Quick Reference: Filter Out Trash Bin Stands

## The Problem
4 stands behind the LiDAR (holding trash bin) are causing navigation to freak out.

## The Solution
Filter them out from LiDAR data in 2 easy steps!

---

## METHOD 1: Automated (Recommended) ⚡

```bash
# Terminal 1: Start robot
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Record and merge (interactive script)
./record_and_merge_filters.sh

# Follow the prompts:
# - Wait 5 seconds while it records
# - Choose option 1 to merge with existing filters
# - Copy the command it gives you

# Terminal 2: Run the command it gave you, like:
./run_hardware_navigation.sh --angles combined_filters_1234567890.txt --explore
```

**Done!** The stands are now filtered out. 🎉

---

## METHOD 2: Manual Control

```bash
# Terminal 1: Start robot
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Record the stands (5 seconds, 1 foot = 0.3048m)
./record_obstacles.sh 5 0.3048

# Extract angles (replace TIMESTAMP with actual value)
./extract_angles.sh close_obstacles_TIMESTAMP.json

# Merge with existing filters
./merge_angle_filters.sh combined.txt \
    close_obstacles_1762028491_unique_angles.txt \
    close_obstacles_TIMESTAMP_unique_angles.txt

# Run navigation with merged filter
./run_hardware_navigation.sh --angles combined.txt --explore
```

---

## Verify It's Working

Open RViz to see the filtering in action:

```bash
# Terminal 3
./view_hardware_nav.sh
```

Look for:
- 🔴 **RED points** = Original scan (includes stands)
- 🟢 **GREEN points** = Filtered scan (stands removed!)

The green points should have gaps where the stands are.

---

## Troubleshooting

### "It's still detecting the stands!"

**Option 1:** Increase the filter tolerance:
```bash
# Kill the navigation first (Ctrl+C in Terminal 2)
# Then restart with looser tolerance:
./run_filtered_lidar.sh combined.txt 3.0  # ±3 degrees instead of ±2
```

**Option 2:** Re-record with a larger distance:
```bash
./record_obstacles.sh 5 0.5  # 0.5 meters instead of 0.3048m
```

### "It recorded too many things!"

Move other objects away and record again:
```bash
# Make sure ONLY the 4 stands are within 1 foot of the LiDAR
# Then record again
./record_obstacles.sh 5 0.3048
```

### "I want to see what angles are being filtered"

```bash
cat combined.txt
# Look at the "UNIQUE ANGLES IN DEGREES" section
# Stands behind robot should be at ±150° to ±180°
```

---

## What's Happening Under the Hood

1. **Record**: Captures all LiDAR points within 1 foot for 5 seconds
2. **Extract**: Finds unique angles where obstacles were detected
3. **Merge**: Combines old filter (robot parts) + new filter (stands)
4. **Filter**: Replaces readings at those angles with `inf` (no reading)
5. **Navigate**: Robot uses filtered scan (`/scan_filtered`) for planning

---

## Commands Reference

| Task | Command |
|------|---------|
| Start robot | `ros2 launch roverrobotics_driver mini.launch.py` |
| Record obstacles | `./record_obstacles.sh 5 0.3048` |
| Extract angles | `./extract_angles.sh close_obstacles_XXX.json` |
| Merge filters | `./merge_angle_filters.sh out.txt file1.txt file2.txt` |
| Run navigation | `./run_hardware_navigation.sh --angles FILE.txt --explore` |
| View in RViz | `./view_hardware_nav.sh` |
| Test filter only | `./run_filtered_lidar.sh FILE.txt` |

---

## Expected Angle Ranges

| Location | Angle Range | Current Filter | Trash Stands |
|----------|-------------|----------------|--------------|
| Front-Right | -30° to 0° | ✗ | ✗ |
| Front-Left | 0° to +30° | ✗ | ✗ |
| Right Side | -90° to -30° | ✗ | ✗ |
| Left Side | +30° to +90° | ✗ | ✗ |
| **Rear-Right** | **-180° to -150°** | **✗** | **✓ Expected** |
| **Rear-Left** | **+150° to +180°** | **✗** | **✓ Expected** |
| Robot Parts (Right) | **-18° to -15.5°** | **✓ Filtered** | ✗ |
| Robot Parts (Left) | **+17° to +18°** | **✓ Filtered** | ✗ |

The trash bin stands should show up in the **Rear** angles (±150° to ±180°).

---

For more details, see `FILTER_TRASH_BIN_STANDS.md`

