# Filtered LiDAR Scanner Guide

## Overview

The Filtered LiDAR Scanner allows you to ignore specific angles from LiDAR scans. This is useful for:
- Filtering out robot parts that appear in the scan
- Ignoring known static obstacles
- Creating custom scanning zones
- Testing navigation algorithms with selective obstacle awareness

## How It Works

The scanner:
1. Subscribes to the original `/scan` topic
2. Filters out readings at specified angles by setting them to infinity
3. Publishes the filtered data to `/scan_filtered`
4. Maintains statistics on filtering efficiency

## Quick Start

### Basic Usage (3 Terminals)

**Terminal 1 - Start the robot:**
```bash
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py
```

**Terminal 2 - Start the filtered scanner:**
```bash
./run_filtered_lidar.sh close_obstacles_1762028491_unique_angles.txt
```

**Terminal 3 - Visualize (optional):**
```bash
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

## Input File Formats

The filtered scanner accepts two file formats:

### 1. Unique Angles Text File (.txt)
```
UNIQUE ANGLES IN RADIANS (sorted):
--------------------------------------------------------------------------------
[
  -0.3134411371,
  -0.3099582400,
  ...
]
```

### 2. Obstacle Recording JSON (.json)
```json
{
  "readings": [
    {
      "angle_rad": -0.3134411371,
      "angle_deg": -17.958854,
      ...
    }
  ]
}
```

## Advanced Usage

### Custom Tolerance

Control how precisely angles must match to be filtered:

```bash
# Default tolerance (0.57 degrees)
./run_filtered_lidar.sh angles.txt

# Larger tolerance (filter nearby angles too)
./run_filtered_lidar.sh angles.txt 1.0    # ±1 degree

# Stricter tolerance (exact matches only)
./run_filtered_lidar.sh angles.txt 0.1    # ±0.1 degrees
```

### Direct Python Execution

```bash
# Basic usage
python3 filtered_lidar_scanner.py angles.txt

# With custom tolerance
python3 filtered_lidar_scanner.py angles.json 0.5

# Using most recent recording
python3 filtered_lidar_scanner.py $(ls -t close_obstacles_*.json | head -1)
```

## ROS2 Topics

### Input
- **Topic**: `/scan`
- **Type**: `sensor_msgs/msg/LaserScan`
- **Description**: Original LiDAR data from the robot

### Output
- **Topic**: `/scan_filtered`
- **Type**: `sensor_msgs/msg/LaserScan`
- **Description**: Filtered LiDAR data with specified angles set to infinity

## Visualization

The filtered LiDAR view shows:
- **Red points**: Original scan data (`/scan`)
- **Green points**: Filtered scan data (`/scan_filtered`)
- Missing green points show which angles were filtered out

### Launch RViz
```bash
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

## Statistics and Monitoring

The scanner logs statistics every 50 scans:

```
[filtered_lidar_scanner]: Processed 50 scans | Filtered 950/32000 points (2.97%)
[filtered_lidar_scanner]: Processed 100 scans | Filtered 1900/64000 points (2.97%)
```

### Check Topics
```bash
# List available topics
ros2 topic list | grep scan

# View original scan rate
ros2 topic hz /scan

# View filtered scan rate
ros2 topic hz /scan_filtered

# Echo filtered scan
ros2 topic echo /scan_filtered
```

## Use Cases

### 1. Filter Out Robot Parts

If your LiDAR detects parts of the robot itself:

```bash
# Record the robot's self-obstruction
./record_obstacles.sh 3

# Filter it out
./run_filtered_lidar.sh $(ls -t close_obstacles_*.json | head -1)
```

### 2. Ignore Static Obstacles

For testing in a known environment:

```bash
# Record static obstacles (walls, furniture, etc.)
./record_obstacles.sh 10

# Extract angles
./extract_angles.sh $(ls -t close_obstacles_*.json | head -1)

# Run with filtered view
./run_filtered_lidar.sh $(ls -t *_unique_angles.txt | head -1)
```

### 3. Custom Scanning Zones

Create multiple angle files for different scenarios:

```bash
# Save specific angle sets
cp close_obstacles_1762028491_unique_angles.txt front_blind_spot.txt

# Use different filters
./run_filtered_lidar.sh front_blind_spot.txt    # Filter front
./run_filtered_lidar.sh side_blind_spots.txt    # Filter sides
```

### 4. Navigation Testing

Use filtered scans with navigation algorithms:

```bash
# Remap navigation to use filtered scan
ros2 run nav2_controller controller_server --ros-args \
  --remap scan:=/scan_filtered
```

## Integration with Other Systems

### With SLAM

Edit SLAM parameters to use filtered scan:

```yaml
# In slam_params.yaml
scan_topic: /scan_filtered
```

### With Nav2

Edit navigation parameters:

```yaml
# In nav2_params.yaml
local_costmap:
  scan:
    topic: /scan_filtered
global_costmap:
  scan:
    topic: /scan_filtered
```

### Recording Filtered Data

Record the filtered scan for later analysis:

```bash
ros2 bag record /scan_filtered /tf /tf_static
```

## Performance Considerations

### Computational Cost
- Very low overhead (~1-2% CPU)
- No significant latency added
- Scales with number of angles to filter

### Memory Usage
- Minimal memory footprint
- Stores set of angles to ignore
- No buffering of scan data

### Network Bandwidth
- Same as original scan (no compression)
- Two topics active simultaneously
- Consider disabling `/scan` topic if not needed

## Troubleshooting

### No Filtered Topic

If `/scan_filtered` is not appearing:

```bash
# Check if scanner is running
ros2 node list | grep filtered_lidar_scanner

# Check for errors in terminal running filtered scanner
```

### Too Many/Few Points Filtered

Adjust the tolerance:

```bash
# If too many points filtered
./run_filtered_lidar.sh angles.txt 0.1  # Stricter matching

# If too few points filtered
./run_filtered_lidar.sh angles.txt 2.0  # Looser matching
```

### Scanner Won't Start

```bash
# Ensure /scan topic exists
ros2 topic list | grep scan

# Check file exists
ls -l close_obstacles_*_unique_angles.txt

# Check file permissions
chmod +x filtered_lidar_scanner.py run_filtered_lidar.sh
```

### RViz Shows No Data

1. Check topics are publishing:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /scan_filtered
   ```

2. Verify RViz is subscribed to correct topics:
   - Original Scan: `/scan`
   - Filtered Scan: `/scan_filtered`

3. Check Fixed Frame is set to `base_link` or `lidar_link`

## Example Workflows

### Workflow 1: Quick Filter

```bash
# 1. Start robot
ros2 launch roverrobotics_driver mini.launch.py

# 2. Record obstacles
./record_obstacles.sh

# 3. Filter using that recording
./run_filtered_lidar.sh $(ls -t close_obstacles_*.json | head -1)

# 4. View results
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

### Workflow 2: Persistent Filter

```bash
# 1. Create a permanent angle file
./extract_angles.sh close_obstacles_1762028491.json
cp close_obstacles_1762028491_unique_angles.txt permanent_blind_spots.txt

# 2. Always use this filter
./run_filtered_lidar.sh permanent_blind_spots.txt

# 3. Can update the file anytime and restart the scanner
```

### Workflow 3: Multiple Filters

```bash
# Create different filters for different scenarios
./extract_angles.sh front_obstacles.json
mv *_unique_angles.txt front_filter.txt

./extract_angles.sh side_obstacles.json
mv *_unique_angles.txt side_filter.txt

# Use as needed
./run_filtered_lidar.sh front_filter.txt    # When testing front sensors
./run_filtered_lidar.sh side_filter.txt     # When testing side sensors
```

## Technical Details

### Angle Matching Algorithm

The scanner uses tolerance-based matching:

```python
def should_ignore_angle(angle_to_check, ignored_angle, tolerance):
    return abs(angle_to_check - ignored_angle) <= tolerance
```

### Filter Implementation

Filtered points are set to `float('inf')` which:
- Indicates "no reading" in LaserScan standard
- Is ignored by most navigation algorithms
- Preserves the scan array structure
- Maintains angle_increment consistency

### Data Preservation

The filtered scan preserves:
- Header and timestamp
- angle_min, angle_max, angle_increment
- range_min, range_max
- Intensities (if present)
- All other LaserScan fields

## Files Created

- `filtered_lidar_scanner.py` - Main filtering node
- `run_filtered_lidar.sh` - Convenience launcher
- `view_filtered_lidar.launch.py` - RViz launch file
- `filtered_lidar_view.rviz` - RViz configuration

## Next Steps

- Use filtered scans with autonomous navigation
- Create angle libraries for common scenarios
- Integrate with obstacle avoidance algorithms
- Compare navigation performance with/without filtering
- Build custom angle filters for specific tasks

