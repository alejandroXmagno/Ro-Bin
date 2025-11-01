# Close Obstacle Recording Guide

## Overview

This tool records LiDAR positions where obstacles are detected within 1 foot (0.3048 meters) of the rover over a specified time period (default: 5 seconds).

## Quick Start

### Basic Usage

**Terminal 1 - Start the robot:**
```bash
pkill -f roverrobotics_ros2_driver || true
fuser -kv /dev/ttyACM0 || true
source install/setup.bash 
ros2 launch roverrobotics_driver mini.launch.py
```

**Terminal 2 - Record close obstacles:**
```bash
./record_obstacles.sh
```

This will record for 5 seconds and display results.

## Advanced Usage

### Custom Duration
```bash
./record_obstacles.sh 10    # Record for 10 seconds
```

### Custom Distance Threshold
```bash
./record_obstacles.sh 5 0.5    # 5 seconds, 0.5 meters (1.64 feet)
```

### Direct Python Execution
```bash
python3 record_close_obstacles.py [duration] [distance_threshold]

# Examples:
python3 record_close_obstacles.py              # 5 seconds, 1 foot
python3 record_close_obstacles.py 10           # 10 seconds, 1 foot
python3 record_close_obstacles.py 5 0.2        # 5 seconds, 0.2 meters
```

## Output Format

### Console Output

The script displays:

1. **Recording Progress** - Real-time count of close readings
2. **Distance Statistics**
   - Closest obstacle detected
   - Farthest obstacle (within threshold)
   - Average distance of all close readings

3. **Angular Distribution** - 10-degree bins showing:
   - Angle range
   - Number of readings in each bin
   - Average distance for that direction

4. **Direction Summary** - Readings grouped by:
   - **Front** (±30°): Directly ahead of robot
   - **Left Side** (60° to 120°): Left side
   - **Right Side** (-120° to -60°): Right side
   - **Rear** (±150° to ±180°): Behind robot

### Example Output

```
================================================================================
CLOSE OBSTACLE RECORDING RESULTS
================================================================================
Recording Duration: 5.0 seconds
Distance Threshold: 0.305m (1 foot)
Total Close Readings: 847

DISTANCE STATISTICS:
  Closest obstacle: 0.156m (0.51 ft)
  Farthest (within threshold): 0.304m (1.00 ft)
  Average distance: 0.234m (0.77 ft)

ANGULAR DISTRIBUTION (10-degree bins):
  Angle Range  | Count | Avg Distance
  ---------------------------------------------
    -30° to  -20° |    45 | 0.245m (0.80 ft)
    -20° to  -10° |    89 | 0.223m (0.73 ft)
    -10° to    0° |   156 | 0.198m (0.65 ft)
      0° to   10° |   178 | 0.187m (0.61 ft)
     10° to   20° |   142 | 0.211m (0.69 ft)
     20° to   30° |    67 | 0.256m (0.84 ft)

OBSTACLE DIRECTIONS:
  Most obstacles at: 0° to 10° (178 readings)

READINGS BY DIRECTION:
  Front (±30°): 677 readings, avg 0.218m (0.72 ft)
  Left Side (60° to 120°): 23 readings, avg 0.289m (0.95 ft)
  Right Side (-120° to -60°): 18 readings, avg 0.291m (0.95 ft)
  Rear (150° to 180° or -180° to -150°): 0 readings

Detailed data saved to: close_obstacles_1730486789.json
================================================================================
```

### JSON Output File

Each recording creates a JSON file with detailed data:

```json
{
  "metadata": {
    "duration_seconds": 5.0,
    "threshold_meters": 0.3048,
    "threshold_feet": 1.0,
    "total_readings": 847,
    "timestamp": 1730486789
  },
  "statistics": {
    "min_distance_m": 0.156,
    "max_distance_m": 0.304,
    "avg_distance_m": 0.234
  },
  "angular_distribution": {
    "-30": 45,
    "-20": 89,
    "-10": 156,
    "0": 178,
    "10": 142,
    "20": 67
  },
  "direction_summary": {
    "Front (±30°)": 677,
    "Left Side (60° to 120°)": 23,
    "Right Side (-120° to -60°)": 18,
    "Rear (150° to 180° or -180° to -150°)": 0
  },
  "readings": [
    {
      "timestamp": 0.145,
      "angle_rad": 0.0,
      "angle_deg": 0.0,
      "distance_m": 0.234,
      "distance_ft": 0.768
    },
    ...
  ]
}
```

## Use Cases

### 1. Collision Detection Testing
Place objects at various distances and angles, then verify the LiDAR detects them correctly.

```bash
# Place object 8 inches from robot
./record_obstacles.sh 5
# Check if readings show ~0.203m (8 inches)
```

### 2. Obstacle Mapping
Drive the robot around and record obstacles to understand environment layout.

```bash
# Record for longer period while moving
./record_obstacles.sh 30
```

### 3. Safety Zone Verification
Verify the robot's immediate safety zone is clear.

```bash
# Check for obstacles within 6 inches
./record_obstacles.sh 5 0.1524
```

### 4. Navigation Algorithm Testing
Generate test data for obstacle avoidance algorithms.

```bash
# Record multiple scenarios
for i in {1..5}; do
  echo "Recording scenario $i..."
  ./record_obstacles.sh 10
  sleep 2
done
```

## Understanding the Angles

The LiDAR uses a standard coordinate system:

```
           0° (Front)
              |
              |
90° (Left) ---+--- -90° (Right)
              |
              |
        ±180° (Rear)
```

- **0°**: Directly in front of the robot
- **90°**: Left side of the robot
- **-90° (or 270°)**: Right side of the robot
- **180° or -180°**: Behind the robot

## Troubleshooting

### No Readings Recorded

If no obstacles are detected:
1. Ensure the robot is actually near objects (within 1 foot)
2. Check LiDAR is working: `ros2 topic hz /scan`
3. Visualize in RViz: `./view_lidar.sh`
4. Verify objects are in LiDAR's field of view (not below/above the scan plane)

### Script Won't Start

```bash
# Check if /scan topic is available
ros2 topic list | grep scan

# Check if robot driver is running
ros2 node list | grep roverrobotics_driver

# Or for simulation
ros2 node list | grep rplidar
```

### Permission Errors

```bash
chmod +x record_obstacles.sh
chmod +x record_close_obstacles.py
```

## Data Analysis Tips

### Using the JSON Output

Load and analyze in Python:
```python
import json
import matplotlib.pyplot as plt

# Load data
with open('close_obstacles_1730486789.json', 'r') as f:
    data = json.load(f)

# Plot distance over time
times = [r['timestamp'] for r in data['readings']]
distances = [r['distance_m'] for r in data['readings']]

plt.plot(times, distances)
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Close Obstacles Over Time')
plt.show()

# Plot polar distribution
angles = [r['angle_rad'] for r in data['readings']]
plt.polar(angles, distances, 'r.')
plt.title('Obstacle Positions (Polar)')
plt.show()
```

### Command Line Analysis

```bash
# Count total readings
jq '.metadata.total_readings' close_obstacles_*.json

# Find closest obstacle across all recordings
jq '.statistics.min_distance_m' close_obstacles_*.json | sort -n | head -1

# List files with obstacles detected in front
jq -r 'select(.direction_summary["Front (±30°)"] > 0) | input_filename' close_obstacles_*.json
```

## Integration with Other Tools

### With WASD Controller

```bash
# Terminal 1: Robot driver
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: WASD control
./wasd_controller.sh

# Terminal 3: Monitor obstacles while driving
watch -n 1 "./record_obstacles.sh 1"
```

### With Autonomous Navigation

Record obstacles during autonomous navigation to analyze the robot's environment perception:

```bash
# Start autonomous navigation, then:
./record_obstacles.sh 60    # Record for 1 minute
```

## Files Created

- `record_close_obstacles.py` - Main Python script
- `record_obstacles.sh` - Convenience bash wrapper
- `close_obstacles_TIMESTAMP.json` - Output data files (one per recording)

## Next Steps

- Analyze recorded data to improve navigation algorithms
- Use obstacle positions to test collision avoidance
- Create heat maps of obstacle density
- Compare recordings from different environments

