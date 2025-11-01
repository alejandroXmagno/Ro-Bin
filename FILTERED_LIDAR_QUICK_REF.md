# Filtered LiDAR Quick Reference

## 🚀 Quick Start

```bash
# Terminal 1: Start robot
ros2 launch roverrobotics_driver mini.launch.py

# Terminal 2: Run filtered scanner
./run_filtered_lidar.sh close_obstacles_1762028491_unique_angles.txt

# Terminal 3: Visualize (optional)
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

## 📋 Common Commands

```bash
# Use most recent recording
./run_filtered_lidar.sh $(ls -t close_obstacles_*_unique_angles.txt | head -1)

# Use JSON file directly
./run_filtered_lidar.sh close_obstacles_1762028491.json

# Custom tolerance (degrees)
./run_filtered_lidar.sh angles.txt 1.0

# Direct Python
python3 filtered_lidar_scanner.py angles.txt
```

## 📊 Topics

| Topic | Description |
|-------|-------------|
| `/scan` | Original LiDAR data (RED in RViz) |
| `/scan_filtered` | Filtered data (GREEN in RViz) |

## 🔧 Key Features

- ✅ Filters specific angles from LiDAR scans
- ✅ Publishes to `/scan_filtered` topic
- ✅ Shows real-time statistics
- ✅ Works with .txt or .json angle files
- ✅ Configurable tolerance
- ✅ Zero-latency filtering

## 📈 Monitoring

```bash
# Check filtered topic
ros2 topic hz /scan_filtered

# View filtered data
ros2 topic echo /scan_filtered

# List scan topics
ros2 topic list | grep scan
```

## 🎯 Use Cases

1. **Filter robot parts** - Remove self-detections
2. **Ignore static obstacles** - Skip known environment features  
3. **Create scan zones** - Define custom detection areas
4. **Test navigation** - Selective obstacle awareness

## 🎨 Visualization

**RViz Color Coding:**
- 🔴 Red = Original scan (`/scan`)
- 🟢 Green = Filtered scan (`/scan_filtered`)
- Missing green = Filtered angles

## ⚙️ Configuration

**Default tolerance:** ±0.57° (0.01 radians)

**Adjust tolerance:**
```bash
./run_filtered_lidar.sh angles.txt 0.5   # Strict (±0.5°)
./run_filtered_lidar.sh angles.txt 2.0   # Loose (±2.0°)
```

## 📖 Full Documentation

- **Comprehensive Guide:** `FILTERED_LIDAR_GUIDE.md`
- **Obstacle Recording:** `OBSTACLE_RECORDING_GUIDE.md`
- **LiDAR Setup:** `LIDAR_HARDWARE_GUIDE.md`
- **Main README:** `README.md`

## 🔄 Workflow Example

```bash
# 1. Record obstacles
./record_obstacles.sh

# 2. Extract angles  
./extract_angles.sh $(ls -t close_obstacles_*.json | head -1)

# 3. Filter LiDAR
./run_filtered_lidar.sh $(ls -t *_unique_angles.txt | head -1)

# 4. Visualize
ros2 launch rover_exploration view_filtered_lidar.launch.py
```

## 🆘 Troubleshooting

| Problem | Solution |
|---------|----------|
| No `/scan_filtered` topic | Check if scanner is running |
| Too many points filtered | Reduce tolerance value |
| Too few points filtered | Increase tolerance value |
| Scanner won't start | Ensure `/scan` topic exists |
| No data in RViz | Check Fixed Frame = `base_link` |

## 📁 Files

- `filtered_lidar_scanner.py` - Main node
- `run_filtered_lidar.sh` - Launcher script
- `view_filtered_lidar.launch.py` - RViz launcher
- `filtered_lidar_view.rviz` - RViz config

