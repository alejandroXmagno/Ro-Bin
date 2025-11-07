# Person Tracking Guide 🤖👤

Your rover now has **AI-powered person tracking** using BlazePose detection at 3Hz!

---

## 🎯 What Was Added

### 1. **BlazePose Person Detector** (3Hz)
- Detects people using MediaPipe AI
- Tracks 33 body keypoints
- Publishes detection results and visualization

### 2. **Person Navigation Logic**
- Autonomous explorer now prioritizes detected people
- Cancels exploration goals to navigate towards people
- Stops 1.5m from person (configurable)
- Resumes exploration when person is lost

---

## 🚀 How to Use

### Launch Everything

**Terminal 1: Gazebo with People**
```bash
cd ~/rover_workspace
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_GL_VERSION_OVERRIDE=3.3 OGRE_RTT_MODE=Copy
source install/setup.bash
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=depot.sdf
```

**Terminal 2: Person Detection (3Hz)**
```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration person_detection.launch.py use_sim_time:=true
```

**Terminal 3: Navigation Stack**
```bash
cd ~/rover_workspace
source install/setup.bash
ros2 launch rover_exploration navigation_stack_only.launch.py use_sim_time:=true
```

---

## 📊 Monitor Person Tracking

### Watch Detection Results
```bash
ros2 topic echo /person_detection/results
```

### View Detection Visualization
```bash
ros2 run rqt_image_view rqt_image_view /person_detection/visualization
```

### Check Robot Logs
```bash
ros2 topic echo /rosout | grep -E "(Person|person|Navigating)"
```

---

## ⚙️ Configuration

### Person Tracking Parameters
Edit `src/rover_exploration/config/exploration_params.yaml`:

```yaml
person_tracking_enabled: true  # Enable/disable person tracking
person_approach_distance: 1.5  # Stop distance from person (meters)
```

### BlazePose Parameters
Edit `src/rover_exploration/config/blazepose_params.yaml`:

```yaml
detection_rate: 3.0                # Detection frequency (Hz)
min_detection_confidence: 0.5      # Detection threshold
enable_visualization: true         # Show pose overlay
```

---

## 🎮 How It Works

### Priority System
1. **HIGHEST**: Person detected → Navigate towards person
2. **MEDIUM**: No person → Explore frontiers
3. **LOWEST**: No frontiers → Random exploration

### Detection Pipeline
```
Side RealSense Camera (RGB)
    ↓
BlazePose AI (MediaPipe) @ 3Hz
    ↓
Person Detection Results
    ↓
Autonomous Explorer
    ↓
Calculate Goal towards Person
    ↓
Nav2 Path Planning
    ↓
Robot moves towards person!
```

### Navigation Logic
- **When person detected**: Cancel current exploration goal, navigate towards person
- **Person in view**: Continuously update goal position
- **Person lost (3s)**: Resume normal exploration
- **Approach distance**: Stops 1.5m from person

---

## 🧪 Testing

### Test 1: Spawn People in Simulation
```bash
# Add person at location (x, y)
ros2 run ros_gz_sim create -world depot \
  -file https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking%20person/tip/files/model.sdf \
  -name test_person -x 2.0 -y 2.0 -z 0.0
```

### Test 2: Manual Drive to See People
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

### Test 3: Check Topics
```bash
# Person detection (should show true when person visible)
ros2 topic echo /person_detection/results | grep person_detected

# Navigation goals
ros2 topic echo /goal_pose
```

---

## 📈 Expected Behavior

### When Person Detected:
1. **BlazePose** publishes `person_detected: true` at 3Hz
2. **Autonomous Explorer** logs: `👤 Person detected at (x, y)!`
3. **Explorer** logs: `🚶 Navigating towards person at (x, y)`
4. **Robot** moves towards person
5. **Robot** stops ~1.5m away
6. **Explorer** logs: `👋 Reached person!`

### When Person Lost:
1. After 3 seconds without detection
2. **Explorer** logs: `Person lost, resuming exploration`
3. **Robot** returns to autonomous exploration mode

---

## 🔧 Troubleshooting

### Person Not Detected
```bash
# Check camera feed
ros2 topic hz /camera/color/image_raw

# Check BlazePose node
ros2 node info /blazepose_detector

# View camera to verify person is visible
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

### Robot Not Moving Towards Person
```bash
# Check if tracking is enabled
ros2 param get /autonomous_explorer person_tracking_enabled

# Enable if disabled
ros2 param set /autonomous_explorer person_tracking_enabled true

# Check explorer logs
ros2 topic echo /rosout | grep autonomous_explorer
```

### Low Detection Rate
```bash
# Increase detection rate (but uses more CPU)
ros2 param set /blazepose_detector detection_rate 5.0

# Reduce confidence threshold
ros2 param set /blazepose_detector min_detection_confidence 0.3
```

---

## 🎯 Key Features

✅ **Real-time person detection** at 3Hz using AI  
✅ **Automatic navigation** towards detected people  
✅ **Smart priority system** (people > exploration)  
✅ **Configurable approach distance** (personal space)  
✅ **Graceful fallback** to exploration when person lost  
✅ **Side-facing camera** for perpendicular detection  
✅ **Pose visualization** with skeleton overlay  
✅ **33 body keypoints** tracked per person  

---

## 📝 Files Modified

### Core Navigation
- `src/rover_exploration/rover_exploration/autonomous_explorer.py`
  - Added person detection callback
  - Added `navigate_to_person()` method
  - Priority logic in exploration callback

### Person Detection
- `src/rover_exploration/rover_exploration/blazepose_detector.py` (NEW)
  - BlazePose detection node
  - MediaPipe integration
  - Visualization

### Launch Files
- `src/rover_exploration/launch/person_detection.launch.py` (NEW)

### Configuration
- `src/rover_exploration/config/exploration_params.yaml`
  - Added `person_tracking_enabled`
  - Added `person_approach_distance`
- `src/rover_exploration/config/blazepose_params.yaml` (NEW)

---

## 🎉 Demo Scenario

```bash
# 1. Launch Gazebo with people already spawned
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=depot.sdf

# 2. Start person detection
ros2 launch rover_exploration person_detection.launch.py use_sim_time:=true

# 3. Start autonomous navigation
ros2 launch rover_exploration navigation_stack_only.launch.py use_sim_time:=true

# 4. Watch visualization
ros2 run rqt_image_view rqt_image_view /person_detection/visualization

# 5. Watch the robot autonomously find and approach people!
```

---

**Your rover is now an autonomous person-seeking robot!** 🤖👥📷

The rover will:
1. Explore the environment autonomously
2. Detect people with its side-facing camera at 3Hz
3. Immediately navigate towards detected people
4. Stop at a comfortable distance (1.5m)
5. Resume exploration when the person leaves

---

## 💡 Advanced: Multiple People

The system currently navigates to the first detected person. Future enhancements could include:
- Track multiple people
- Choose closest person
- Remember person locations
- Follow specific person by pose characteristics

---

**Status**: ✅ Person tracking fully integrated and ready to test!

