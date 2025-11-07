# Person Tracking Update - Wait Behavior 🤖⏳

## 🎯 Changes Made

Your rover now has enhanced person tracking behavior:

### New Behavior:
1. ✅ **Approaches to 1 foot (0.3m)** from detected person
2. ✅ **Waits 10 seconds** by the person  
3. ✅ **Looks for another person** after waiting
4. ✅ **Repeats the cycle** for each detected person

---

## 📦 What Was Modified

### 1. **Approach Distance** → Changed from 1.5m to 0.3m (~1 foot)
**File**: `src/rover_exploration/config/exploration_params.yaml`
```yaml
person_approach_distance: 0.3  # Now 1 foot instead of 1.5 meters
```

### 2. **Added Waiting State**
**File**: `src/rover_exploration/rover_exploration/autonomous_explorer.py`

**New state variables:**
```python
self.waiting_by_person = False         # Is robot waiting by person?
self.person_wait_start_time = None     # When did waiting start?
self.person_wait_duration = 10.0       # Wait 10 seconds
```

**New logic:**
- When robot reaches person (within 0.3m):
  - Sets `waiting_by_person = True`
  - Starts 10-second timer
  - Cancels navigation goal
  - Logs: "👋 Reached person! Waiting 10 seconds..."

- While waiting:
  - Stays in place
  - Logs countdown every 2 seconds: "⏳ Waiting by person... Xs remaining"
  - Ignores new person detections (stays with current person)

- After 10 seconds:
  - Sets `waiting_by_person = False`
  - Logs: "✅ Wait complete! Looking for another person..."
  - Resumes exploration/person detection

---

## 🔄 New Flow Diagram

```
Person Detected (via camera @ 3Hz)
    ↓
Navigate towards person
    ↓
Reached person (< 0.3m / 1 foot)
    ↓
Stop and Wait 10 seconds ⏳
    ↓
    ├─ 2s: "⏳ Waiting... 8s remaining"
    ├─ 4s: "⏳ Waiting... 6s remaining"
    ├─ 6s: "⏳ Waiting... 4s remaining"
    ├─ 8s: "⏳ Waiting... 2s remaining"
    └─ 10s: "✅ Wait complete!"
    ↓
Look for another person
    ↓
    ├─ Person found? → Navigate to them (repeat)
    └─ No person? → Resume exploration
```

---

## 🚀 How to Test

### Rebuild the Package
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration --symlink-install
source install/setup.bash
```

### Launch Everything
```bash
# Terminal 1: Gazebo with people
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
  MESA_GL_VERSION_OVERRIDE=3.3 OGRE_RTT_MODE=Copy
ros2 launch roverrobotics_gazebo 2wd_rover_gazebo.launch.py world:=depot.sdf

# Terminal 2: Person detection
ros2 launch rover_exploration person_detection.launch.py use_sim_time:=true

# Terminal 3: Autonomous navigation with person tracking
ros2 launch rover_exploration navigation_stack_only.launch.py use_sim_time:=true
```

### Watch It Work
```bash
# Monitor robot's actions
ros2 topic echo /rosout | grep -E "(Reached|Waiting|Looking|Navigating)"

# You should see:
# "🚶 Navigating towards person..."
# "👋 Reached person! Waiting 10 seconds..."
# "⏳ Waiting by person... 8s remaining"
# "⏳ Waiting by person... 6s remaining"
# "⏳ Waiting by person... 4s remaining"
# "⏳ Waiting by person... 2s remaining"
# "✅ Wait complete! Looking for another person..."
```

---

## 🎮 Expected Behavior

### Scenario 1: Single Person
1. Robot explores
2. Camera detects person
3. Robot navigates to person
4. Robot stops 1 foot away
5. Robot waits 10 seconds
6. Robot resumes exploration (person still there but already "visited")

### Scenario 2: Multiple People
1. Robot detects Person A
2. Navigates to Person A
3. Waits 10 seconds by Person A
4. Looks for another person
5. Detects Person B
6. Navigates to Person B
7. Waits 10 seconds by Person B
8. Repeats...

### Scenario 3: Person Moves Away
1. Robot navigating to person
2. Person moves out of camera view
3. After 3 seconds of no detection:
   - Robot stops tracking
   - Logs: "Person lost, resuming exploration"
   - Resumes exploration

---

## ⚙️ Configuration Options

### Change Wait Duration
Edit `src/rover_exploration/rover_exploration/autonomous_explorer.py`:
```python
self.person_wait_duration = 10.0  # Change to desired seconds
```

### Change Approach Distance
Edit `src/rover_exploration/config/exploration_params.yaml`:
```yaml
person_approach_distance: 0.3  # Change to desired meters
# Examples:
# 0.3 = ~1 foot (current)
# 0.5 = ~1.5 feet
# 1.0 = ~3 feet
```

### Disable Waiting (approach only)
Edit `src/rover_exploration/rover_exploration/autonomous_explorer.py`:
```python
self.person_wait_duration = 0.0  # No waiting, immediately look for next
```

---

## 📊 Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Approach Distance** | 0.3m | Stop 1 foot from person |
| **Wait Duration** | 10s | Time to wait by person |
| **Detection Rate** | 3Hz | Person detection frequency |
| **Lost Timeout** | 3s | Time before person considered "lost" |
| **Detection Confidence** | 0.5 | AI confidence threshold |

---

## 🔧 Troubleshooting

### Robot Doesn't Wait
```bash
# Check if waiting state is active
ros2 topic echo /rosout | grep waiting_by_person

# Verify approach distance
ros2 param get /autonomous_explorer person_approach_distance
# Should show: 0.3
```

### Robot Doesn't Get Close Enough
```bash
# Increase approach distance (if obstacles preventing)
ros2 param set /autonomous_explorer person_approach_distance 0.5

# Or edit config file and rebuild
```

### Wait Duration Too Short/Long
Edit the code and change:
```python
self.person_wait_duration = 15.0  # For 15 seconds
# or
self.person_wait_duration = 5.0   # For 5 seconds
```
Then rebuild:
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration --symlink-install
```

---

## 🎯 Summary of Changes

### Before:
- Approached to 1.5m
- Immediately looked for next person
- No waiting behavior

### After:
- ✅ Approaches to 0.3m (1 foot)
- ✅ Waits 10 seconds by person
- ✅ Logs countdown
- ✅ Then looks for another person
- ✅ Repeats cycle

---

## 💡 Future Enhancements

Possible improvements:
- **Wave or gesture detection** - Wait until person waves
- **Person re-identification** - Don't revisit same person
- **Multiple person queue** - Visit all detected people in order
- **Dynamic wait time** - Adjust based on person's behavior
- **Audio feedback** - Play sound when reaching person
- **LED indicators** - Show waiting state visually

---

**Status**: ✅ Updated! Robot now approaches to 1 foot and waits 10 seconds before looking for next person.

To apply changes, rebuild the package:
```bash
cd ~/rover_workspace
colcon build --packages-select rover_exploration --symlink-install
source install/setup.bash
```

Then restart the autonomous navigation node.

