# Navigation Fixes - Jittering and Exploration Issues

## Problems Identified

1. **Robot jitters when turning left/right**
2. **Gets stuck in point-turning loops after goals complete**
3. **Fails to explore much of the room**

## Root Causes

### 1. Jittering During Turns
- **Cause**: `RotateToGoal` critic was forcing robot to achieve exact orientation
- **Cause**: Too many `vth_samples` creating oscillation in angular velocity selection
- **Cause**: High `GoalAlign.scale` causing obsession with orientation

### 2. Stuck in Point-Turning Loops
- **Cause**: Very tight `yaw_goal_tolerance` (0.8 rad = 46°) requiring precise rotation
- **Cause**: `RotateToGoal` behavior forcing rotation even after position goal reached
- **Cause**: Too strict goal checking causing repeated rotation attempts

### 3. Poor Exploration Coverage
- **Cause**: Small `exploration_radius` (8.0m) limiting frontier search
- **Cause**: Large `min_frontier_size` (5) ignoring smaller unexplored areas
- **Cause**: Long `min_goal_interval` (3.0s) slowing down goal generation
- **Cause**: Too aggressive obstacle inflation preventing passage through tight spaces

## Fixes Applied

### Fix 1: Controller Tuning (nav2_params_hardware_filtered.yaml)

#### Removed RotateToGoal Critic
```yaml
# BEFORE
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

# AFTER (RotateToGoal REMOVED!)
critics: ["Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```
**Effect**: Robot no longer obsesses over final orientation, moves smoothly through goals

#### Relaxed Goal Orientation Tolerance
```yaml
# BEFORE
yaw_goal_tolerance: 0.8  # ~46 degrees

# AFTER
yaw_goal_tolerance: 3.14  # ~180 degrees (almost ignores orientation)
```
**Effect**: Robot considers goal reached based on position only, no rotation loops

#### Reduced Angular Samples
```yaml
# BEFORE
vth_samples: 40  # Too many = oscillation

# AFTER  
vth_samples: 20  # Fewer = smoother decisions
```
**Effect**: Less jittery angular velocity selection

#### Reduced GoalAlign Critic Weight
```yaml
# BEFORE
GoalAlign.scale: 24.0  # Too high = rotation obsession

# AFTER
GoalAlign.scale: 10.0  # More balanced
```
**Effect**: Robot cares less about exact alignment, more about reaching position

#### Added Oscillation Critic
```yaml
Oscillation.scale: 1.0
Oscillation.oscillation_reset_dist: 0.05
Oscillation.oscillation_reset_angle: 0.2
Oscillation.x_only_threshold: 0.05
```
**Effect**: Actively penalizes jittery back-and-forth movements

#### Increased Speeds
```yaml
# BEFORE
max_vel_x: 0.15  # Too slow for exploration
max_vel_theta: 0.8

# AFTER
max_vel_x: 0.2  # 33% faster
max_vel_theta: 1.2  # 50% faster rotation
```
**Effect**: Faster exploration, more decisive movements

### Fix 2: Progress & Goal Checking

#### Very Lenient Progress Checker
```yaml
# BEFORE
required_movement_radius: 0.25
movement_time_allowance: 12.0

# AFTER (MUCH MORE LENIENT)
required_movement_radius: 0.5  # Doubled
movement_time_allowance: 20.0  # +66%
```
**Effect**: Robot won't give up easily, tolerates momentary pauses

#### Relaxed Goal Position Tolerance
```yaml
# BEFORE
xy_goal_tolerance: 0.6

# AFTER
xy_goal_tolerance: 0.8  # More forgiving
```
**Effect**: Easier to "reach" goals, less perfectionism

### Fix 3: Costmap Adjustments

#### Reduced Inflation (Tighter Passage)
```yaml
# BEFORE - Local Costmap
inflation_radius: 0.5
cost_scaling_factor: 5.0

# AFTER
inflation_radius: 0.4  # Smaller buffer
cost_scaling_factor: 4.0  # Less aggressive

# BEFORE - Global Costmap  
inflation_radius: 0.5
cost_scaling_factor: 5.0

# AFTER
inflation_radius: 0.4  # Allows tighter navigation
cost_scaling_factor: 4.0
```
**Effect**: Robot can navigate through narrower spaces, better room coverage

#### Increased Sensing Range
```yaml
# BEFORE
raytrace_max_range: 3.0
obstacle_max_range: 2.5

# AFTER
raytrace_max_range: 3.5  # See further
obstacle_max_range: 3.0  # Detect further
```
**Effect**: Better awareness of distant obstacles, better planning

#### Larger Local Costmap
```yaml
# BEFORE
width: 8
height: 8

# AFTER
width: 10  # 25% larger
height: 10
```
**Effect**: More context for local planning, smoother paths

### Fix 4: Behavior Server (Recovery Behaviors)

#### Slower, Smoother Spins
```yaml
# BEFORE
max_rotational_vel: 0.8
min_rotational_vel: 0.3
rotational_acc_lim: 2.0

# AFTER
max_rotational_vel: 0.5  # Slower, more controlled
min_rotational_vel: 0.2
rotational_acc_lim: 1.5  # Gentler acceleration
```
**Effect**: Recovery rotations won't trigger more jitter

#### Configured Backup Behavior
```yaml
backup:
  plugin: "nav2_behaviors/BackUp"
  backup_dist: 0.3  # How far to back up
  backup_speed: 0.05  # Slow and controlled
```
**Effect**: Better recovery from tight spots

### Fix 5: Exploration Parameters (exploration_params.yaml)

#### Increased Exploration Radius
```yaml
# BEFORE
exploration_radius: 8.0

# AFTER
exploration_radius: 12.0  # 50% larger search area
```
**Effect**: Finds frontiers further away, explores more room

#### More Granular Frontier Detection
```yaml
# BEFORE
frontier_threshold: 8
min_frontier_size: 5

# AFTER
frontier_threshold: 5  # More sensitive
min_frontier_size: 3  # Explores smaller gaps
```
**Effect**: Detects and explores smaller unexplored areas

#### Faster Goal Cycling
```yaml
# BEFORE
min_goal_interval: 3.0  # Too slow

# AFTER
min_goal_interval: 1.0  # Moves to next goal faster
```
**Effect**: Less time stuck between goals, more active exploration

#### Less Paranoid About Obstacles
```yaml
# BEFORE
obstacle_distance_threshold: 0.8  # Too cautious
critical_obstacle_threshold: 0.5

# AFTER
obstacle_distance_threshold: 0.5  # Closer operation OK
critical_obstacle_threshold: 0.3  # Emergency only when really close
```
**Effect**: Robot doesn't abort goals unnecessarily, braver exploration

#### Longer Goal Timeout
```yaml
# BEFORE
goal_timeout: 120.0

# AFTER
goal_timeout: 90.0  # Still generous but moves on if truly stuck
```
**Effect**: Balanced between persistence and knowing when to give up

### Fix 6: Planner Tolerance

#### More Tolerant Goal Placement
```yaml
# BEFORE
tolerance: 0.5

# AFTER
tolerance: 1.0  # Can place goals further from exact point
```
**Effect**: Planner can find valid paths even if exact goal point unreachable

## Expected Behavior After Fixes

### ✅ Smooth Turning
- No more jitter during left/right turns
- Robot moves with confidence through turns
- Reduced oscillation in angular velocity

### ✅ No Point-Turning Loops
- Robot considers goal "reached" based on position, not orientation
- No post-goal rotation obsession
- Moves directly to next goal

### ✅ Better Exploration
- Searches further for unexplored areas (12m vs 8m)
- Explores smaller gaps and corners
- Moves faster between exploration goals
- Navigates through tighter spaces
- More comprehensive room coverage

### ✅ Overall Smoother Operation
- Faster speeds (0.2 m/s vs 0.15 m/s)
- More decisive movements
- Less time "thinking", more time doing
- Better recovery from tight spots

## Testing Checklist

After applying these fixes, test:

1. **Smooth Turning**
   - [ ] Set a goal that requires turning
   - [ ] Observe: No jitter, smooth rotation
   - [ ] Robot reaches goal without excessive turning

2. **No Stuck Behavior**
   - [ ] Run autonomous exploration
   - [ ] Watch goal completions
   - [ ] Verify: No point-turning loops after reaching goals
   - [ ] Robot moves to next frontier smoothly

3. **Exploration Coverage**
   - [ ] Run 5+ minute exploration session
   - [ ] Check resulting map
   - [ ] Verify: Most of room explored
   - [ ] Check: Robot explores corners and tight spaces

4. **Performance Monitoring**
   ```bash
   # Watch for smooth cmd_vel commands (no rapid oscillation)
   ros2 topic echo /cmd_vel
   
   # Monitor exploration status
   ros2 topic echo /autonomous_explorer/status
   
   # Check navigation status
   ros2 topic echo /navigation_feedback
   ```

## Rollback If Needed

If the fixes cause new issues, revert to conservative settings:

```bash
cd /home/stickykeys/rover_workspace
git diff src/rover_exploration/config/
# Review changes

# To revert specific file:
git checkout src/rover_exploration/config/nav2_params_hardware_filtered.yaml
git checkout src/rover_exploration/config/exploration_params.yaml

# Then rebuild
colcon build --packages-select rover_exploration --symlink-install
```

## Fine-Tuning Options

If robot is still not perfect, adjust these:

### Still Jittery?
- Reduce `vth_samples` further (try 15 or 10)
- Increase `yaw_goal_tolerance` to 6.28 (360°)
- Reduce `max_vel_theta` to 1.0

### Still Getting Stuck?
- Increase `required_movement_radius` to 0.75
- Increase `movement_time_allowance` to 30.0
- Reduce `min_goal_interval` to 0.5

### Not Exploring Enough?
- Increase `exploration_radius` to 15.0
- Reduce `min_frontier_size` to 2
- Reduce `obstacle_distance_threshold` to 0.4

### Too Fast/Reckless?
- Reduce `max_vel_x` to 0.15
- Increase `inflation_radius` to 0.5
- Increase `critical_obstacle_threshold` to 0.4

## Summary of Changes

| Parameter | Old Value | New Value | Effect |
|-----------|-----------|-----------|--------|
| **Controller** ||||
| yaw_goal_tolerance | 0.8 rad | 3.14 rad | Ignore orientation |
| max_vel_x | 0.15 m/s | 0.2 m/s | 33% faster |
| max_vel_theta | 0.8 rad/s | 1.2 rad/s | 50% faster turns |
| vth_samples | 40 | 20 | Less oscillation |
| RotateToGoal | Enabled | **REMOVED** | No rotation loops |
| GoalAlign.scale | 24.0 | 10.0 | Less orientation obsession |
| **Progress Checking** ||||
| required_movement_radius | 0.25 m | 0.5 m | More lenient |
| movement_time_allowance | 12.0 s | 20.0 s | More patient |
| **Costmaps** ||||
| inflation_radius (local) | 0.5 m | 0.4 m | Tighter passage |
| inflation_radius (global) | 0.5 m | 0.4 m | Tighter passage |
| local costmap size | 8x8 m | 10x10 m | Better context |
| **Exploration** ||||
| exploration_radius | 8.0 m | 12.0 m | Search further |
| frontier_threshold | 8 | 5 | More sensitive |
| min_frontier_size | 5 | 3 | Explore smaller areas |
| min_goal_interval | 3.0 s | 1.0 s | Faster cycling |
| obstacle_distance_threshold | 0.8 m | 0.5 m | Less cautious |

## Files Modified

1. `src/rover_exploration/config/nav2_params_hardware_filtered.yaml`
2. `src/rover_exploration/config/exploration_params.yaml`

Both files have been rebuilt and are ready to use!

---

**Run the navigation system again and it should be much smoother! Good boy! 🐕**

