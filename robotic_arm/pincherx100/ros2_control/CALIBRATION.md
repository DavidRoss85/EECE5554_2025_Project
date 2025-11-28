# PincherX100 Calibration Guide

Detailed calibration procedures for the PincherX100 arm.

## Why Calibrate?

Calibration is essential to:
1. **Verify servo connectivity** - Ensure all servos are responding
2. **Set safe joint limits** - Prevent mechanical damage from overextension
3. **Define home position** - Establish a consistent reference position
4. **Configure gripper** - Set proper open/close positions
5. **Test range of motion** - Verify each joint moves correctly

## When to Calibrate

- **First time setup** - Before using the arm for the first time
- **After hardware changes** - When replacing or reconfiguring servos
- **Unusual behavior** - If joints move unexpectedly or hit limits
- **New workspace** - When operating in a different environment

---

## Calibration Tool

The package includes an interactive calibration tool.

### Starting the Calibration Tool

```bash
# Enter Docker container
docker exec -it pincherx100_ros2 bash

# Run calibration tool
ros2 run pincherx100_control calibrate
```

### Calibration Menu

The tool provides these options:

1. **Test servo range of motion** - Move each servo through test positions
2. **Find joint limits** - Interactively determine safe min/max positions
3. **Set home position** - Define the default home position
4. **Set gripper positions** - Configure open and closed positions
5. **Generate configuration file** - Save calibration to YAML
6. **Exit** - Exit the calibration tool

---

## Step-by-Step Calibration

### Step 1: Scan for Servos

When you start the calibration tool, it automatically scans for servos:

```
==========================================================
Scanning Servos
==========================================================
  ✓ Servo ID 1: Found (Position: 2048)
  ✓ Servo ID 2: Found (Position: 2156)
  ✓ Servo ID 3: Found (Position: 1987)
  ✓ Servo ID 4: Found (Position: 2048)
  ✓ Servo ID 5: Found (Position: 2300)
```

If any servos are not found:
- Check power supply
- Verify USB connection
- Check servo IDs match configuration
- Try different baudrate

### Step 2: Test Range of Motion

Select option 1 from the menu to test each joint:

```
Testing base (ID 1)
Press Enter to continue, or 's' to skip...
```

For each joint, the test will:
1. Move to center position (2048)
2. Move counter-clockwise (~1024)
3. Move clockwise (~3072)
4. Return to starting position

**Watch carefully** for:
- Unusual sounds (grinding, clicking)
- Hitting mechanical limits
- Unexpected movement directions

### Step 3: Find Joint Limits

Select option 2 to interactively find safe limits for each joint.

#### For Each Joint:

1. **Find Minimum Position**
   ```
   Finding limits for base (ID 1)
   Move the joint to its MINIMUM safe position
   Current position: 2048
   
   Enter target position (0-4095) or 'done':
   > 1500
   Moved to 1500
   > 1000
   Moved to 1000
   > done
   Minimum position set: 1000
   ```

   - Move the joint gradually toward its minimum
   - Stop before hitting mechanical limits
   - Leave some safety margin (50-100 units)

2. **Find Maximum Position**
   ```
   Now move to MAXIMUM safe position
   Enter target position (0-4095) or 'done':
   > 3000
   Moved to 3000
   > 3500
   Moved to 3500
   > done
   Maximum position set: 3500
   ```

   - Move the joint gradually toward its maximum
   - Stop before hitting mechanical limits
   - Leave some safety margin

#### Recommended Limits

Based on typical PincherX100 mechanical design:

| Joint    | Min (units) | Max (units) | Min (degrees) | Max (degrees) | Notes |
|----------|-------------|-------------|---------------|---------------|-------|
| Base     | 512         | 3584        | 45°           | 315°          | Avoid cable twist |
| Shoulder | 1024        | 3072        | 90°           | 270°          | Prevent table collision |
| Elbow    | 512         | 3584        | 45°           | 315°          | Full range usually safe |
| Wrist    | 1024        | 3072        | 90°           | 270°          | Prevent gripper collision |
| Gripper  | 1500        | 2600        | 132°          | 228°          | Depends on gripper design |

 **Important**: These are examples. Your limits may differ based on:
- Mounting configuration
- Workspace constraints
- Gripper type
- Cable routing

### Step 4: Set Home Position

Select option 3 to define the home position.

```
==========================================================
Finding Home Position
==========================================================
Position each joint at its desired HOME position

base (ID 1)
Current: 2048
Enter home position (or press Enter for current):
> 2048
Using current position: 2048

shoulder (ID 2)
Current: 2048
Enter home position (or press Enter for current):
> 2048
...
```

**Recommended home position**:
- All joints at center (2048) is safest
- Alternative: Configure a "ready to work" position
- Ensure home position is:
  - Stable (won't tip over)
  - Clear of obstacles
  - Within all joint limits

### Step 5: Configure Gripper

Select option 4 to set gripper positions.

#### Find Open Position

```
Gripper calibration
Move gripper to OPEN position
Enter position or 'done':
> 2400
(Gripper moves)
> 2450
(Gripper moves more)
> done
Open position: 2450
```

#### Find Closed Position

```
Move gripper to CLOSED position
Enter position or 'done':
> 1800
(Gripper moves)
> 1700
(Gripper closes more)
> 1650
(Gripper fully closed but not straining)
> done
Closed position: 1650
```

**Tips for gripper calibration**:
- Open position: Fingers fully extended
- Closed position: Fingers just touching (or desired grip)
- Don't over-tighten (avoid stalling motor)
- Test with objects of different sizes

### Step 6: Generate Configuration

Select option 5 to generate the configuration file.

```
Generated configuration:
# PincherX100 Configuration

limits:
  base:
    min: 512
    max: 3584
  shoulder:
    min: 1024
    max: 3072
  ...

poses:
  home: [2048, 2048, 2048, 2048, 2048]
  sleep: [2048, 1024, 512, 2048, 1650]
  ready: [2048, 2500, 2200, 2048, 2450]
  gripper_open: 2450
  gripper_closed: 1650

Save to file? (y/n):
> y
Saved to config/arm_config.yaml
```

The configuration is saved to:
```
/ros2_ws/src/pincherx100_control/config/arm_config.yaml
```

Which is mounted to (on host):
```
/home/jx/Dev/Robotics/pincherx100/ros2_control/pincherx100_control/config/arm_config.yaml
```

---

## Manual Configuration

You can also edit the configuration file manually:

```yaml
# config/arm_config.yaml

limits:
  base:
    min: 512
    max: 3584
  shoulder:
    min: 1024
    max: 3072
  elbow:
    min: 512
    max: 3584
  wrist:
    min: 1024
    max: 3072
  gripper:
    min: 1500
    max: 2600

poses:
  home: [2048, 2048, 2048, 2048, 2048]
  sleep: [2048, 1024, 512, 2048, 2048]
  ready: [2048, 2500, 2200, 2048, 2048]
  gripper_open: 2448
  gripper_closed: 1648
```

After editing, restart the controller to apply changes.

---

## Verification After Calibration

### 1. Test Joint Limits

```bash
# Start controller
ros2 launch pincherx100_control arm_control.launch.py

# In another terminal, try moving to limits
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.5, 0.5, 0.5, 0.5]"

# Should move to near minimum limits without hitting them
```

### 2. Test Preset Poses

```bash
# Test home
ros2 service call /go_home std_srvs/srv/Trigger

# Test sleep
ros2 service call /go_sleep std_srvs/srv/Trigger

# Test ready
ros2 service call /go_ready std_srvs/srv/Trigger
```

### 3. Test Gripper

```bash
# Open
ros2 service call /gripper_open std_srvs/srv/Trigger

# Close
ros2 service call /gripper_close std_srvs/srv/Trigger
```

### 4. Run Joint Test

```bash
ros2 run pincherx100_control test_joints
```

This will move each joint individually. Verify:
- Movements are smooth
- No hitting limits
- Directions are correct

---

## Troubleshooting Calibration

### Servo positions jump unexpectedly

**Cause**: Multi-turn mode or incorrect operating mode

**Solution**:
```bash
# Check operating mode (should be 3 for position control)
# Use Dynamixel Wizard or low-level tools to verify
```

### Cannot reach desired position

**Cause**: Limits configured too restrictively

**Solution**:
1. Re-run calibration
2. Expand limits slightly
3. Or adjust target positions

### Gripper doesn't grip objects

**Cause**: Closed position not tight enough

**Solution**:
1. Re-calibrate gripper
2. Reduce closed position value more
3. But don't exceed current limits (watch for stalling)

### Joint moves in wrong direction

**Cause**: Servo might be inverted or mounted differently

**Solution**:
1. Note which joints are inverted
2. You may need to modify the control code to invert commands
3. Or physically remount the servo

### Calibration tool can't find servos

**Cause**: Connection, power, or baudrate mismatch

**Solution**:
```bash
# Try different baudrates (common: 57600, 1000000)
# Check with low-level tool first
cd /home/jx/Dev/Robotics/pincherx100/low-level_control
source venv/bin/activate
python scan_servos.py
```

---

## Advanced Calibration

### Custom Poses

Add custom poses to the configuration:

```yaml
poses:
  home: [2048, 2048, 2048, 2048, 2048]
  sleep: [2048, 1024, 512, 2048, 2048]
  ready: [2048, 2500, 2200, 2048, 2048]
  # Custom poses:
  inspect: [1500, 2800, 2400, 2048, 2448]  # Looking down
  side_reach: [3200, 2400, 2000, 2048, 2448]  # Reaching to side
```

To use custom poses, you'll need to add services in the code or use position commands.

### Velocity and Acceleration Tuning

Adjust movement characteristics:

```yaml
profile_velocity: 150      # Lower = faster, Higher = slower
profile_acceleration: 80   # Lower = aggressive, Higher = smooth
```

- **Fast response**: velocity=50, acceleration=20
- **Balanced**: velocity=100, acceleration=50
- **Max torque**: velocity=150, acceleration=80
- **Very smooth**: velocity=200, acceleration=150

### Power Limits

Adjust power and torque:

```yaml
pwm_limit: 885            # Max: 885 (100% voltage)
current_limit: 1193       # Max: 1193 (2.69A for XL430)
```

 **Warning**: Do not exceed these maximum values:
- PWM_LIMIT: 885
- CURRENT_LIMIT: 1193 (for XL430-W250)

---

## Recalibration Schedule

Consider recalibrating:
- **Monthly** if used frequently
- **After any collision or unusual event**
- **If behavior changes** (drift, unexpected limits)
- **After maintenance** (servo replacement, mechanical adjustment)

---

## Backup Configuration

Always keep a backup of your working configuration:

```bash
# On host
cp pincherx100_control/config/arm_config.yaml \
   pincherx100_control/config/arm_config.yaml.backup

# Or with timestamp
cp pincherx100_control/config/arm_config.yaml \
   pincherx100_control/config/arm_config_$(date +%Y%m%d_%H%M%S).yaml
```

---

## Summary Checklist

- [ ] Scan and verify all servos found
- [ ] Test range of motion for each joint
- [ ] Set safe joint limits with margin
- [ ] Define home position
- [ ] Configure gripper open/close positions
- [ ] Generate and save configuration
- [ ] Restart controller with new config
- [ ] Verify limits and poses work correctly
- [ ] Backup configuration file
- [ ] Document any special notes or observations

---

For additional help, refer to:
- [SETUP_GUIDE.md](./SETUP_GUIDE.md) - Setup instructions
- [USAGE.md](./USAGE.md) - Usage examples
- [README.md](./README.md) - Overview and quick start




