# Three Bottle Pick and Place - User Manual

## Setup

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_up
./setup.sh
source venv/bin/activate
```

## Commands

### Detection
```bash
python scripts/pick_place.py detect           # Detect all bottles
python scripts/pick_place.py detect --no-display  # No visualization
```

### Pick Operations
```bash
python scripts/pick_place.py pick orange
python scripts/pick_place.py pick apple
python scripts/pick_place.py pick yogurt

# Debug mode (stop at specific step)
python scripts/pick_place.py pick orange --debug 1  # Stop after detection
python scripts/pick_place.py pick orange --debug 2  # Stop after approach
python scripts/pick_place.py pick orange --debug 3  # Stop after descend
python scripts/pick_place.py pick orange --debug 4  # Stop after grip
python scripts/pick_place.py pick orange --debug 5  # Stop after lift
```

### Place Operations
```bash
python scripts/pick_place.py place orange
python scripts/pick_place.py place apple
python scripts/pick_place.py place yogurt
```

### Move (Pick + Place)
```bash
python scripts/pick_place.py move orange apple
```

### Arm Positions
```bash
python scripts/pick_place.py pos home          # Center position
python scripts/pick_place.py pos retract       # Retracted position
python scripts/pick_place.py home              # Same as pos home
```

### System
```bash
python scripts/pick_place.py status            # Show system status
```

## Configuration

### Positions (`configs/robot_config.yaml`)
```yaml
positions:
  home:
    values: [2048, 2048, 2048, 2048]
    description: "Center position"
  
  retract:
    values: [2048, 1353, 3073, 937]
    description: "Retracted position"
```

To add new positions, just add to this section.

### Bottle Heights
```yaml
objects:
  orange_bottle:
    height: 0.185
  apple_bottle:
    height: 0.200
  yogurt_bottle:
    height: 0.155
```

### Movement Speed
```yaml
movement:
  profile_velocity: 300      # HIGHER = SLOWER with MORE TORQUE (range: 0-1023)
  profile_acceleration: 150  # HIGHER = GENTLER acceleration
```

**IMPORTANT**: For Dynamixel servos, HIGHER values = SLOWER movements with MORE TORQUE.

If you experience servo overload errors, INCREASE these values (not decrease!):
- `profile_velocity: 400` - Even slower, more torque
- `profile_acceleration: 200` - Even gentler

### AprilTag Settings
```yaml
apriltags:
  tag_size: 0.0254  # 1.0 inch in meters
```

### Position Calibration (X, Y, Z offsets)

If detected positions don't match actual measurements:

```yaml
coordinate_transform:
  x_offset: 0.0    # X-axis correction
  y_offset: 0.084  # Y-axis correction
  z_offset: 0.482  # Z-axis correction
```

**To calibrate Y-axis (distance from base):**
1. Measure actual distance from robot base center to AprilTag center (e.g., 20cm = 0.20m)
2. Run `python scripts/pick_place.py pick orange --debug 1`
3. Check "Tag Position (base frame - RAW)" Y value (e.g., Y=0.1158m)
4. Calculate offset: `y_offset = actual - detected` (e.g., 0.20 - 0.1158 = 0.084)
5. Set in config and re-run to verify

**To calibrate Z-axis (height):**
1. Place AprilTag flat on platform (actual Z=0)
2. Run debug mode and check detected Z (e.g., Z=-0.482m)
3. Set `z_offset = actual - detected` (e.g., 0 - (-0.482) = 0.482)
4. Re-run to verify Z is now close to 0

**X-axis calibration:** Similar process for left/right offset if needed

## Pick Strategy

The system uses a horizontal wrist approach:
- **Shoulder starts at 90°** to reach target
- **Elbow and wrist adjust** to maintain horizontal orientation
- **Wrist stays parallel to ground** (pointing in +Y direction)
- Shoulder moves only when needed to extend reach

This ensures stable picking with consistent end-effector orientation.

## Tag IDs

| Item | Tag ID |
|------|--------|
| Orange Bottle | 0 |
| Apple Bottle | 1 |
| Yogurt Bottle | 2 |
| Orange Drop Zone | 10 |
| Apple Drop Zone | 11 |
| Yogurt Drop Zone | 12 |

## Generate Tags

```bash
python scripts/generate_apriltag_pdf.py --all --size 1.0
```

Print at 100% scale. Verify size is exactly 1.0 inch (25.4mm).

## Calibration Tools

### Camera Position Calibration

Visualize camera optical center to measure position:

```bash
python tools/camera_optical_center.py
```

This shows a red crosshair at the camera's optical center. Place a marker under it, then measure distances from robot base to update camera position in config.

## Troubleshooting

**Bottles not detected**: Check camera connection, tag visibility, lighting

**Arm not moving**: Check USB connection, verify arm is powered on

**Poor accuracy**: Calibrate camera, verify tag sizes are correct

**Config not loading**: Check YAML syntax, restart script

