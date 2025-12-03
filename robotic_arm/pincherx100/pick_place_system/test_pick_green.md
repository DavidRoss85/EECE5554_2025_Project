# Test Pick Operation for Green Bottle

## Quick Test

Test the full pick operation with green bottle detection:

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_place_system
source venv/bin/activate

# Test with debug to see detection
python scripts/arm_cli.py pick orange --debug

# Or without debug
python scripts/arm_cli.py pick orange
```

## What Should Happen

1. **Detection**: System detects green cap (orange bottle)
2. **Coordinate Transformation**: 
   - Cap position (pixel) → Bottle base position (robot base frame)
   - Accounts for bottle height (18.5cm) for perspective
   - Sets Z to platform level (0.0)
3. **Arm Movement**:
   - Step 1: Move to approach position (5cm above grip height)
   - Step 2: Open gripper
   - Step 3: Descend to grip position (middle of bottle, ~9.25cm above platform)
   - Step 4: Close gripper (default position)
   - Step 5: Lift to 10cm above platform

## Expected Output

```
Detecting orange_bottle...
  Total detections: 1
    Detection 0: orange_bottle (confidence: 0.XX, color_match: True, center: [XXX, XXX])
  Found orange_bottle at pixel: [XXX, XXX]
  Confidence: 0.XX
  Color match: True
  Cap detected at pixel: (XXX.X, XXX.X)
  Bottle base position in base frame: (X.XXX, X.XXX, 0.000)
  Bottle height: 18.5cm

Pick Parameters:
  Object height: 18.5cm
  Grip height ratio: 0.5
  Grip height: 9.25cm above platform

Starting pick operation...
  Position: (X.XXX, X.XXX, 0.000)
  Grip at: 9.25cm above platform

============================================================
PICK OPERATION
============================================================
Object position (bottle base): [X.XXX, X.XXX, 0.000]
Grip height: 9.25cm above platform

Step 1: Moving to approach position: (X.XXX, X.XXX, 0.143)
...
```

## Troubleshooting

### If detection fails:
- Check camera feed with `--debug` flag
- Verify green cap is visible and detected
- Check lighting conditions

### If arm doesn't move:
- Check arm initialization
- Verify USB connection
- Check if position is within workspace limits

### If position seems wrong:
- Check coordinate transformation
- Verify camera calibration is loaded
- Check if hand-eye calibration is needed (currently using approximate transform)

## Next Steps

After successful movement test:
1. Calibrate gripper for each bottle type
2. Fine-tune approach/grip positions
3. Test with actual pick (with gripper calibration)
4. Add error recovery

