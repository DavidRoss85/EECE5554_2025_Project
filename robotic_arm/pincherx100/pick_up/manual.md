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

## Generate AprilTags

### Quick Start

Generate all tags for the three bottle system (1.0 inch size):

```bash
python scripts/generate_apriltag_pdf.py --all --size 1.0 --multi
```

This creates `apriltags_sheet.pdf` with all 6 tags (0, 1, 2, 10, 11, 12) on one sheet.

### Detailed Usage

**Generate all tags on one sheet:**
```bash
python scripts/generate_apriltag_pdf.py --all --size 1.0 --multi
```

**Generate specific tags:**
```bash
# Generate tags 0, 1, 2 on one sheet
python scripts/generate_apriltag_pdf.py --id 0 1 2 --size 1.0 --multi

# Generate individual PDFs for each tag
python scripts/generate_apriltag_pdf.py --id 0 1 2 --size 1.0
```

**Generate single tag:**
```bash
python scripts/generate_apriltag_pdf.py --id 0 --size 1.0
```

**Custom output filename:**
```bash
python scripts/generate_apriltag_pdf.py --all --size 1.0 --multi --output my_tags.pdf
```

**Adjust layout (tags per row):**
```bash
python scripts/generate_apriltag_pdf.py --all --size 1.0 --multi --tags-per-row 3
```

### Command Options

- `--id <id1> <id2> ...` - Generate specific tag IDs (default: 0 1 2)
- `--all` - Generate all tags for three bottle system (0, 1, 2, 10, 11, 12)
- `--size <inches>` - Tag size in inches (default: 1.0)
- `--multi` - Generate multiple tags on one sheet
- `--output <filename>` - Custom output filename
- `--tags-per-row <n>` - Number of tags per row for multi-tag sheet (default: 2)

### Printing Instructions

**CRITICAL: Print at actual size (100% scale)**

1. **Open PDF in PDF viewer** (Adobe Reader, Evince, etc.)
2. **Print settings:**
   - Set scaling to **100%** or **"Actual Size"**
   - **DO NOT** use "Fit to Page" or "Shrink to Fit"
   - **DO NOT** use any automatic scaling
3. **Verify size after printing:**
   - Measure the printed tag with a ruler
   - Should be exactly **1.0 inch (25.4 mm)** for default size
   - If incorrect, check printer settings and reprint
4. **Cut out tags:**
   - Cut carefully along the edges
   - Keep corners square
   - Avoid wrinkles or creases
5. **Attach to bottles:**
   - Place tag on top of bottle (flat surface)
   - Ensure tag is visible to camera
   - Keep tag flat and unwrinkled

### Tag ID Mapping

| Tag ID | Object |
|--------|--------|
| 0 | Orange Bottle |
| 1 | Apple Bottle |
| 2 | Yogurt Bottle |
| 10 | Orange Drop Zone |
| 11 | Apple Drop Zone |
| 12 | Yogurt Drop Zone |

### Update Configuration

After generating tags, ensure `robot_config.yaml` has the correct tag size:

```yaml
apriltags:
  tag_size: 0.0254  # 1.0 inch in meters (25.4 mm)
```

If you use a different size (e.g., 0.5 inch), update accordingly:
- 0.5 inch = 0.0127 meters
- 1.5 inch = 0.0381 meters

### Troubleshooting

**Tags not detected:**
- Verify tag size matches configuration (measure printed tag)
- Check tag is flat and not wrinkled
- Ensure good lighting (tags need contrast)
- Verify tag is in camera's field of view

**Wrong tag size after printing:**
- Check printer settings (disable all scaling)
- Some printers have "Scale to Fit" enabled by default
- Try printing from different PDF viewer
- Measure and adjust `--size` parameter if needed

**Tags look distorted:**
- Ensure printer is printing at 100% scale
- Check paper is loaded correctly
- Verify PDF viewer is not applying scaling

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

