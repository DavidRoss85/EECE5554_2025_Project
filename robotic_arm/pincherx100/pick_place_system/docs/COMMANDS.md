# Command Reference

Complete reference for all commands and operations in the Pick and Place System.

## Command-Line Interface

### Basic Usage

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_place_system
source venv/bin/activate
python scripts/arm_cli.py <command> [options]
```

### Available Commands

#### Pick Object

Pick up an orange juice bottle or yogurt bottle.

```bash
python scripts/arm_cli.py pick orange
python scripts/arm_cli.py pick yogurt

# With debug window (shows camera feed with detections)
python scripts/arm_cli.py pick orange --debug
python scripts/arm_cli.py pick yogurt --debug
```

**Process:**
1. Captures camera frame
2. Detects specified object type
3. Converts pixel coordinates to robot base coordinates
4. Moves arm to approach position
5. Descends to object
6. Closes gripper
7. Lifts object

**Output:**
- Detection status
- Object position (pixel and base coordinates)
- Movement progress
- Success/failure status

#### Drop Object

Drop object at detected drop location (checkerboard center).

```bash
python scripts/arm_cli.py drop

# With debug window (shows checkerboard detection)
python scripts/arm_cli.py drop --debug
```

**Process:**
1. Captures camera frame
2. Detects checkerboard pattern
3. Finds center of checkerboard
4. Converts to robot base coordinates
5. Moves arm to drop position
6. Opens gripper
7. Lifts arm

**Output:**
- Drop location detection status
- Position (pixel and base coordinates)
- Movement progress
- Success/failure status

#### Show Status

Display current system status.

```bash
python scripts/arm_cli.py status
```

**Output:**
- Arm initialization status
- Current joint positions
- Camera status
- Hand-eye calibration status

#### Move to Home

Move arm to home (center) position.

```bash
python scripts/arm_cli.py home
```

**Process:**
1. Initializes arm if not already initialized
2. Moves all joints to center positions (2048)
3. Waits for movement to complete

#### Test Detection

Test object detection without moving the arm. Shows live camera feed with detections.

```bash
python scripts/arm_cli.py test_detection
```

**Features:**
- Live camera feed with detection overlays
- Shows all detected bottles with labels
- Displays detection confidence and color match status
- Press 'Q' to quit
- Press 'S' to save current frame

**Use this to:**
- Verify objects are detected correctly
- Check detection confidence
- Debug detection issues
- Test different YOLO models

## Configuration Options

### Custom Config File

```bash
python scripts/arm_cli.py pick orange --config /path/to/config.yaml
```

### Debug Mode

Show camera feed with detections before executing command:

```bash
python scripts/arm_cli.py pick yogurt --debug
python scripts/arm_cli.py drop --debug
```

### YOLO Model Selection

Use different YOLO models for better accuracy:

```bash
# Use small model (better accuracy)
python scripts/arm_cli.py pick yogurt --model yolov8s.pt --debug

# Use medium model (best accuracy, slower)
python scripts/arm_cli.py pick yogurt --model yolov8m.pt --debug
```

Available models: `yolov8n.pt` (default, fastest), `yolov8s.pt`, `yolov8m.pt`, `yolov8l.pt`, `yolov8x.pt`

## Error Handling

### Object Not Detected

If object is not detected:
- System prints error message
- Operation stops
- No arm movement occurs

**Troubleshooting:**
- Ensure object is in camera view
- Check lighting conditions
- Verify object type matches command
- Check YOLO model is loaded correctly

### Drop Location Not Detected

If checkerboard is not detected:
- System prints error message
- Operation stops
- No arm movement occurs

**Troubleshooting:**
- Ensure checkerboard is in camera view
- Check checkerboard is flat and well-lit
- Verify pattern size matches (6x7)
- Check square size is correct (12mm)

### Arm Not Initialized

If arm is not initialized:
- System attempts to initialize
- If initialization fails, operation stops
- Error message displayed

**Troubleshooting:**
- Check USB connection
- Verify device permissions
- Check baudrate settings
- Ensure arm is powered on

## Examples

### Complete Pick and Place Sequence

```bash
# Pick orange juice bottle
python scripts/arm_cli.py pick orange

# Drop at location
python scripts/arm_cli.py drop

# Return to home
python scripts/arm_cli.py home
```

### Status Check Before Operation

```bash
# Check system status
python scripts/arm_cli.py status

# If arm not initialized, it will initialize on first command
python scripts/arm_cli.py pick yogurt
```

## Web Interface

### Start Web Interface

```bash
python ui/web_interface.py
```

Access at: `http://<raspberry_pi_ip>:5000`

**Features:**
- Live camera feed with detection overlays
- Real-time object detection data
- Drop location detection status
- Arm status and joint positions
- Coordinate information (pixel and base)

**Usage:**
- View camera feed and detections
- Monitor system status
- Debug detection issues
- Verify coordinate transformations

## Advanced Usage

### Testing Individual Components

#### Test Object Detection

```bash
python vision/object_detector.py
```

#### Test Checkerboard Detection

```bash
python vision/checkerboard_detector.py
```

#### Test Coordinate Transformation

```bash
python vision/coordinate_transform.py
```

#### Test Inverse Kinematics

```bash
python vision/inverse_kinematics.py
```

### Debug Mode

The CLI now includes built-in debug features:

```bash
# Test detection without moving arm
python scripts/arm_cli.py test_detection

# Pick with debug window
python scripts/arm_cli.py pick yogurt --debug

# Use Python debugger for code-level debugging
python -m pdb scripts/arm_cli.py pick orange
```

See `docs/DEBUGGING.md` for detailed debugging guide.

## Notes

- All coordinates are in meters
- Pixel coordinates are in image frame (0,0 at top-left)
- Base coordinates are in robot base frame (forward = +Y, right = +X, up = +Z)
- Gripper positions are in servo units (0-4095)
- Joint angles are in radians (converted internally)

