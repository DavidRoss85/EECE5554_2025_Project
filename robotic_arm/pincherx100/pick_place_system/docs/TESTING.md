# Testing Guide

Step-by-step testing procedures to verify system functionality.

## Testing Sequence

Follow these tests in order to verify each component before full operation.

## Test 1: Camera and Calibration

### Objective
Verify camera is working and calibration data is loaded correctly.

### Steps

1. **Test camera access:**
```bash
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAILED'); cap.release()"
```

2. **Verify calibration data exists:**
```bash
ls -lh ../camera_calibration/camera_calibration.npz
```

3. **Test coordinate transformer:**
```bash
python vision/coordinate_transform.py
```

**Expected Result:**
- Camera opens successfully
- Calibration file exists
- Coordinate transformer loads without errors

## Test 2: Object Detection

### Objective
Verify YOLO model can detect bottles in camera feed.

### Steps

1. **Place test objects:**
   - Orange juice bottle on white background
   - Yogurt bottle on white background
   - Ensure good lighting

2. **Run object detector:**
```bash
python vision/object_detector.py
```

3. **Verify detections:**
   - Orange bottle should be detected with green cap
   - Yogurt bottle should be detected with pink cap
   - Bounding boxes drawn correctly
   - Confidence scores displayed

**Expected Result:**
- Objects detected with confidence > 0.5
- Color filtering works (green/pink caps identified)
- Detection boxes align with objects

**Troubleshooting:**
- If objects not detected: adjust lighting, check YOLO model loaded
- If wrong type detected: verify color ranges in config
- If low confidence: improve lighting, check object visibility

## Test 3: Checkerboard Detection

### Objective
Verify checkerboard pattern can be detected for drop location.

### Steps

1. **Place checkerboard:**
   - Print 6x7 tiny US letter checkerboard
   - Mount on rigid surface
   - Place on platform in front of robot
   - Ensure flat and well-lit

2. **Run checkerboard detector:**
```bash
python vision/checkerboard_detector.py
```

3. **Verify detection:**
   - Checkerboard corners detected
   - Center point marked
   - Axis drawn correctly

**Expected Result:**
- Checkerboard detected reliably
- Center point calculated correctly
- Pattern visible in camera view

**Troubleshooting:**
- If not detected: check lighting, verify pattern size, ensure flat surface
- If corners inaccurate: check square size (12mm), verify print quality

## Test 4: Hand-Eye Calibration

### Objective
Complete hand-eye calibration to establish camera-to-robot transformation.

### Steps

1. **Follow calibration procedure** (see `CALIBRATION.md`)

2. **Verify calibration file created:**
```bash
ls -lh calibration/hand_eye_calibration.json
```

3. **Test coordinate transformation:**
   - Use web interface or test script
   - Verify pixel-to-base conversion is reasonable
   - Check that coordinates match expected values

**Expected Result:**
- Calibration file created with transformation matrix
- Coordinate transformation produces reasonable values
- Base coordinates align with physical positions

## Test 5: Inverse Kinematics

### Objective
Verify IK solver can calculate joint angles from positions.

### Steps

1. **Test IK solver:**
```bash
python vision/inverse_kinematics.py
```

2. **Verify output:**
   - Joint angles calculated
   - Servo positions within valid range (0-4095)
   - No errors or warnings

**Expected Result:**
- IK solver runs without errors
- Calculated positions are valid
- Joint angles are reasonable

## Test 6: Arm Control

### Objective
Verify arm can be controlled and moved to positions.

### Steps

1. **Test arm initialization:**
```bash
python scripts/arm_cli.py status
```

2. **Test home position:**
```bash
python scripts/arm_cli.py home
```

3. **Verify movement:**
   - Arm moves to center positions
   - No errors reported
   - Joint positions readable

**Expected Result:**
- Arm initializes successfully
- Moves to home position
- Joint positions can be read

**Troubleshooting:**
- If initialization fails: check USB connection, verify permissions
- If movement fails: check power supply, verify servo IDs
- If positions not readable: check baudrate, verify communication

## Test 7: Coordinate Mapping

### Objective
Verify pixel coordinates correctly map to robot positions.

### Steps

1. **Place object at known position:**
   - Use ruler to measure position relative to robot base
   - Place object on platform
   - Note physical coordinates

2. **Detect object:**
   - Use web interface or detection script
   - Record pixel coordinates
   - Record calculated base coordinates

3. **Compare:**
   - Compare calculated base coordinates with measured position
   - Verify they match within reasonable tolerance (few cm)

**Expected Result:**
- Calculated coordinates match measured positions
- Error is within acceptable range (< 5cm for prototype)

**Troubleshooting:**
- If large error: redo hand-eye calibration, check camera position in config
- If coordinates inverted: check coordinate frame definitions

## Test 8: Pick Operation (Dry Run)

### Objective
Test pick operation without actual object.

### Steps

1. **Place marker at test position:**
   - Use small marker or tape
   - Place at known position
   - Ensure visible to camera

2. **Run pick command:**
```bash
python scripts/arm_cli.py pick orange
```

3. **Observe:**
   - Object detected correctly
   - Arm moves to approach position
   - Arm descends to object
   - Gripper closes
   - Arm lifts

**Expected Result:**
- All movement steps execute
- No collisions or errors
- Arm returns to safe position

**Troubleshooting:**
- If IK fails: check target position is reachable
- If movement fails: check joint limits, verify workspace constraints
- If gripper fails: check gripper positions in config

## Test 9: Full Pick and Place

### Objective
Complete end-to-end pick and place operation.

### Steps

1. **Setup:**
   - Place object on platform
   - Place checkerboard for drop location
   - Ensure both visible to camera

2. **Pick object:**
```bash
python scripts/arm_cli.py pick orange
```

3. **Drop object:**
```bash
python scripts/arm_cli.py drop
```

4. **Verify:**
   - Object picked successfully
   - Object placed at drop location
   - No errors occurred

**Expected Result:**
- Complete operation succeeds
- Object moved from pick to drop location
- System ready for repeated operations

## Test 10: Web Interface

### Objective
Verify web interface displays all information correctly.

### Steps

1. **Start web interface:**
```bash
python ui/web_interface.py
```

2. **Access in browser:**
   - Open `http://<raspberry_pi_ip>:5000`
   - Verify page loads

3. **Check features:**
   - Camera feed visible
   - Object detections displayed
   - Drop location shown when detected
   - Arm status updates
   - Coordinates displayed correctly

**Expected Result:**
- All features work correctly
- Data updates in real-time
- No errors in browser console

## Performance Testing

### Detection Accuracy

Test detection accuracy with various conditions:
- Different lighting conditions
- Different object orientations
- Multiple objects
- Cluttered backgrounds

### Repeatability

Test repeatability of pick and place:
- Run same operation multiple times
- Measure position accuracy
- Check for consistent results

### Speed

Measure operation times:
- Detection time
- Movement time
- Total operation time

## Troubleshooting Common Issues

### Objects Not Detected

- Check lighting
- Verify object is in view
- Check YOLO model confidence threshold
- Verify color filtering parameters

### Poor Position Accuracy

- Recalibrate hand-eye transformation
- Verify camera position in config
- Check coordinate transformation
- Verify IK solver accuracy

### Arm Movement Errors

- Check joint limits
- Verify workspace constraints
- Check for collisions
- Verify servo communication

### Web Interface Not Loading

- Check Flask is running
- Verify port 5000 is accessible
- Check firewall settings
- Verify camera is connected

## Next Steps

After successful testing:
1. Fine-tune parameters in config
2. Adjust gripper positions for objects
3. Optimize movement speeds
4. Add error recovery procedures

