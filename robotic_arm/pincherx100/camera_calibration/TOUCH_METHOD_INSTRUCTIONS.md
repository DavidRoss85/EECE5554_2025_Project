# Hand-Eye Calibration - Touch Method (Approach 2)

Complete instructions for performing hand-eye calibration using the touch method, where the gripper center (TCP) touches specific corners of the checkerboard.

## Overview

In this method:
- **Gripper center (TCP)** touches specific **checkerboard corners**
- The TCP is the center point between the two gripper fingers
- Forward kinematics calculates the TCP position from joint angles
- The relationship between TCP and checkerboard corner is used for calibration

## Prerequisites

1. Camera calibration completed (`camera_calibration.npz` exists)
2. Checkerboard printed (6x7 large, 24mm squares)
3. Checkerboard mounted on rigid surface
4. Arm controller accessible (optional - can enter positions manually)

## Setup

### 1. Print and Mount Checkerboard

- Print `checkerboard_6x7_large_US_letter.svg` at 100% scale
- Measure one square - should be 24mm (2.4cm)
- Mount on rigid flat surface (foam board recommended)
- Place on platform in front of robot
- Keep checkerboard in **fixed position** throughout calibration

### 2. Start Calibration Tool

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate
python hand_eye_calibration_touch_method.py --camera 0 --calib camera_calibration.npz --square 24
```

## Step-by-Step Workflow

### For Each Observation (Repeat 5-10 times):

#### Step 1: Detect Checkerboard (Gripper Out of View)

1. **Move gripper away** from checkerboard (so it doesn't block the view)
2. Ensure checkerboard is **fully visible** in camera
3. Press **'D'** to **Detect and Store** checkerboard pose
   - Tool detects checkerboard pattern
   - Stores the pose for later use
   - Status shows "Checkerboard stored"

#### Step 2: Select Corner to Touch

1. Press **'S'** to **Select Corner**
2. Enter the **corner index** when prompted:
   - Corner 0: Top-left corner
   - Corner 41: Bottom-right corner (for 6x7 pattern: 6×7 = 42 corners, indices 0-41)
   - Pattern layout: 6 columns × 7 rows
3. The selected corner will be highlighted in yellow on the display

**Corner Numbering:**
```
Corner 0    Corner 5
  (0,0)       (5,0)
    ▓▓▓▓▓▓▓▓▓▓▓▓
    ▓▓▓▓▓▓▓▓▓▓▓▓
    ▓▓▓▓▓▓▓▓▓▓▓▓
    ▓▓▓▓▓▓▓▓▓▓▓▓
    ▓▓▓▓▓▓▓▓▓▓▓▓
    ▓▓▓▓▓▓▓▓▓▓▓▓
Corner 36   Corner 41
  (0,6)       (5,6)
```

**Recommended corners to use:**
- Corner 0 (top-left)
- Corner 5 (top-right)
- Corner 36 (bottom-left)
- Corner 41 (bottom-right)
- A few corners in the middle for variety

#### Step 3: Move Gripper to Touch Corner

1. **Move gripper center (TCP)** to touch the selected corner
   - The TCP is the center point between the two gripper fingers
   - Align the gripper center with the checkerboard corner
   - You can close the gripper slightly to make the center point more visible
   
2. **Visual guidance:**
   - The selected corner is highlighted in yellow on the camera display
   - Move gripper until its center aligns with the highlighted corner
   - It's OK if checkerboard is now partially occluded - we already stored its pose

3. **Tips for accurate touching:**
   - Approach from above (gripper horizontal, pointing down)
   - Use the gripper center as reference point
   - Touch gently - don't press hard
   - Ensure gripper center is directly over the corner

#### Step 4: Capture Gripper Pose

1. Once gripper center is touching the corner, press **'C'** to **Capture**
2. If arm controller is connected:
   - Joint positions are read automatically
   - Gripper pose is calculated using forward kinematics
3. If arm controller not available:
   - Enter joint positions manually when prompted
   - Format: `base,shoulder,elbow,wrist`
   - Example: `2048,2048,2048,2048`

4. Observation is saved with:
   - Checkerboard pose (from Step 1)
   - Gripper TCP pose (from Step 3)
   - Corner index (from Step 2)

#### Step 5: Repeat

- Move gripper to a different position
- Select a different corner
- Repeat Steps 1-4
- Collect **5-10 observations** total

### Final Step: Calibrate

1. After collecting 5-10 observations, press **'K'** to **Calibrate**
2. Tool performs hand-eye calibration using `cv2.calibrateHandEye()`
3. Result is saved to `hand_eye_calibration.json`

## Keyboard Controls

- **'D'**: Detect and store checkerboard pose (gripper should be out of view)
- **'S'**: Select corner to touch (then enter corner number)
- **'C'**: Capture gripper pose (when gripper is touching corner)
- **'K'**: Calibrate and save
- **'Q'**: Quit

## Visual Feedback

- **Green status**: Checkerboard detected and ready
- **Yellow highlight**: Selected corner (touch this one)
- **Status messages**: Show current step and what to do next

## Understanding the Gripper Center (TCP)

### What is TCP?

- **TCP = Tool Center Point**
- For PincherX100, this is the **center point between the two gripper fingers**
- Located at the midpoint of the gripper opening
- The forward kinematics calculates this position from joint angles

### How to Identify Gripper Center

1. **With gripper open:**
   - Center is the midpoint between the two fingers
   - Approximately at the center of the gripper opening

2. **With gripper closed:**
   - Center is where the two fingers meet
   - Still the same point, just easier to see

3. **Visual reference:**
   - Look at the gripper from above
   - Find the center point between the fingers
   - This is what should touch the checkerboard corner

### TCP Position Calculation

The forward kinematics calculates TCP position as:
- Wrist position + L4 (109mm) along the approach vector
- L4 is the distance from wrist joint to TCP
- This is automatically calculated by the tool

## Tips for Accurate Calibration

### 1. Corner Selection
- Use corners at different positions (corners, edges, center)
- Vary which corners you touch
- Don't always use the same corner

### 2. Gripper Positioning
- Approach from different angles
- Vary base rotation
- Vary arm extension
- Vary height

### 3. Touch Accuracy
- Take time to align gripper center with corner
- Touch gently - don't press hard
- Ensure good alignment before capturing

### 4. Checkerboard
- Keep checkerboard in **same position** throughout
- Ensure it's flat and rigid
- Good lighting for detection

## Troubleshooting

### "Checkerboard: NOT FOUND"
- Move gripper further away
- Check lighting
- Ensure checkerboard is flat
- Verify pattern size matches (6x7, 24mm)

### "No checkerboard pose stored"
- Press 'D' when checkerboard is detected (green status)
- Wait for "Checkerboard stored" message

### "No corner selected"
- Press 'S' after storing checkerboard pose
- Enter valid corner number (0 to 41 for 6x7 pattern)

### "Failed to read from arm controller"
- Check USB connection
- Verify arm is powered on
- Or enter joint positions manually

### Poor Touch Accuracy
- Take more time to align gripper center with corner
- Use visual feedback (yellow highlight)
- Approach from above for better visibility
- Close gripper slightly to see center point better

## Expected Results

After calibration, you should have:
- `hand_eye_calibration.json` - Calibration result with transformation matrix
- `hand_eye_calibration_observations.json` - All observations for reference

The transformation matrix converts coordinates from camera frame to robot base frame.

## Next Steps

1. **Verify calibration:**
   - Test coordinate transformation
   - Check that pixel coordinates map correctly to robot positions

2. **Use in pick and place system:**
   - Load calibration in `coordinate_transform.py`
   - Use for object detection and positioning

3. **Fine-tune if needed:**
   - If accuracy is poor, collect more observations
   - Use more varied positions
   - Ensure accurate corner touching

## Comparison with Approach 1

| Aspect | Approach 1 (Anywhere) | Approach 2 (Touch) |
|--------|----------------------|-------------------|
| **Complexity** | Simpler | More precise |
| **Gripper position** | Anywhere in workspace | Must touch specific corner |
| **Accuracy** | Good | Potentially better |
| **Ease of use** | Easier | Requires careful alignment |
| **Recommended for** | First calibration | When precision is critical |

Both methods work well. Approach 2 (touch method) can provide slightly better accuracy when done carefully.

