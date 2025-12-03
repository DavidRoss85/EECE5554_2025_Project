# Hand-Eye Calibration Workflow - Solving the Occlusion Problem

## The Problem

When the gripper moves into the image to touch the checkerboard, it **occludes** (blocks) the pattern, making detection impossible. This is a common issue in hand-eye calibration.

## The Solution

Use a **two-step workflow** that separates detection from gripper positioning:

1. **Step 1**: Detect checkerboard with gripper **OUT OF VIEW**
2. **Step 2**: Move gripper to touch/near checkerboard and capture joint positions

This way, the checkerboard is always detected when it's fully visible.

## Improved Calibration Tool

Use `hand_eye_calibration_improved.py` which implements this workflow:

```bash
python hand_eye_calibration_improved.py --camera 0 --calib camera_calibration.npz --square 24
```

## Workflow Steps

### 1. Setup
- Place checkerboard flat on platform
- Ensure good lighting
- Checkerboard should be visible to camera

### 2. For Each Observation:

#### Step A: Detect Checkerboard (Gripper Out of View)
1. **Move gripper away** from checkerboard (so it doesn't block the view)
2. Ensure checkerboard is **fully visible** in camera
3. Press **'D'** to **Detect and Store** checkerboard pose
   - Tool detects checkerboard
   - Stores the pose (rvec, tvec) for later use
   - Status shows "Checkerboard: STORED"

#### Step B: Capture Gripper Pose
1. **Move gripper** to touch (or near) the checkerboard corner
   - It's OK if checkerboard is now occluded - we already stored its pose!
2. Press **'C'** to **Capture** gripper pose
   - If arm controller is connected: automatically reads joint positions
   - If not: prompts you to enter joint positions manually
   - Associates stored checkerboard pose with current gripper pose
   - Creates one complete observation

### 3. Repeat
- Repeat steps A and B for **5-10 different positions**
- Vary:
  - Base rotation
  - Arm extension
  - Gripper position relative to checkerboard

### 4. Calibrate
- Press **'K'** to **Calibrate**
- Tool uses `cv2.calibrateHandEye()` with all observations
- Saves result to `hand_eye_calibration.json`

## Keyboard Controls

- **'D'**: Detect and store checkerboard pose (gripper should be out of view)
- **'C'**: Capture gripper pose (gripper can be touching checkerboard)
- **'K'**: Calibrate and save
- **'Q'**: Quit

## Visual Feedback

The tool shows:
- **Green**: Checkerboard detected and ready
- **Yellow**: Checkerboard pose stored, ready to capture gripper
- **Red**: Checkerboard not found

## Tips

1. **Start with gripper in home position** - keeps it out of view
2. **Move gripper to side** when detecting checkerboard
3. **Touch different corners** of checkerboard for variety
4. **Use varied arm positions** - don't just rotate base
5. **Ensure good lighting** throughout

## Why This Works

The key insight: **We don't need to detect checkerboard and read gripper position at the exact same moment.**

- Checkerboard pose is **relative to camera** - it doesn't change if we move the gripper
- We can detect it when fully visible
- Then move gripper and capture its position
- The association is valid because checkerboard is fixed in space

## Comparison with Old Method

### Old Method (Problematic):
1. Move gripper to touch checkerboard
2. Try to detect checkerboard (fails due to occlusion)
3. Can't capture observation

### New Method (Works):
1. Detect checkerboard (gripper out of view) → **SUCCESS**
2. Store checkerboard pose
3. Move gripper to touch checkerboard
4. Capture gripper pose → **SUCCESS**
5. Associate the two → **Complete observation**

## Troubleshooting

### "Checkerboard: NOT FOUND"
- Move gripper further away
- Check lighting
- Ensure checkerboard is flat
- Verify pattern size matches (6x7, 24mm squares)

### "No checkerboard pose stored"
- Press 'D' when checkerboard is detected (green status)
- Wait for "Checkerboard: STORED" message

### "Failed to read from arm controller"
- Check USB connection
- Verify arm is powered on
- Or enter joint positions manually when prompted

## Next Steps

After calibration:
1. Verify calibration by testing coordinate transformation
2. Use in pick and place system
3. Fine-tune if needed

