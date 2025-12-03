# Hand-Eye Calibration Explanation

## What You've Done So Far

You've captured 36 observations using `hand_eye_calibration.py`. Each observation contains:
- **Timestamp**: When the observation was captured
- **Checkerboard pose**: The position and orientation of the checkerboard relative to the camera (rvec, tvec)

## What's Missing

The current observations are missing:
1. **Joint positions**: The actual servo positions (0-4095) of each joint when the observation was captured
2. **Gripper poses**: The calculated position and orientation of the gripper in the robot base frame

## Why We Need These

Hand-eye calibration solves the equation:

```
T_cam2base = T_gripper2base × T_target2gripper × T_cam2target
```

Where:
- `T_cam2base`: Transformation from camera to robot base (what we want to find)
- `T_gripper2base`: Transformation from gripper to base (calculated from joint positions using forward kinematics)
- `T_target2gripper`: Transformation from checkerboard to gripper (when gripper touches checkerboard, this is known)
- `T_cam2target`: Transformation from checkerboard to camera (already captured in observations)

## The Three Steps Explained

### 1. Implement Forward Kinematics

**What it is**: Forward kinematics calculates where the gripper is (position and orientation) given the joint angles.

**Why needed**: We need to know where the gripper was when you captured each observation.

**Status**: ✅ **DONE** - Created `pick_place_system/vision/forward_kinematics.py`

This module:
- Takes joint angles (or servo positions) as input
- Calculates the 4x4 transformation matrix of the gripper in base frame
- Uses the robot's link lengths (L1, L2, L3, L4, Lm)

### 2. Record Actual Gripper Poses

**What it is**: For each of your 36 observations, we need to know:
- The joint positions (servo values 0-4095) when that observation was captured
- Or the calculated gripper pose from those joint positions

**Why needed**: OpenCV's `cv2.calibrateHandEye()` needs pairs of:
- Gripper pose in base frame (from forward kinematics)
- Checkerboard pose in camera frame (already captured)

**Current status**: ❌ **MISSING** - Your observations don't have joint positions

**How to fix**: You have two options:

#### Option A: Add Joint Positions to Existing Observations

Edit `hand_eye_observations.json` and add `joint_positions` to each observation:

```json
{
  "timestamp": "2025-12-01T17:10:51.878663",
  "joint_positions": [2048, 2048, 2048, 2048],  // [base, shoulder, elbow, wrist]
  "rvec": [...],  // If you have these
  "tvec": [...]   // If you have these
}
```

To get joint positions:
- If you noted them down during capture, add them
- Or use the arm controller to read current positions (but this only works if arm is still in same position)
- Or re-capture observations with joint positions recorded automatically

#### Option B: Re-capture with Joint Positions

Modify `hand_eye_calibration.py` to automatically read joint positions from the arm controller when capturing.

### 3. Use cv2.calibrateHandEye() with Proper Transformations

**What it is**: OpenCV's function that solves the hand-eye calibration problem.

**Input needed**:
- List of gripper poses (rotation matrices and translation vectors)
- List of checkerboard poses (rotation matrices and translation vectors)

**Output**: 
- Transformation matrix from camera to robot base

**Status**: ✅ **IMPLEMENTED** - Created `complete_hand_eye_calibration.py`

## How to Complete the Calibration

### Step 1: Add Joint Positions to Observations

You need to add joint positions to your observations. Here's how:

1. **If you have the positions noted down**: Edit `hand_eye_observations.json` manually

2. **If you need to get them from the arm**: 
   - Use the arm controller to read positions
   - But this only works if the arm is still in the same positions (unlikely)

3. **Best option: Re-capture with automatic recording**:
   - Modify `hand_eye_calibration.py` to read joint positions automatically
   - Or use the improved version in `pick_place_system/calibration/`

### Step 2: Run Complete Calibration

Once observations have joint positions:

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
python complete_hand_eye_calibration.py --observations hand_eye_observations.json --square 24
```

This will:
1. Load your observations
2. Calculate gripper poses using forward kinematics
3. Use `cv2.calibrateHandEye()` to compute transformation
4. Save result to `hand_eye_calibration.json`

### Step 3: Use the Calibration

The output file can be loaded by `coordinate_transform.py` to convert between camera and robot coordinates.

## Simplified Approach (If You Can't Get Joint Positions)

If you can't get the exact joint positions for your 36 observations, you can:

1. **Re-capture with joint positions**: Use the improved calibration tool that records joint positions automatically

2. **Use approximate transformation**: The system can work with approximate camera position from config, but accuracy will be lower

3. **Manual measurement**: Measure camera position relative to base and enter in config file

## Next Steps

1. **Add joint positions to observations** (or re-capture)
2. **Run `complete_hand_eye_calibration.py`**
3. **Verify calibration** by testing coordinate transformation
4. **Use in pick and place system**

## Files Created

- `pick_place_system/vision/forward_kinematics.py` - Forward kinematics solver
- `camera_calibration/complete_hand_eye_calibration.py` - Calibration completion script
- This explanation document

