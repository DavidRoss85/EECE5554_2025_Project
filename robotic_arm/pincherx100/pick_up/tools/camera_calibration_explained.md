# Camera Calibration Explained

## What Camera Calibration Does

Camera calibration finds the **intrinsic parameters** of your camera:
- **fx, fy**: Focal length in pixels (how "zoomed in" the camera is)
- **cx, cy**: Optical center / principal point (where the optical axis hits the sensor)
- **Distortion coefficients**: k1, k2, k3, p1, p2 (lens distortion)

## The Fundamental Equation

The pinhole camera model projects 3D world points to 2D image pixels:

```
         ┌─────────────────┐
         │ fx   0    cx    │     ┌───┐
[u, v] = │  0   fy   cy    │  ×  │ X │  ÷ Z
         │  0   0     1    │     │ Y │
         └─────────────────┘     │ Z │
                                 └───┘

Where:
  (X, Y, Z) = 3D point in camera frame
  (u, v)    = 2D pixel in image
  fx, fy    = focal length in pixels
  cx, cy    = principal point (optical center)
```

Simplified:
```
u = fx × (X/Z) + cx
v = fy × (Y/Z) + cy
```

## How the Checkerboard Provides Information

### What You Tell the Calibration Algorithm

1. **Checkerboard square size**: 24mm × 24mm
2. **Number of inner corners**: e.g., 6×7 grid has 5×6 = 30 inner corners

### What the Algorithm Detects

From each calibration image:
- **Pixel coordinates** (u, v) of each corner in the image
- Example: Corner 1 at pixel (345.2, 678.9), Corner 2 at (389.4, 681.2), etc.

### What the Algorithm Knows

Since you told it the square size is 24mm:
- **Real-world coordinates** of each corner relative to the checkerboard
- Example: If bottom-left is origin (0, 0, 0):
  - Corner at (0mm, 0mm, 0mm)
  - Corner at (24mm, 0mm, 0mm)
  - Corner at (48mm, 0mm, 0mm)
  - Corner at (0mm, 24mm, 0mm)
  - etc.

## How Focal Length is Calculated

### The Key Insight

When you move the checkerboard to different distances:

**At Z = 50cm (close to camera):**
- A 24mm square appears LARGE in the image (maybe 100 pixels wide)
- Detected at pixel coordinates: (200, 300) to (300, 300)
- Size in pixels: 100 pixels

**At Z = 70cm (far from camera):**
- The SAME 24mm square appears SMALLER (maybe 70 pixels wide)
- Detected at pixel coordinates: (215, 315) to (285, 315)
- Size in pixels: 70 pixels

### The Calculation

Using the projection equation:
```
pixel_width = fx × (real_width / Z)

At Z=50cm: 100 pixels = fx × (24mm / 500mm)
At Z=70cm:  70 pixels = fx × (24mm / 700mm)

Solving: fx = 100 / (24/500) = 2083 pixels
         fx =  70 / (24/700) = 2042 pixels
```

With multiple images at different distances and angles, the algorithm finds the **best fx that explains all observations**.

## Why Move the Checkerboard?

### 1. Different Distances (Z values change)

**Purpose**: Calibrate focal length (fx, fy)

- Close to camera: Checkerboard appears large
- Far from camera: Checkerboard appears small
- The ratio of sizes tells you the focal length

**Example:**
```
Image 1: Checkerboard at 30cm → appears 150 pixels wide
Image 2: Checkerboard at 60cm → appears 75 pixels wide
Ratio: 150/75 = 2.0 (exactly matches 30cm/60cm = 0.5 inverted)
This helps solve for fx
```

### 2. Different Angles (Rotation changes)

**Purpose**: Cover the full image, calibrate distortion

- Checkerboard straight-on
- Checkerboard tilted left/right
- Checkerboard tilted up/down
- Checkerboard in corners of image

**Why it helps:**
- Lens distortion is not uniform across the image
- More distortion at edges and corners
- Different angles put the checkerboard at different image locations
- This maps out the distortion field

### 3. Different Positions (X, Y changes)

**Purpose**: Calibrate optical center (cx, cy), verify distortion

- Checkerboard in center of image
- Checkerboard at top, bottom, left, right
- This helps find where the optical axis hits the sensor

## Checkerboard Size Requirements

### Does it need to fill the entire image?

**NO!** But:
- **Too small**: Hard to detect accurately, poor calibration
- **Too large**: Hard to get different angles
- **Ideal**: Occupies 30-60% of the image

### How large should it be?

For a camera at 50-70cm distance:
- **Checkerboard**: 20cm × 15cm total size works well
- **Square size**: 24mm × 24mm is good
- **Pattern**: 8×6 or 9×7 grid (gives 7×5 or 8×6 inner corners)

### Example Calculation

If your camera is at Z = 60cm (600mm) and has fx ≈ 800 pixels:

```
Real checkerboard width: 200mm (20cm)
Pixel width in image: fx × (200mm / 600mm) = 800 × 0.333 = 267 pixels

If image is 1280 pixels wide:
Checkerboard occupies: 267/1280 = 21% of image width ✓ Good!

If checkerboard was 600mm wide:
Pixel width: 800 × (600mm / 600mm) = 800 pixels
Occupies: 800/1280 = 62% of image ✓ Still okay!
```

## What Information is Given to Calibration?

### Input to OpenCV calibrateCamera():

```python
# For each calibration image:

# 1. Object points (real-world coordinates in mm or m)
object_points = [
    [0, 0, 0],      # Corner 0: origin
    [24, 0, 0],     # Corner 1: 24mm to the right
    [48, 0, 0],     # Corner 2: 48mm to the right
    [0, 24, 0],     # Corner 3: 24mm up
    [24, 24, 0],    # Corner 4: diagonal
    # ... for all corners
]

# 2. Image points (detected pixel coordinates)
image_points = [
    [345.2, 678.9],  # Corner 0 at pixel (345.2, 678.9)
    [389.4, 681.2],  # Corner 1 at pixel (389.4, 681.2)
    [433.8, 683.1],  # Corner 2 at pixel (433.8, 683.1)
    [343.1, 712.4],  # Corner 3 at pixel (343.1, 712.4)
    # ... for all corners
]

# 3. Image size
image_size = (1280, 720)  # width × height in pixels
```

### The Algorithm Solves For:

```python
# Camera intrinsic matrix
K = [fx   0   cx]
    [0   fy   cy]
    [0    0    1]

# Distortion coefficients
dist = [k1, k2, p1, p2, k3]

# And for each image, the camera pose (rotation & translation)
```

## Example: Your Setup

### Given Information

- **Checkerboard square size**: 24mm × 24mm
- **Pattern**: 9×7 grid (8×6 inner corners)
- **Camera distance**: 50-70cm from platform
- **Image resolution**: 1280×720 pixels

### Expected Checkerboard Size in Image

At Z = 60cm with typical webcam (fx ≈ 800):

```
Checkerboard: 8 squares × 24mm = 192mm wide

Pixel width = 800 × (192mm / 600mm) = 256 pixels
Percentage = 256 / 1280 = 20% of image width ✓ Good!
```

### What Happens at Different Distances

**At Z = 50cm (20cm from platform, 70cm from camera):**
```
Pixel width = 800 × (192mm / 500mm) = 307 pixels (24% of image)
Checkerboard appears LARGER
```

**At Z = 70cm (platform level, 70cm from camera):**
```
Pixel width = 800 × (192mm / 700mm) = 219 pixels (17% of image)  
Checkerboard appears SMALLER
```

**The ratio**: 307/219 = 1.40 matches 700mm/500mm = 1.40 ✓

This is how the algorithm solves for focal length!

## Best Practices for Calibration

### 1. Capture Strategy

Take 15-25 images with:
- **5 images** at different distances (30cm, 45cm, 60cm, 75cm, 90cm)
- **At each distance**, capture at different angles:
  - Straight-on (perpendicular to camera)
  - Tilted left ~20-30°
  - Tilted right ~20-30°
  - Tilted up ~20-30°
  - Tilted down ~20-30°

### 2. Coverage

- Checkerboard should appear in:
  - Center of image (several times)
  - Top-left corner (at least once)
  - Top-right corner (at least once)
  - Bottom-left corner (at least once)
  - Bottom-right corner (at least once)

### 3. Quality Checks

**Good image:**
- All corners detected (software draws them)
- Checkerboard sharp and in focus
- Good lighting, no glare
- Checkerboard flat, not bent

**Bad image:**
- Blurry or out of focus
- Too dark or bright (clipping)
- Checkerboard bent or wrinkled
- Motion blur

## Why This Works

### The Key Principle

The camera calibration exploits the fact that:

1. **You know real-world dimensions** (24mm squares)
2. **You measure pixel dimensions** (detected corners)
3. **The ratio depends on focal length and distance**

By observing the SAME known pattern at:
- Different distances → solves for focal length
- Different angles → solves for distortion
- Different positions → solves for optical center

With 15-25 images, you have:
- 15-25 × 30 corners = 450-750 observations
- Solving for only 9-14 unknowns (fx, fy, cx, cy, 5 distortion coefficients, plus poses)

This **overdetermined system** gives a robust solution.

## Common Misconceptions

### ❌ "Checkerboard must fill the image"
**✓ FALSE**: 20-60% coverage is ideal. Too large makes it hard to get variety.

### ❌ "All images should be at the same distance"
**✓ FALSE**: Different distances are CRUCIAL for estimating focal length.

### ❌ "Need to tell the algorithm the camera distance"
**✓ FALSE**: The algorithm figures it out! You only specify square size.

### ❌ "More squares = better calibration"
**✓ PARTLY TRUE**: More corners help, but 8×6 = 48 corners is plenty. More images at varied poses is more important than a giant checkerboard.

## Your Calibration Checklist

- [ ] Print checkerboard with 24mm squares (use provided SVG/PDF)
- [ ] Mount on flat, rigid surface (cardboard, foam board)
- [ ] Measure one square with ruler to verify 24mm
- [ ] Take 20 images:
  - [ ] 5 at 30cm from camera (close)
  - [ ] 5 at 50cm from camera (medium)
  - [ ] 5 at 70cm from camera (far - platform level)
  - [ ] 5 at 90cm from camera (very far)
  - [ ] For each distance: straight, tilt left, tilt right, tilt up, tilt down
- [ ] Verify all corners detected in each image
- [ ] Run calibration script
- [ ] Check reprojection error < 0.5 pixels (good) or < 1.0 pixels (acceptable)

---

**Summary**: The checkerboard provides known real-world measurements. By observing how these measurements appear in pixels at different distances and angles, the algorithm solves for the camera's intrinsic parameters (focal length, optical center, distortion). The key is variety: different distances for focal length, different angles for distortion, different positions for optical center.

