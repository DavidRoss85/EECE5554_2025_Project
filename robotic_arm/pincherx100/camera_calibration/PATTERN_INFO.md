# Checkerboard Pattern Information

## Available Patterns

###  RECOMMENDED: Small Pattern (checkerboard_6x5_small.svg)

**Specifications:**
- **Inner corners:** 6 x 5
- **Total squares:** 7 x 6 = 42 squares
- **Square size:** 15mm (1.5cm)
- **Total dimensions:** 105mm x 90mm (10.5cm x 9cm)
- **Fits under:** 10cm x 10cm requirement ✓

**Usage:**
```bash
python calibrate_camera.py --pattern 6x5 --square 15
```

**Advantages:**
-  Fits easily on standard paper with margins
-  Easy to handle and position
-  Perfect for overhead camera (covers good field of view)
-  Easier to print without scaling issues
-  Less likely to bend or warp

### Alternative: Large Pattern (checkerboard_8x6.svg)

**Specifications:**
- **Inner corners:** 8 x 6
- **Total squares:** 9 x 7 = 63 squares
- **Square size:** 25mm (2.5cm)
- **Total dimensions:** 225mm x 175mm (22.5cm x 17.5cm)
- **Note:** Exceeds 10cm x 10cm limit

**Usage:**
```bash
python calibrate_camera.py --pattern 8x6 --square 25
```

**When to use:**
- If camera is mounted very high
- If you need larger pattern for wide-angle lens
- If printer supports larger prints accurately

## Calibration Method

**Zhang's Method** (Zhengyou Zhang, 1999)

The calibration uses Zhang's flexible camera calibration technique, which is the standard method in OpenCV's `cv2.calibrateCamera()` function.

**How it works:**
1. Print a known pattern (checkerboard)
2. Capture images of pattern from different angles
3. Detect corners in each image (2D image points)
4. Match to known 3D world coordinates
5. Solve for camera intrinsic parameters and distortion

**Why it's good:**
-  Only requires planar pattern (checkerboard)
-  No expensive 3D calibration rig needed
-  Flexible - works with various camera types
-  Robust and widely validated
-  Industry standard since 1999

**Reference:**
Zhang, Z. (1999). "Flexible Camera Calibration By Viewing a Plane From Unknown Orientations". 
IEEE International Conference on Computer Vision (ICCV).

## Pattern Size Selection Guide

### For Overhead Camera on PincherX100:

**Camera height: 30-50cm above workspace**
→ Use **6x5 small pattern** (10.5cm x 9cm)

**Camera height: 50-80cm above workspace**
→ Use **8x6 large pattern** (22.5cm x 17.5cm) if you have space

**Rule of thumb:**
- Pattern should fill 30-70% of camera field of view
- Too small: Hard to detect accurately
- Too large: Can't capture varied angles easily
- **For PincherX100 setup: 6x5 small pattern is ideal**

## Custom Pattern Sizes

You can generate custom patterns:

```bash
# Tiny pattern (4x3 corners, 12mm squares = 6cm x 4.8cm)
python generate_checkerboard_pdf.py --size 4x3 --square 12

# Small pattern (6x5 corners, 15mm squares = 10.5cm x 9cm) [RECOMMENDED]
python generate_checkerboard_pdf.py --size 6x5 --square 15

# Medium pattern (7x5 corners, 20mm squares = 16cm x 12cm)
python generate_checkerboard_pdf.py --size 7x5 --square 20

# Large pattern (8x6 corners, 25mm squares = 22.5cm x 17.5cm)
python generate_checkerboard_pdf.py --size 8x6 --square 25
```

Then calibrate with matching parameters:
```bash
python calibrate_camera.py --pattern WxH --square SIZE
```

## Printing Tips

### Critical Requirements:
1. **Print at 100% scale** (actual size, no scaling)
2. **Verify with ruler** after printing
3. **Mount on rigid surface** (foam board best)
4. **Ensure perfectly flat** (no bending/warping)

### Printer Settings:
- **Scale:** 100% / Actual Size / Do Not Scale
- **Paper:** White cardstock (heavier = better)
- **Quality:** Best/High quality
- **Color:** Black & White (high contrast)

### Verification:
```
Measure with ruler:
- Each square = specified size (e.g., 15mm)
- Total width = squares × size (e.g., 7 × 15mm = 105mm)
- Total height = squares × size (e.g., 6 × 15mm = 90mm)
```

If measurements are off by >1mm, reprint with adjusted settings.

## Summary Table

| Pattern | Corners | Squares | Square Size | Total Size | Use Case |
|---------|---------|---------|-------------|------------|----------|
| **Small** | 6x5 | 7x6 | 15mm | 10.5×9cm | **Recommended for PincherX100** |
| Large | 8x6 | 9x7 | 25mm | 22.5×17.5cm | High camera or wide lens |
| Tiny | 4x3 | 5x4 | 12mm | 6×4.8cm | Very close camera |
| Medium | 7x5 | 8x6 | 20mm | 16×12cm | Alternative size |

---

**For PincherX100 with overhead camera, use the small pattern:**
- File: `checkerboard_6x5_small.svg`
- Command: `python calibrate_camera.py --pattern 6x5 --square 15`
- Size: 10.5cm x 9cm (fits under 10cm requirement)


