# Working with Printer Limitations (8.5cm x 10cm)

## Your Situation

Your printer can only print up to **8.5cm x 10cm**. This is fine! The calibration will work perfectly.

##  The Key Point: SIZE DOESN'T MATTER!

**What matters for calibration:**
1.  All squares are the **same size** as each other
2.  The pattern is **flat and rigid**
3.  You **measure the actual printed size** with a ruler
4.  You **tell the software** the measured size

**What doesn't matter:**
-  The absolute size (12mm, 15mm, 25mm - any size works!)
-  If it prints slightly smaller/larger than designed
-  The exact dimensions (8.4cm vs 10.5cm doesn't matter)

##  How It Works

Zhang's calibration method (used by OpenCV) works like this:

1. **Known geometry:** You tell it "each square is X mm"
2. **Detected corners:** Software finds corners in the image
3. **Math magic:** Solves for camera parameters using the known geometry

As long as you measure correctly, any size works!

##  Use the Optimized Pattern

I've created a pattern specifically for your **8.5cm x 10cm** limit:

**File:** `checkerboard_6x7_tiny.svg`
- **Pattern:** 6x7 inner corners (7x8 squares)
- **Designed size:** 8.4cm x 9.6cm (fits within 8.5cm x 10cm ✓)
- **Square size:** 12mm (but measure actual!)

##  Step-by-Step Process

### Step 1: Print the Pattern

```bash
# Open the tiny version
firefox checkerboard_6x7_tiny.svg
# or
chromium-browser checkerboard_6x7_tiny.svg
```

- Print at **100% scale** (actual size)
- Don't use "Fit to Page"
- Use white paper or cardstock

### Step 2: **CRITICAL** - Measure with Ruler

After printing, use a ruler to measure one square:

```
┌─────────┬─────────┐
│ Black   │ White   │
│ Square  │ Square  │
│         │         │
└─────────┴─────────┘
    ↑
  Measure this width
```

**Example measurements:**
- If you measure **12mm** → Perfect! Use `--square 12`
- If you measure **11.5mm** → Use `--square 11.5`
- If you measure **10mm** → Use `--square 10`
- If you measure **13mm** → Use `--square 13`

**The measured size is what matters, not the designed size!**

### Step 3: Mount on Rigid Surface

- Use foam board, cardboard, or thin wood
- Glue evenly to prevent warping
- Ensure perfectly flat

### Step 4: Calibrate with YOUR Measured Size

```bash
# If you measured 12mm squares:
python calibrate_camera.py --pattern 6x7 --square 12

# If you measured 11mm squares:
python calibrate_camera.py --pattern 6x7 --square 11

# If you measured 10.5mm squares:
python calibrate_camera.py --pattern 6x7 --square 10.5

# Use whatever you actually measured!
```

##  Why This Works

The calibration algorithm calculates:

```
Real World Coordinates = [0, 0], [12mm, 0], [24mm, 0], ...
                         [0, 12mm], [12mm, 12mm], ...
                         (based on your --square value)

Image Coordinates = Detected corners in pixels
                   e.g., (100, 200), (145, 198), ...

Camera Parameters = Solve the transformation between these
```

It doesn't matter if your squares are 8mm or 25mm - the math scales perfectly!

##  Pattern Comparison

| File | Size | Inner Corners | Fits Your Printer? |
|------|------|---------------|-------------------|
| **checkerboard_6x7_tiny.svg** | 8.4×9.6cm | 6×7 |  **YES** (Recommended) |
| checkerboard_6x5_small.svg | 10.5×9cm | 6×5 |  Too wide |
| checkerboard_8x6.svg | 22.5×17.5cm | 8×6 |  Too large |

##  Understanding the Pattern Size Parameter

```bash
python calibrate_camera.py --pattern 6x7 --square 12
                                     ↑           ↑
                            Inner corners  Square size
```

**Pattern 6x7 means:**
- 6 inner corners horizontally
- 7 inner corners vertically
- = 7 squares wide × 8 squares tall
- Total size = (7 × square_size) × (8 × square_size)

**If --square 12:**
- Width = 7 × 12mm = 84mm = 8.4cm ✓
- Height = 8 × 12mm = 96mm = 9.6cm ✓

##  Common Mistakes to Avoid

###  WRONG: Using designed size without measuring
```bash
# Don't assume it's 12mm if you didn't measure!
python calibrate_camera.py --pattern 6x7 --square 12
```

###  RIGHT: Measuring first
```bash
# 1. Print pattern
# 2. Measure actual square size with ruler (e.g., 11mm)
# 3. Use measured size:
python calibrate_camera.py --pattern 6x7 --square 11
```

###  WRONG: Measuring total size
```bash
# Don't measure the whole pattern and divide
# This introduces errors
```

###  RIGHT: Measuring one square
```bash
# Measure a single square carefully
# Use that exact measurement
```

## 🧪 Test Your Measurement

After printing, verify:

```
1. Measure one square: ______ mm
2. Count squares horizontally: 7 squares
3. Calculate: 7 × measured_size = ______ mm
4. This should be close to 8.4cm (84mm)

If close → good print! ✓
If very different → printer scaled it, remeasure carefully
```

##  Pro Tips

1. **Measure carefully:** Use a good ruler with mm markings
2. **Measure multiple squares:** Average 3-4 measurements
3. **Be precise:** Calibration accuracy depends on this!
4. **Don't guess:** Always measure, never assume

##  Troubleshooting

### "My squares printed at 10mm instead of 12mm"

**No problem!** Just use `--square 10` in calibration.

### "My pattern is 7.5cm x 9cm instead of 8.4cm x 9.6cm"

**No problem!** Measure one square size and use that value.

### "Can I use the 6x5 pattern if I scale it down?"

**Yes, but simpler to use 6x7 tiny version.** If you scale:
1. Measure actual printed square size
2. Use that measured size in `--square`

### "Will smaller pattern affect accuracy?"

**No!** Calibration accuracy depends on:
- Number of images (15-20+)
- Variety of angles/positions
- Pattern flatness
- Measurement accuracy

NOT on absolute pattern size.

##  Summary

**For your 8.5cm x 10cm printer:**

1. Print: `checkerboard_6x7_tiny.svg` at 100% scale
2. Measure: One square with ruler (e.g., 12mm)
3. Mount: On rigid flat surface
4. Calibrate: `python calibrate_camera.py --pattern 6x7 --square [measured_size]`

**The calibration will work perfectly regardless of the exact printed size!**

---

**Questions?** The key is: measure accurately and use the measured value. That's it! 

