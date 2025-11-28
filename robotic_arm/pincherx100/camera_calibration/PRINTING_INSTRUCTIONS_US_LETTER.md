# Printing Instructions for US Letter Paper

##  Your Printer Setup

- **Paper Size:** US Letter (8.5" × 11" = 215.9mm × 279.4mm)
- **NOT A4:** A4 is 210mm × 297mm (different!)
- **Pattern File:** `checkerboard_6x7_tiny_US_letter.svg`

##  Why This Matters

The original SVG was designed for A4 paper. When you print an A4-sized SVG on US Letter paper, the printer scales it automatically, which throws off the measurements!

**Solution:** Use `checkerboard_6x7_tiny_US_letter.svg` - specifically designed for US Letter.

##  Step-by-Step Printing

### Method 1: Print from Browser (Easiest)

```bash
# Open the US Letter version
firefox checkerboard_6x7_tiny_US_letter.svg
# or
chromium-browser checkerboard_6x7_tiny_US_letter.svg
# or
open checkerboard_6x7_tiny_US_letter.svg  # Mac
```

**Print Settings:**
1. File → Print
2. **Page Size:** US Letter (8.5" × 11")
3. **Scale:** 100% or "Actual Size" or "Do Not Scale"
4. **Margins:** Default is fine
5. Print!

### Method 2: Inkscape (Most Precise)

```bash
# Install Inkscape if needed
sudo apt install inkscape  # Raspberry Pi
# or brew install inkscape  # Mac

# Open file
inkscape checkerboard_6x7_tiny_US_letter.svg
```

**Print Settings in Inkscape:**
1. File → Document Properties
2. Page size: US Letter (8.5 × 11 in)
3. File → Print
4. **Rendering:** "Print as bitmap" - UNCHECKED
5. **Scale:** 100%
6. Print!

### Method 3: Export to PDF, Then Print

```bash
# Convert to PDF (maintains exact size)
inkscape checkerboard_6x7_tiny_US_letter.svg \
  --export-filename=checkerboard.pdf \
  --export-area-page

# Print the PDF
# Make sure printer settings: US Letter, 100% scale, no "Fit to Page"
```

##  Verification After Printing

**CRITICAL STEP:** Measure with a ruler!

```
┌───────┬───────┐
│ ■ Black│ □ White│
│ Square│ Square│
└───────┴───────┘
   ↑
Measure this: should be ~12mm

If you measure:
• 12.0mm → Perfect! Use --square 12
• 11.5mm → Use --square 11.5
• 11.0mm → Use --square 11
• 13.0mm → Use --square 13
```

**How to verify total size:**
1. Measure one square (e.g., 12mm)
2. Count squares: 7 wide
3. Calculate: 7 × 12mm = 84mm = 8.4cm ✓

If calculation matches visual measurement, you're good!

## 🖨 Common Printing Issues

### Issue 1: "Scale doesn't look right"

**Cause:** Printer scaling to fit page
**Solution:** 
- Look for "100%" or "Actual Size" option
- Disable "Fit to Page"
- Disable "Scale to Fit"
- Select "Do Not Scale"

### Issue 2: "Squares measure 10mm instead of 12mm"

**This is FINE!** Just use the measured value:
```bash
python calibrate_camera.py --pattern 6x7 --square 10
```

### Issue 3: "Pattern is cut off"

**Cause:** Margins too small
**Solution:** Pattern is 8.4cm × 9.6cm, fits easily on US Letter with standard margins. Check printer settings.

### Issue 4: "Using Inkscape, PDF export is wrong size"

Make sure:
```bash
# Correct export command
inkscape checkerboard_6x7_tiny_US_letter.svg \
  --export-type=pdf \
  --export-filename=output.pdf \
  --export-area-page \
  --export-dpi=300
```

Then print PDF at 100% scale.

##  Expected Measurements

| Element | Designed Size | Your Measurement |
|---------|--------------|------------------|
| One square | 12mm | _______ mm (measure this!) |
| Width (7 squares) | 84mm (8.4cm) | _______ mm |
| Height (8 squares) | 96mm (9.6cm) | _______ mm |

**Use YOUR measured square size in calibration!**

##  Calibration Command

After measuring, use:

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate

# Replace XX with your measured square size in mm
python calibrate_camera.py --pattern 6x7 --square XX
```

**Examples:**
```bash
# If you measured 12.0mm per square:
python calibrate_camera.py --pattern 6x7 --square 12

# If you measured 11.5mm per square:
python calibrate_camera.py --pattern 6x7 --square 11.5

# If you measured 10.8mm per square:
python calibrate_camera.py --pattern 6x7 --square 10.8
```

##  Printer-Specific Tips

### HP Printers
- Settings → Print at: 100% (normal size)
- Uncheck "Fit to page"

### Epson Printers
- Page Setup → Size: US Letter
- Scale: 100%
- Uncheck "Fit to page"

### Brother Printers
- Properties → Page Setup: US Letter
- Scaling: None or 100%

### Canon Printers
- Page Setup → Paper Size: Letter
- Print Options → Scaling: 100%

##  Quick Checklist

Before calibration:
- [ ] Printed on US Letter paper
- [ ] Printed at 100% scale (no "Fit to Page")
- [ ] Measured one square with ruler
- [ ] Noted measured size (e.g., 12mm)
- [ ] Mounted on rigid flat surface
- [ ] Pattern is perfectly flat (no warping)
- [ ] Ready to use measured size in calibration

##  Pro Tips

1. **Test print first:** Print on plain paper, measure, adjust if needed
2. **Use cardstock:** Heavier paper = easier to mount flat
3. **Foam board mounting:** Best option for rigidity
4. **Measure carefully:** Use a good ruler with mm markings
5. **Average measurements:** Measure 3-4 squares, use average

##  Related Files

- `checkerboard_6x7_tiny_US_letter.svg` - **USE THIS ONE** for US Letter
- `checkerboard_6x7_tiny.svg` - Original (A4 sized, may cause scaling issues)
- `PRINTER_LIMITATION_GUIDE.md` - Why size doesn't matter
- `QUICK_REFERENCE.md` - Fast reference

## ❓ FAQ

**Q: Can I use the A4 version on US Letter?**
A: Possible but not recommended - it will scale and measurements will be off. Use the US Letter version.

**Q: What if my printer automatically scales?**
A: Try printing to PDF first (at 100%), then print the PDF. Or measure the actual printed size and use that.

**Q: Does it matter if squares are 11mm instead of 12mm?**
A: NO! Just use `--square 11` in calibration. Any size works!

**Q: Can I scale it to fit exactly 8.5cm × 10cm?**
A: You can, but easier to just measure what you get and use that value. The algorithm doesn't care about absolute size!

---

**Remember:** The most important step is MEASURING the actual printed square size! 

