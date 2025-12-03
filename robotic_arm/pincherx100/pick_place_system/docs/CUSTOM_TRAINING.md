# Custom YOLO Model Training Guide

## When to Train a Custom Model

For top-down camera views where only bottle caps are visible, **color-based cap detection is recommended** (already implemented in `cap_detector.py`). However, if you want to train a custom YOLO model, this guide will help.

## Training vs. Color Detection

### Color-Based Detection (Recommended for Top-Down)
- **Pros**: Fast, no training needed, works well for distinct colors
- **Cons**: Sensitive to lighting, may fail if colors are similar
- **Best for**: Top-down views with distinct cap colors (green/pink)

### Custom YOLO Model
- **Pros**: More robust to lighting, can learn complex features
- **Cons**: Requires labeled data, training time, more complex
- **Best for**: Side views, complex scenes, or when color detection fails

## Training a Custom YOLO Model

### Step 1: Collect Training Data

1. **Capture images** of bottles from your top-down camera:
   ```bash
   python scripts/arm_cli.py test_detection
   # Press 'S' to save frames with bottles visible
   ```

2. **Collect diverse images**:
   - Different lighting conditions
   - Different bottle positions
   - Multiple bottles in frame
   - Various backgrounds

3. **Aim for 100-200 images minimum** (more is better)

### Step 2: Label Images

Use a labeling tool like [LabelImg](https://github.com/tzutalin/labelImg) or [Roboflow](https://roboflow.com/):

1. **Install LabelImg**:
   ```bash
   pip install labelImg
   labelImg
   ```

2. **Create classes**:
   - `orange_bottle_cap` (green cap)
   - `yogurt_bottle_cap` (pink cap)

3. **Label bounding boxes** around each cap in all images

4. **Export in YOLO format** (creates `.txt` files with bounding boxes)

### Step 3: Organize Dataset

```
dataset/
├── images/
│   ├── train/
│   │   ├── img001.jpg
│   │   ├── img002.jpg
│   │   └── ...
│   └── val/
│       ├── img101.jpg
│       └── ...
└── labels/
    ├── train/
    │   ├── img001.txt
    │   ├── img002.txt
    │   └── ...
    └── val/
        ├── img101.txt
        └── ...
```

### Step 4: Create Dataset Configuration

Create `dataset.yaml`:

```yaml
path: /path/to/dataset
train: images/train
val: images/val

names:
  0: orange_bottle_cap
  1: yogurt_bottle_cap
```

### Step 5: Train Model

```python
from ultralytics import YOLO

# Load a pretrained model
model = YOLO('yolov8n.pt')  # Start with nano for speed

# Train the model
results = model.train(
    data='dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='bottle_caps'
)

# The trained model will be saved in runs/detect/bottle_caps/weights/best.pt
```

### Step 6: Use Custom Model

```bash
# Update config
# In robot_config.yaml:
detection:
  use_cap_detector: false  # Disable cap detector
  yolo_model: "runs/detect/bottle_caps/weights/best.pt"  # Use custom model

# Or use via command line
python scripts/arm_cli.py pick yogurt --model runs/detect/bottle_caps/weights/best.pt
```

## Alternative: Use Roboflow

[Roboflow](https://roboflow.com/) provides an easier workflow:

1. Upload images to Roboflow
2. Label online (or use auto-labeling)
3. Generate dataset
4. Train model in cloud
5. Download trained model

## Recommendations

**For your use case (top-down camera):**

1. **Start with color-based cap detection** (already implemented)
   - It's simpler and works well for distinct colors
   - No training required
   - Fast and reliable

2. **If color detection fails**, try:
   - Adjusting color ranges in `cap_detector.py`
   - Improving lighting
   - Using a custom YOLO model as fallback

3. **Train custom YOLO model only if**:
   - Color detection consistently fails
   - You need to detect other objects
   - You want more robustness to lighting changes

## Quick Test

Test color-based cap detection:

```bash
# Test cap detector
python vision/cap_detector.py
```

This will show live detection of caps using color and shape analysis.

