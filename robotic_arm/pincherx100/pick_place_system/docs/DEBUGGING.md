# Debugging Guide

## Camera Feed and Debug Mode

The CLI now supports debug mode to show camera feed with detections.

### Basic Usage with Debug

```bash
# Pick yogurt bottle with debug window
python scripts/arm_cli.py pick yogurt --debug

# Pick orange bottle with debug window
python scripts/arm_cli.py pick orange --debug

# Drop with checkerboard detection debug
python scripts/arm_cli.py drop --debug
```

### Test Detection Without Moving Arm

Use the `test_detection` command to see what the camera sees:

```bash
python scripts/arm_cli.py test_detection
```

This will:
- Show live camera feed with detections
- Display detection count and details
- Press 'Q' to quit
- Press 'S' to save current frame

## YOLO Model Selection

The default model is `yolov8n.pt` (nano, fastest). For better accuracy, try larger models:

```bash
# Use small model (better accuracy, slightly slower)
python scripts/arm_cli.py pick yogurt --model yolov8s.pt --debug

# Use medium model (best accuracy, slower)
python scripts/arm_cli.py pick yogurt --model yolov8m.pt --debug
```

### Available Models

- `yolov8n.pt` - Nano (fastest, lowest accuracy) - **Default**
- `yolov8s.pt` - Small (good balance)
- `yolov8m.pt` - Medium (better accuracy)
- `yolov8l.pt` - Large (high accuracy)
- `yolov8x.pt` - Extra Large (best accuracy, slowest)

**Note**: Larger models will download automatically on first use.

## Configuration

Edit `configs/robot_config.yaml` to set default model:

```yaml
detection:
  yolo_model: "yolov8s.pt"  # Change default model
  confidence_threshold: 0.3  # Lower = more detections
```

## Troubleshooting Detection Issues

### No Objects Detected

1. **Test detection first**:
   ```bash
   python scripts/arm_cli.py test_detection
   ```

2. **Check lighting**: Ensure good lighting on objects

3. **Try larger model**:
   ```bash
   python scripts/arm_cli.py pick yogurt --model yolov8s.pt --debug
   ```

4. **Lower confidence threshold** in config:
   ```yaml
   detection:
     confidence_threshold: 0.2  # More sensitive
   ```

5. **Check color detection**: The system uses color filtering (green cap for orange bottle, pink cap for yogurt). If caps are not visible, detection may fail.

### Wrong Object Type Detected

1. **Use debug mode** to see what's detected:
   ```bash
   python scripts/arm_cli.py pick yogurt --debug
   ```

2. **Check color match**: The debug window shows if color matching succeeded

3. **Improve lighting**: Better lighting improves color detection

### Debug Output

When using `--debug`, you'll see:
- All detections found (even if not matching target type)
- Detection confidence scores
- Color match status
- Selected detection highlighted in green

## Example Workflow

1. **Test detection first**:
   ```bash
   python scripts/arm_cli.py test_detection
   ```
   Verify objects are detected correctly.

2. **Try pick with debug**:
   ```bash
   python scripts/arm_cli.py pick yogurt --debug
   ```
   Review the detection window before arm moves.

3. **If detection fails, try better model**:
   ```bash
   python scripts/arm_cli.py pick yogurt --model yolov8s.pt --debug
   ```

4. **Once working, run without debug**:
   ```bash
   python scripts/arm_cli.py pick yogurt
   ```

