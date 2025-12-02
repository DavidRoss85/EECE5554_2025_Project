# Mission Control System - Integration Info
---

## What I'm Publishing (What You Subscribe To)

### `/mission/state` (std_msgs/String)
**Current robot state - EVERYONE SHOULD SUBSCRIBE TO THIS**

Possible values:
- `"IDLE"` - Waiting for commands
- `"DETECTING"` - Searching for target object
- `"APPROACHING"` - Moving toward object
- `"PICKING"` - Executing grasp
- `"DONE"` - Mission complete

### `/mission/status` (std_msgs/String)
Human-readable status messages (optional, for debugging)

---

## What I'm Subscribing To (What You Need to Publish)

### From Detection:
- `/objects/detections` (RSyncDetectionList)
- `/objects/locations` (RSyncLocationList)

### From Visual Servo:
**Topic:** `/visual_servo/status` (std_msgs/String)

**When to publish:**
- When robot is positioned correctly (distance ~0.4m, aligned with object)
- Publish message: `"aligned"`

**Code to add to your `approach_controller_node.py`:**
See the integration code guide for specific code changes.

### From Arm:
**Topic:** `/arm/status` (std_msgs/String)

**When to publish:**
- After pick attempt completes
- Publish `"picked"` if successful
- Publish `"error"` if failed

## What Arm side Needs to Create

Create a simple wrapper node that:
1. Subscribes to `/mission/state`
2. When state = `"PICKING"`, calls your existing arm code
3. Publishes result to `/arm/status`
---

## How the System Works (Complete Flow)

1. **User clicks "Pick Bottle" in web dashboard**
   - Web UI publishes to `/mission/command`: `"pick_bottle"`

2. **State Machine → DETECTING**
   - Waits for `/objects/detections` to find "bottle"
   - Timeout: 10 seconds

3. **State Machine → APPROACHING** (when bottle detected)
   - Publishes `/mission/state`: `"APPROACHING"`
   - Visual servo: Your node should start moving robot
   - When aligned, publish to `/visual_servo/status`: `"aligned"`

4. **State Machine → PICKING** (when aligned)
   - Publishes `/mission/state`: `"PICKING"`
   - Arm team: Your node should execute pick
   - When done, publish to `/arm/status`: `"picked"` or `"error"`

5. **State Machine → DONE → IDLE**
   - Mission complete, ready for next command

---

## Testing Commands
### Terminal Setup:

**Terminal 1 - rosbridge:**
```bash
ros2 run rosbridge_server rosbridge_websocket
```

**Terminal 2 - sync node:**
```bash
ros2 run object_location robo_sync_node
```

**Terminal 3 - detection:**
```bash
ros2 run object_location detection_node
```

**Terminal 4 - distance:**
```bash
ros2 run object_location distance_node
```

**Terminal 5 - Visual servo:**
```bash
ros2 run object_location approach_controller_node
```

**Terminal 6 - state machine:**
```bash
~/ros2_ws/install/mission_control/bin/state_machine
```

**Terminal 7 - Dashboard:**
```bash
firefox ~/ros2_ws/src/mission_control/web/dashboard.html
```