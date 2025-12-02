# Mission Control Integration Guide

## Modify approach_controller_node.py

ADD these lines to `approach_controller_node.py`:

### Add at top with other imports:
```python
from std_msgs.msg import String
```

### Add in `__init__()` method after line 71 (after other subscribers):

```python
# Subscribe to mission state
self.mission_state_sub = self.create_subscription(
    String,
    '/mission/state',
    self.mission_state_callback,
    10
)

# Publisher for alignment status
self.status_pub = self.create_publisher(
    String,
    '/visual_servo/status',
    10
)

self.mission_active = False  # Flag to track if mission is active
```

### Add this new method (anywhere in the class):

```python
def mission_state_callback(self, msg):
    """Handle mission state updates from mission control"""
    if msg.data == "APPROACHING":
        self.mission_active = True
        self.get_logger().info('Mission Control: Start approaching')
    elif msg.data == "IDLE":
        self.mission_active = False
        self.get_logger().info('Mission Control: Mission idle')
```

### Modify `control_loop()` method - Change line 162 from:
```python
def control_loop(self):
    if not self.enabled:
        return
```

**To:**
```python
def control_loop(self):
    if not self.enabled or not self.mission_active:
        return
```

### Add alignment detection - After line 183, add:

```python
        # Check if aligned (no more movement needed)
        if self.countdown_timer_forward <= 0 and self.countdown_timer_turn <= 0:
            if self.approaching:  # Only publish once
                status_msg = String()
                status_msg.data = 'aligned'
                self.status_pub.publish(status_msg)
                self.get_logger().info('✓ Robot aligned - notifying mission control')
                self.approaching = False  # Don't spam the message
```

---

## What Arm Needs

The arm team needs to create a node that:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArmMissionNode(Node):
    def __init__(self):
        super().__init__('arm_mission_node')
        
        # Subscribe to mission state
        self.state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.state_callback,
            10
        )
        
        # Publish arm status
        self.status_pub = self.create_publisher(
            String,
            '/arm/status',
            10
        )
        
        self.get_logger().info('Arm Mission Node ready')
    
    def state_callback(self, msg):
        if msg.data == "PICKING":
            self.get_logger().info('Starting pick sequence')
            # Call their existing arm control code
            success = self.execute_pick()  # Their function
            
            # Publish result
            status = String()
            status.data = 'picked' if success else 'error'
            self.status_pub.publish(status)
    
    def execute_pick(self):
        # Their existing arm control logic goes here
        # Return True if successful, False otherwise
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ArmMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

### Test Flow:

1. Click "Pick Bottle" in dashboard
2. State machine: IDLE → DETECTING
3. Detection finds bottle
4. State machine: DETECTING → APPROACHING
5. Visual servo moves robot
6. Visual servo publishes "aligned"
7. State machine: APPROACHING → PICKING
8. Arm picks object
9. Arm publishes "picked"
10. State machine: PICKING → DONE → IDLE

---

## Debugging Commands

```bash
# Check all active topics
ros2 topic list

# Monitor state machine output
ros2 topic echo /mission/state

# Monitor visual servo status
ros2 topic echo /visual_servo/status

# Monitor detections
ros2 topic echo /objects/detections

# Monitor locations
ros2 topic echo /objects/locations

# Send manual test command
ros2 topic pub --once /mission/command std_msgs/String "data: 'pick_bottle'"
```

---

## Quick Reference: Topic Map

```
Mission Control (YOU):
  Publishes: /mission/state, /mission/status
  Subscribes: /objects/detections, /objects/locations, 
              /visual_servo/status, /arm/status

David's Detection:
  Publishes: /objects/detections, /objects/locations
  
Visual Servo:
  Subscribes: /objects/locations, /mission/state
  Publishes: /visual_servo/status
  
Arm:
  Subscribes: /mission/state
  Publishes: /arm/status
```
