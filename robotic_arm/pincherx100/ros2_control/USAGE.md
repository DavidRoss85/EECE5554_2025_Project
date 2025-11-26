# PincherX100 ROS2 Usage Guide

Comprehensive guide for controlling the PincherX100 arm using ROS2.

## Table of Contents

1. [Starting the Controller](#starting-the-controller)
2. [Using Services (Preset Poses)](#using-services-preset-poses)
3. [Position Control](#position-control)
4. [Velocity Control](#velocity-control)
5. [Trajectory Execution](#trajectory-execution)
6. [Monitoring Joint States](#monitoring-joint-states)
7. [Python Examples](#python-examples)
8. [C++ Examples](#c-examples)
9. [Command-Line Examples](#command-line-examples)
10. [Troubleshooting](#troubleshooting)

---

## Starting the Controller

### Basic Start

```bash
# Enter Docker container
docker exec -it pincherx100_ros2 bash

# Launch controller
ros2 launch pincherx100_control arm_control.launch.py
```

### With Custom Parameters

```bash
# Custom serial port
ros2 launch pincherx100_control arm_control.launch.py port:=/dev/ttyUSB1

# Custom baudrate
ros2 launch pincherx100_control arm_control.launch.py baudrate:=57600

# Custom config file
ros2 launch pincherx100_control arm_control.launch.py \
  config_file:=/path/to/custom_config.yaml
```

---

## Using Services (Preset Poses)

Services provide the simplest way to control the arm with predefined poses.

### Available Services

- `/go_home` - Move to home position (center of all joints)
- `/go_sleep` - Move to compact rest position
- `/go_ready` - Move to ready-to-grasp position
- `/gripper_open` - Open gripper
- `/gripper_close` - Close gripper

### Command-Line Usage

```bash
# Move to home position
ros2 service call /go_home std_srvs/srv/Trigger

# Move to ready position
ros2 service call /go_ready std_srvs/srv/Trigger

# Open gripper
ros2 service call /gripper_open std_srvs/srv/Trigger

# Close gripper
ros2 service call /gripper_close std_srvs/srv/Trigger

# Move to sleep position
ros2 service call /go_sleep std_srvs/srv/Trigger
```

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.home_client = self.create_client(Trigger, '/go_home')
        self.gripper_open_client = self.create_client(Trigger, '/gripper_open')
        
    def go_home(self):
        request = Trigger.Request()
        future = self.home_client.call_async(request)
        return future
    
    def open_gripper(self):
        request = Trigger.Request()
        future = self.gripper_open_client.call_async(request)
        return future

def main():
    rclpy.init()
    commander = ArmCommander()
    
    # Go home
    future = commander.go_home()
    rclpy.spin_until_future_complete(commander, future)
    result = future.result()
    print(f"Home: {result.success} - {result.message}")
    
    # Open gripper
    future = commander.open_gripper()
    rclpy.spin_until_future_complete(commander, future)
    result = future.result()
    print(f"Gripper: {result.success} - {result.message}")
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Position Control

Control individual joint positions directly.

### Topic Information

- **Topic**: `/joint_position_commands`
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Format**: `[base, shoulder, elbow, wrist, gripper]` in **radians**
- **Range**: 0 to 2π radians (0-360°), mapped to 0-4095 Dynamixel units

### Joint Mapping

- Index 0: **Base** (rotation left/right)
- Index 1: **Shoulder** (up/down)
- Index 2: **Elbow** (up/down)
- Index 3: **Wrist** (up/down)
- Index 4: **Gripper** (open/close)

### Command-Line Usage

```bash
# Move all joints to center (π radians ≈ 3.14)
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [3.14, 3.14, 3.14, 3.14, 3.14]"

# Move base to 90 degrees (π/2 radians ≈ 1.57)
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [1.57, 3.14, 3.14, 3.14, 3.14]"

# Move shoulder up (higher values)
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [3.14, 4.0, 3.14, 3.14, 3.14]"

# Close gripper
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [3.14, 3.14, 3.14, 3.14, 2.5]"
```

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class PositionCommander(Node):
    def __init__(self):
        super().__init__('position_commander')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_position_commands',
            10
        )
    
    def send_positions(self, positions):
        """Send position command (list of 5 floats in radians)."""
        msg = Float64MultiArray()
        msg.data = positions
        self.pub.publish(msg)
        self.get_logger().info(f'Sent positions: {positions}')

def main():
    rclpy.init()
    commander = PositionCommander()
    
    # Move to center
    commander.send_positions([math.pi] * 5)
    
    rclpy.spin_once(commander, timeout_sec=0.1)
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Velocity Control

Control joint velocities (converted to position control internally).

### Topic Information

- **Topic**: `/joint_velocity_commands`
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Format**: `[base, shoulder, elbow, wrist, gripper]` in **rad/s**

### Command-Line Usage

```bash
# Rotate base slowly (0.5 rad/s)
ros2 topic pub --rate 10 /joint_velocity_commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.0, 0.0, 0.0, 0.0]"

# Stop all joints
ros2 topic pub --once /joint_velocity_commands std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
```

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_commands',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity = 0.3  # rad/s
        self.direction = 1
        self.count = 0
    
    def timer_callback(self):
        msg = Float64MultiArray()
        # Oscillate base joint
        msg.data = [self.velocity * self.direction, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
        
        self.count += 1
        if self.count > 30:  # Reverse direction every 3 seconds
            self.direction *= -1
            self.count = 0

def main():
    rclpy.init()
    commander = VelocityCommander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Trajectory Execution

Execute smooth trajectories with multiple waypoints.

### Using Topic

```bash
# Not recommended for command-line due to message complexity
# See Python/C++ examples below
```

### Using Action Server (Recommended)

The action server provides feedback during trajectory execution.

- **Action**: `/follow_joint_trajectory`
- **Type**: `control_msgs/action/FollowJointTrajectory`

### Python Example - Simple Trajectory

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory'
        )
    
    def send_trajectory(self):
        # Wait for action server
        self.action_client.wait_for_server()
        
        # Create trajectory
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['base', 'shoulder', 'elbow', 'wrist', 'gripper']
        
        # Waypoint 1: Start position (at t=0)
        point1 = JointTrajectoryPoint()
        point1.positions = [math.pi, math.pi, math.pi, math.pi, math.pi]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        # Waypoint 2: Intermediate position (at t=2s)
        point2 = JointTrajectoryPoint()
        point2.positions = [math.pi/2, math.pi*1.2, math.pi, math.pi, math.pi]
        point2.time_from_start = Duration(sec=2, nanosec=0)
        
        # Waypoint 3: Final position (at t=4s)
        point3 = JointTrajectoryPoint()
        point3.positions = [math.pi*1.5, math.pi*0.8, math.pi*1.1, math.pi, math.pi]
        point3.time_from_start = Duration(sec=4, nanosec=0)
        
        trajectory.points = [point1, point2, point3]
        goal.trajectory = trajectory
        
        # Send goal
        self.get_logger().info('Sending trajectory...')
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        return future
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Executing... Actual: {feedback.actual.positions[0]:.2f}')

def main():
    rclpy.init()
    commander = TrajectoryCommander()
    
    # Send trajectory
    future = commander.send_trajectory()
    rclpy.spin_until_future_complete(commander, future)
    
    goal_handle = future.result()
    if goal_handle.accepted:
        print('Trajectory accepted')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(commander, result_future)
        print('Trajectory completed')
    else:
        print('Trajectory rejected')
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Example - Pick and Place

```python
#!/usr/bin/env python3
"""
Pick and place example using trajectory execution.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration
import math
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
        # Action client for trajectories
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/follow_joint_trajectory'
        )
        
        # Service clients for gripper
        self.gripper_open = self.create_client(Trigger, '/gripper_open')
        self.gripper_close = self.create_client(Trigger, '/gripper_close')
        
        # Wait for services
        self.traj_client.wait_for_server()
        self.gripper_open.wait_for_service()
        self.gripper_close.wait_for_service()
    
    def execute_trajectory(self, waypoints, times):
        """Execute trajectory with given waypoints and times."""
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['base', 'shoulder', 'elbow', 'wrist', 'gripper']
        
        for positions, t in zip(waypoints, times):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            trajectory.points.append(point)
        
        goal.trajectory = trajectory
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return True
        return False
    
    def gripper_command(self, open_gripper):
        """Open or close gripper."""
        client = self.gripper_open if open_gripper else self.gripper_close
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def pick_and_place_sequence(self):
        """Execute pick and place sequence."""
        self.get_logger().info('Starting pick and place...')
        
        # Define positions (radians)
        home = [math.pi, math.pi, math.pi, math.pi, math.pi]
        above_object = [math.pi, math.pi * 1.3, math.pi * 0.9, math.pi, math.pi]
        grasp_object = [math.pi, math.pi * 1.5, math.pi * 0.7, math.pi, math.pi]
        place_location = [math.pi * 1.5, math.pi * 1.3, math.pi * 0.9, math.pi, math.pi]
        
        # 1. Open gripper
        self.get_logger().info('Opening gripper...')
        self.gripper_command(True)
        time.sleep(1)
        
        # 2. Move above object
        self.get_logger().info('Moving above object...')
        self.execute_trajectory([home, above_object], [0, 2])
        time.sleep(0.5)
        
        # 3. Move down to grasp
        self.get_logger().info('Moving to grasp...')
        self.execute_trajectory([above_object, grasp_object], [0, 1.5])
        time.sleep(0.5)
        
        # 4. Close gripper
        self.get_logger().info('Closing gripper...')
        self.gripper_command(False)
        time.sleep(1)
        
        # 5. Lift object
        self.get_logger().info('Lifting object...')
        self.execute_trajectory([grasp_object, above_object], [0, 1.5])
        time.sleep(0.5)
        
        # 6. Move to place location
        self.get_logger().info('Moving to place location...')
        self.execute_trajectory([above_object, place_location], [0, 2])
        time.sleep(0.5)
        
        # 7. Open gripper
        self.get_logger().info('Releasing object...')
        self.gripper_command(True)
        time.sleep(1)
        
        # 8. Return home
        self.get_logger().info('Returning home...')
        self.execute_trajectory([place_location, home], [0, 2])
        
        self.get_logger().info('Pick and place complete!')

def main():
    rclpy.init()
    node = PickAndPlace()
    node.pick_and_place_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Monitoring Joint States

### Command-Line

```bash
# Echo all joint states
ros2 topic echo /joint_states

# Echo only positions
ros2 topic echo /joint_states/position

# View topic info
ros2 topic info /joint_states

# View message rate
ros2 topic hz /joint_states
# Should show ~50Hz
```

### Python Example - Joint State Monitor

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
    
    def joint_state_callback(self, msg):
        # Print joint states in a readable format
        print("\n" + "="*60)
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            pos_deg = math.degrees(pos)
            print(f"{name:10s}: {pos_deg:7.2f}° | {vel:6.2f} rad/s | {eff:6.2f} Nm")
        print("="*60)

def main():
    rclpy.init()
    monitor = JointStateMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Command-Line Examples

### Quick Commands

```bash
# List all topics
ros2 topic list

# List all services
ros2 service list

# List all actions
ros2 action list

# Get node info
ros2 node info /arm_controller

# View parameter values
ros2 param list /arm_controller
ros2 param get /arm_controller joint_names
```

### Recording and Playback

```bash
# Record joint states
ros2 bag record /joint_states

# Record all topics
ros2 bag record -a

# Play back recording
ros2 bag play <bagfile>

# Info about bag file
ros2 bag info <bagfile>
```

---

## Troubleshooting

### Arm not responding to commands

```bash
# Check if controller is running
ros2 node list | grep arm_controller

# Check if joint states are being published
ros2 topic hz /joint_states

# Check for errors in controller logs
docker logs pincherx100_ros2
```

### Commands are delayed or jerky

- Reduce `profile_velocity` in config for faster movement
- Check system load: `htop` (in container)
- Verify publish rate: `ros2 topic hz /joint_states`

### Joint limits being violated

```bash
# Check current limits
ros2 param get /arm_controller limits.base.min
ros2 param get /arm_controller limits.base.max

# Adjust limits in config/arm_config.yaml and restart controller
```

### Position commands not reaching target

- Verify position is within joint limits
- Check if trajectory is already executing
- Monitor `/arm_status` topic for errors

---

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
- [trajectory_msgs/JointTrajectory](https://docs.ros2.org/latest/api/trajectory_msgs/msg/JointTrajectory.html)
- [control_msgs/FollowJointTrajectory](https://docs.ros2.org/latest/api/control_msgs/action/FollowJointTrajectory.html)

---

## Tips and Best Practices

1. **Always start with home position**: Call `/go_home` before complex movements
2. **Use trajectories for smooth motion**: Better than rapid position commands
3. **Monitor joint states**: Watch for unexpected behavior
4. **Respect joint limits**: Configure appropriate limits for your workspace
5. **Gradual movements**: Start with small, slow movements when testing
6. **Emergency stop**: Keep Ctrl+C ready to stop the controller
7. **Test in simulation first**: If possible, test complex trajectories carefully




