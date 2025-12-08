# EECE 5554 Final Project — Team Quaternions
## Semantic Mobile Manipulation with TurtleBot 4

---

## Project Overview

This project implements a semantic mobile manipulation pipeline on a TurtleBot 4 equipped with an OAK-D Pro depth camera. The robot autonomously performs the following sequence:

- Undocks from its charging station  
- Detects a target object using YOLO-based perception  
- Uses SLAM Toolbox and Nav2 to navigate toward the target  
- Executes a filtered approach maneuver using an alpha filter  
- Stops at a defined interaction distance for a robotic arm  
- Allows the arm to pick up the detected object using either manual control or a predefined routine  

This integrates perception, mapping, navigation, motion control, and manipulation into one unified system.

---

## Achievement Level

The project achieved **Level 2 (“Target Goal”)** as specified in the team contract.

### Level 2 Capabilities Demonstrated

- Autonomous undocking  
- Real-time object detection using YOLO and depth fusion    
- Approach controller with alpha-filter smoothing  
- Final pose alignment for robotic arm manipulation  
- Successful robotic arm pick-up (manual or predefined algorithm)  

The robot autonomously locates an object, navigates to it, approaches it, and positions itself for a successful pick-up.

---

## Repository Structure

```
project_root/
│
├── main_ws/
│   ├── config/
│   │   ├── oakd_pro.yaml
│   │   └── fast_slam.yaml
│   │
│   ├── notes/
│   │   └── *.md / *.txt
│   │
│   ├── src/
│   │   ├── object_location/
│   │   ├── object_location_interfaces/
│   │   ├── mission_control/
│   │   ├── robotic_arm/
│   │   └── ...
│   │
│   ├── requirements.txt
│   └── ...
│
├── scripts/
│   ├── front_end/
│   │   └── ...
│   ├── object_detection/
│   │   └── ...
│   ├── setup_env.sh
│   ├── install_binaries.sh
│   └── run_live_demo.sh
│
├── demo/
│   └── README.md
│
└── README.md
```

---

## Demo Video

A full demonstration of the Level 2 pipeline — including undocking, object detection, navigation, approach, and arm pick-up — is available online.

The link to the video is stored in the following file:

```
demo/README.md
```

Open that file to access the Google Drive URL containing the recorded live demo.

---

## Installation Instructions

### 1. Install System Dependencies (ROS Jazzy + Nav2 + TurtleBot4)

From the project root:

```bash
./scripts/install_binaries.sh
```

This installs:

- ROS 2 Jazzy  
- Navigation2  
- nav2-bringup  
- nav2-minimal-tb*  
- nav2-route  
- TurtleBot4 packages  
- teleop-twist-keyboard  
- rosbridge-server  

---

### 2. Set Up Virtual Environment and Python Dependencies

```bash
./scripts/setup_env.sh
```

This script:

- Creates a `venv/` folder  
- Installs dependencies from `requirements.txt`  
- Installs additional packages:  
  - typeguard  
  - ultralytics  
  - opencv-python  
  - scipy  
  - message-filters  
- Opens a new terminal and runs a clean `colcon build` without activating the venv  

---

## How to Launch and Run the Project

After installing the required binaries and setting up the environment using the provided scripts, the entire system can be launched with a single command.

From the project root:

```bash
./scripts/run_live_demo.sh
```

This script automatically launches:

- Perception pipeline (YOLO + depth processing)
- SLAM or map loading (depending on configuration)
- Nav2 navigation stack
- Approach controller
- Any required supporting nodes

No additional manual launch steps are required.

---

## Using the System After Launch

After running the live demo script, the full perception–navigation–approach pipeline will start automatically. Please follow the steps below to operate the system correctly:

### 1. Wait for the TurtleBot to Undock
The robot will automatically undock from its charging station.  
Do **not** issue commands or attempt to interact with the system until undocking is complete.

### 2. Ensure the Target Object Is Visible to the Camera
The object you want the robot to approach must be:

- Within the forward field of view of the OAK-D Pro camera  
- At a reasonable distance for YOLO to detect  
- Belonging to one of the YOLO class labels included in the model  

### 3. Click the "Pick Up Bottle" Button
The current user interface includes a single action button labeled **“Pick Up Bottle.”**  
Clicking this button instructs the system to:

1. Look for the YOLO class associated with a bottle  
2. Select the closest detected bottle  
3. Navigate toward it using the SLAM + Nav2 pipeline  
4. Execute the approach controller to position the robot for pickup  

### 4. Important Limitations
- The system **only attempts to approach objects belonging to YOLO classes.**  
- The **target object class is currently hardcoded as “bottle.”**  
  - To pick up another object (e.g., cup, box), the class must be manually changed in the source code.  
- This is a temporary limitation and will be updated in the future.


## Known Limitations and Future Work

- IK (Inverse Kinematics) for the arm is incomplete  
- Additional research needed to finalize IK consistency and reliability  
- Hand–eye calibration required to improve pick-up accuracy  
- Potential use of an AprilTag mounted on the arm for automatic arm posture recognition  
- Approach controller currently supports only one target item; multi-item selection and prioritization will be added later  
- Post-pickup behavior is not implemented; next step is to deliver the item to a specified location using a **custom navigation algorithm**, as Nav2’s action server can be unpredictable in dynamic conditions  
- The frontend interface will be extended to include **multiple buttons**, each corresponding to a different object class.  
Users will be able to select targets dynamically, without modifying the code.

---

## Team Members

- David Ross  
- Rongxuan Zhang  
- Zesong Guo  
- Junrui Xu  
