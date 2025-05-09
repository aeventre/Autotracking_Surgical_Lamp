# Autotracking Surgical Lamp — ROS 2 Workspace

This ROS 2 workspace contains all packages required for the Auto-Tracking Surgical Lamp system. The lamp uses 3D pose tracking of a handheld remote (via green ball detection) to plan and execute motions in real-time, with fallback modes for manual GUI control and demo sequences. The system is built using ROS 2 Jazzy and MoveIt 2.

## Setup

To build and source the workspace:

```bash
cd /path/to/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Packages

### `command_manager`

Manages the system's behavior based on user input. It switches between modes such as idle, manual control, tracking, zeroing, and demo.

**Subscriptions**
- `/system_mode` (`std_msgs/Int32`)
- `/remote_pose` (`geometry_msgs/PoseStamped`)
- `/remote_orientation` (`geometry_msgs/QuaternionStamped`)
- `/user_command` (`surg_lamp_msgs/UserCommand`)

**Publications**
- `/goal_pose` (`geometry_msgs/PoseStamped`)
- `/joint_commands` (`std_msgs/Float64MultiArray`)
- `/user_command` (`surg_lamp_msgs/UserCommand`)

---

### `lamp_gui`

PyQt5-based graphical user interface for interacting with the lamp.

**Features**
- Dropdown menu for system mode selection
- Sliders for manual joint control
- Buttons for triggering demo modes or resets
- Status label for system feedback

**Publications**
- `/system_mode` (`std_msgs/Int32`)
- `/joint_commands` (`std_msgs/Float64MultiArray`)
- `/user_command` (`surg_lamp_msgs/UserCommand`)

---

### `remote_tracker`

Tracks a green marker (ball) on a handheld remote using OpenCV and depth cameras. Estimates the 3D position and uses IMU orientation to compute the full pose.

**Inputs**
- RGB and depth images from stereo cameras
- Camera intrinsics and extrinsics from `camera_calibration`
- IMU orientation from the remote

**Outputs**
- `/remote_pose` (`geometry_msgs/PoseStamped`)
- `/remote_orientation` (`geometry_msgs/QuaternionStamped`)

---

### `camera_calibration`

Contains stereo camera calibration and extrinsic transform files.

**Contents**
- `camera_01.yaml`, `camera_02.yaml`: intrinsics for each camera
- `camera_01_to_base.yaml`, `camera_02_to_base.yaml`: transformations from camera to robot base frame

Used by the `remote_tracker` for accurate triangulation and transformation into the base frame.

---

### `goal_pose_executor`

Plans and executes motions from the current pose to the goal pose using MoveIt 2.

**Inputs**
- `/goal_pose` (`geometry_msgs/PoseStamped`)

**Behavior**
- Uses `MoveGroupInterface` to plan paths
- Sends joint angles to `/joint_commands`
- Reports failure if planning is unsuccessful

---

### `controller_coms`

Handles RS-485 serial communication with two microcontrollers that control the lamp's motors and lighting.

**Responsibilities**
- Sends joint angle commands to MCU1 and MCU2
- Receives and parses motor feedback
- Manages half-duplex communication (send then receive)
- Relays light mode and button state

**Serial Protocol**
- `mcu1`: `<joint1,joint2,joint3,joint4,lightmode>`
- `mcu2`: `<joint0>`
- Half-duplex format: send commands, then wait for and parse response

---

### `surg_bot_description`

URDF/Xacro model of the surgical lamp.

**Includes**
- Link and joint definitions
- Joint limits and axes
- Visual and collision meshes
- Base frame and end-effector frame definitions

Used for visualization and planning.

---

### `surg_bot_moveit_config`

MoveIt 2 configuration for the surgical lamp.

**Includes**
- Kinematics solvers
- OMPL planning pipeline
- SRDF and robot semantic info
- RViz and MoveGroup launch support

---

## Launching Nodes

```bash
# GUI
ros2 run lamp_gui lamp_gui

# Command manager
ros2 run command_manager command_manager

# Remote tracker
ros2 run remote_tracker remote_tracker_node

# Goal pose executor
ros2 run goal_pose_executor goal_pose_executor_node

# Controller communication
ros2 run controller_coms controller_coms_node
```

Ensure the cameras and IMU are connected, and the camera calibration YAMLs are properly configured.
