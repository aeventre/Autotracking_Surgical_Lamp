# AutoTracking Surgical Robot lamp

<img src="https://github.com/user-attachments/assets/09352c4d-7256-4a17-90d0-0a140b2da211" alt="Alt Text" width="400" height="400">

## Table of Contents
- [AutoTracking Surgical Robot lamp](#autotracking-surgical-robot-lamp)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Background](#background)
  - [Acknowledgments](#acknowledgments)
- [System Overview](#system-overview)
- [Software Design](#software-design)
  - [Setup](#setup)
  - [Package Overview](#package-overview)
    - [`command_manager`](#command_manager)
    - [`lamp_gui`](#lamp_gui)
    - [`remote_tracker`](#remote_tracker)
    - [`camera_calibration`](#camera_calibration)
    - [`goal_pose_executor`](#goal_pose_executor)
    - [`controller_coms`](#controller_coms)
    - [`surg_bot_description`](#surg_bot_description)
    - [`surg_bot_moveit_config`](#surg_bot_moveit_config)
  - [Launch Configurations](#launch-configurations)
- [Electrical Design](#electrical-design)
- [CAD](#cad)
  - [3D Printed Components of Surgical Robot](#3d-printed-components-of-surgical-robot)
  - [Sensors and Servos](#sensors-and-servos)
  - [Bill of materials for Surgical Robot](#bill-of-materials-for-surgical-robot)
  - [Bill of Materials for Surgical Robot Base](#bill-of-materials-for-surgical-robot-base)

## Overview

## Background
## Acknowledgments

# System Overview
<img src="https://github.com/user-attachments/assets/09f3d186-7873-4ec3-ac99-0a97c6b37549" alt="Alt Text" width="600" height="300">  

# Software Design

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

## Package Overview

![software_architecture](https://github.com/user-attachments/assets/fd5ed8b2-e8f9-41f9-8dcc-f4682f1d20a9)


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
- Camera extrinsics from `camera_calibration`
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

## Launch Configurations

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

# Electrical Design

# CAD



## 3D Printed Components of Surgical Robot
PLA was the material used to print the prototype. ABS and PETG would be good options to use as well
| Name         | Photo                                                                                            | Link (coming soon)       | 
| ------------ | ------------------------------------------------------------------------------------------------ | ------------- |
| Link 1       | ![Picture3](https://github.com/user-attachments/assets/87dfc9d8-75c0-4b11-83c7-dda14b3a484b)     | LINK          |
| Joint 1      | ![Picture4](https://github.com/user-attachments/assets/a68e6df6-cf14-4317-85c4-67fdfe561876)     | LINK          |
| Link 2       | ![Picture5](https://github.com/user-attachments/assets/ce58bc68-0d48-4a11-a220-372c4e131fa6)     | LINK          |
| Link 3       | ![Picture6](https://github.com/user-attachments/assets/a6944806-5902-48d9-97da-c85d450341f3)     | LINK          |
| Link 4       | ![Picture7](https://github.com/user-attachments/assets/20489ea1-b9c3-42f5-be44-d0572020ce89)     | LINK          |
| Headlight    | ![Picture8](https://github.com/user-attachments/assets/53586ea6-2c06-40ee-9ba3-b020ba184a41)     | LINK          |

## Sensors and Servos
Below is a list of sensors and servos being used 
- Lidar
- 2 ESP32 Cameras
- IMU
- Time of Flight Sensors
- 180 degrees servo

## Bill of materials for Surgical Robot

## Bill of Materials for Surgical Robot Base

Provisional Patent 
