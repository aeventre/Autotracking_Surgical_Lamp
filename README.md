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
An open-source ROS 2 and MoveIt 2 surgical lamp capable of autonomous real-time tracking of a handheld remote using monocular vision and 3D pose estimation. 

## Background
This project aims to develop a solution to the challenges posed by modern surgical lamps,
particularly focused on the ceiling-mounted lamps. The proposed solution is a responsive
surgical lamp that uses real-time tracking and automated movement to adjust its lighting
position. This approach minimizes manual interaction, reduces delays and allows support staff
to remain focused on higher priority clinical responsibilities

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

## Remote Design 
<img width="732" height="534" alt="image" src="https://github.com/user-attachments/assets/8518f90e-6a1e-4195-b4e3-009d1ec1a07b" />

## PCB Design 
<img width="922" height="628" alt="image" src="https://github.com/user-attachments/assets/91fa7ea2-843f-440c-808c-bc5cea24d7ab" />






# Pictures
<img width="632" height="545" alt="image" src="https://github.com/user-attachments/assets/a43fd7c9-dbca-49dd-9ab8-42eebba4224f" />
<img width="702" height="918" alt="image" src="https://github.com/user-attachments/assets/00eb7c82-7a03-440d-8ad5-3878b8ce05df" />
<img width="436" height="820" alt="image" src="https://github.com/user-attachments/assets/e57f7218-d5af-47de-af74-344857514425" />



All links and joints links are avaiable to download in the CAD section as .STL and .SLDPRT

## 3D Printed Components of Surgical Robot
ABS was the material used to print the prototype. ABS and PETG would be good options to use as well

## Sensors and Servos 
Below is a list of sensors and motors being used 
- 160 KG Servos 
- 20 KG Servos
- Adafruit 9-DOF Orientation IMU Fusion Sensor
- Magnetic Encoder 
- NEMA 17
- Bluetooth/WiFi module
- Motor Drivers
- Orbbec 335 Cameras 

## Bill of materials for Surgical Robot
- Full HD webcam
- 4 channels IIC I2C logic level converter
- ESP32 Cam
- RS-485 MAX485 Transceiver
- 160 KG Servo
- Nema 17 Stepper
- Taidacent 12 wire 5A Slip Rings 
- USB to RS-485 Converter
- AS5600 Magnetic Encoder 
- Magnets 
- Laser Diodes
- LED Circle Array
- Raspberry Pi 5 8GB

## Bill of Materials for Surgical Robot Base
