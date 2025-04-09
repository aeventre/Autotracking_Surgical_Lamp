# ROS2 Overview

## System Architecture
The system consists of **several ROS2 packages**, each responsible for a different function. The software runs on a **Raspberry Pi 5**, which communicates with microcontrollers and sensors to control the lamp.

---

## ROS2 Packages

### 1. **System Management & User Interaction**
| Package | Description |
|---------|------------|
| `command_manager` | Listens to user input commands and determines what nodes need to be running depending on the selected mode of the surgical lamp. |
| `remote_pub` | Listens to commands and data from the remote control. Publishes user commands and raw IMU data. |
| `debug_logger` | Collects and publishes system diagnostics, errors, and debug information for troubleshooting. |
| `gui_interface` | Provides a graphical user interface for monitoring system state and sending manual commands. |

### 2. **Sensor Processing & State Estimation**
| Package | Description |
|---------|------------|
| `attitude_estimator` | Subscribes to raw IMU data, applies filters, and publishes attitude as a unit quaternion. |
| `position_estimator` | Processes camera data to estimate the full 6D pose of the target. Computes necessary tracking offsets for the lamp’s end-effector. |
| `vision_processing` | Enhances target tracking by applying image filtering, feature detection, and OpenCV-based object recognition. |
| `time_synchronizer` | Ensures sensor data (IMU, cameras) and control commands are synchronized with accurate timestamps. |

### 3. **Motion Planning & Execution**
| Package | Description |
|---------|------------|
| `moveit_planner` | Uses MoveIt2 for high-level motion planning, inverse kinematics, collision avoidance, and trajectory generation. |
| `motion_executor` | Executes planned trajectories by sending joint commands to `controller_coms`. Handles real-time control adjustments to ensure accurate motion. |
| `controller_coms` | Sends and receives data over serial communications to the MCUs controlling the actuators. Sends joint commands and receives joint angle feedback for closed-loop control. |

### 4. **Coordinate Transformations & Data Synchronization**
| Package | Description |
|---------|------------|
| `tf_broadcaster` | Publishes transformations between coordinate frames (base, joints, end-effector, and target) using TF2. |
| `tf_listener` | Subscribes to TF2 transforms to determine frame relationships for motion planning and control. |

### 5. **System Safety & Recovery** 
| Package | Description |
|---------|------------|
| `fault_detector` | Monitors system health and detects failures (e.g., lost tracking, joint errors). Can trigger safety shutdowns or recovery procedures. |

### 6. System Simulation & Testing
| Package | Description |
|---------|------------|
| `surg_bot_sim` | Provides a physics-based simulation environment using Gazebo or Isaac Sim for testing the lamp’s motion planning and control strategies before real-world implementation. |

### 7. **System Description & Custom Message Types**
| Package | Description |
|---------|------------|
| `surg_bot_description` | Contains URDF and mesh files to create an accurate model of the surgical lamp. Used for visualization and simulation. |
| `surg_bot_msgs` | Contains custom message types to be used by all other packages for inter-package communication. |

---

## ROS2 Topics

The following ROS2 topics facilitate communication between nodes:

| Topic Name | Message Type | Publisher | Subscriber(s) | Description |
|------------|-------------|-----------|---------------|-------------|
| `/user_commands` | `std_msgs/String` | `remote_pub` | `command_manager` | Sends user-selected commands from the remote control or GUI. |
| `/system_status` | `std_msgs/String` | `command_manager` | `gui_interface` | Provides updates on the current system mode and state. |
| `/debug_log` | `std_msgs/String` | `debug_logger` | `gui_interface` | Publishes system logs for debugging and monitoring. |
| `/imu/data_raw` | `sensor_msgs/Imu` | `remote_pub` | `attitude_estimator` | Raw IMU data from the remote control. |
| `/imu/attitude` | `geometry_msgs/Quaternion` | `attitude_estimator` | `moveit_planner`, `position_estimator` | Filtered IMU data converted into a quaternion representing orientation. |
| `/camera/image_raw` | `sensor_msgs/Image` | `Cam-1`, `Cam-2` | `vision_processing` | Raw image frames from ESP32 cameras. |
| `/camera/processed` | `sensor_msgs/Image` | `vision_processing` | `position_estimator` | Image with feature detection or object recognition applied. |
| `/target_position` | `geometry_msgs/Point` | `position_estimator` | `moveit_planner`, `tf_broadcaster` | The estimated XYZ position of the target relative to the lamp base. |
| `/target_pose` | `geometry_msgs/PoseStamped` | `position_estimator` | `moveit_planner`, `tf_broadcaster` | Full 6D pose (position + orientation) of the target. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `position_estimator` | `moveit_planner` | Desired pose for the lamp end-effector based on tracking data. |
| `/planned_trajectory` | `trajectory_msgs/JointTrajectory` | `moveit_planner` | `motion_executor` | The computed trajectory from MoveIt2 for lamp movement. |
| `/joint_commands` | `sensor_msgs/JointState` | `motion_executor` | `controller_coms` | Commands to move joints to specific angles. |
| `/joint_feedback` | `sensor_msgs/JointState` | `controller_coms` | `motion_executor` | Real-time joint positions received from the microcontrollers. |
| `/lamp_status` | `std_msgs/String` | `controller_coms` | `command_manager`, `gui_interface` | Provides updates on lamp motor status. |
| `/tf` | `tf2_msgs/TFMessage` | `tf_broadcaster` | `moveit_planner`, `motion_executor` | Publishes transformations between all system frames. |
| `/error_status` | `std_msgs/String` | `fault_detector` | `command_manager`, `gui_interface` | Reports any detected errors or failures. |
| `/safety_shutdown` | `std_msgs/Bool` | `fault_detector` | `controller_coms` | Triggers a safety stop if an error is detected. |

---

## Getting Started

1. **Install ROS2 (Jazzy or compatible version).**
2. **Clone the repository and build the workspace:**
   ```bash
   mkdir -p ~/surgical_lamp_ws/src
   cd ~/surgical_lamp_ws/src
   git clone <repository_url>
   cd ~/surgical_lamp_ws
   colcon build --symlink-install
   ```
3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```
4. **Launch the system:**
   ```bash
   ros2 launch command_manager command_manager.launch.py
   ```

