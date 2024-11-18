#!/bin/bash
# Single script to launch the surg_bot with Gazebo and ROS 2 Controllers
 
cleanup() {
    echo "Cleaning up..."
    echo "Restarting ROS 2 daemon..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}
 
# Set up cleanup trap
trap 'cleanup' SIGINT
 
echo "Launching Gazebo simulation..."
ros2 launch surg_bot_gazebo surg_bot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.05 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0