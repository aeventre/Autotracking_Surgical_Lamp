from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
 
 
def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.
 
    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    # Start arm controller
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')

 
    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
 
    # Register event handlers for sequencing
    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))
 

 
    # Create the launch description and populate
    ld = LaunchDescription()
 
    # Add the actions to the launch description in sequence
    ld.add_action(start_joint_state_broadcaster_cmd)
    ld.add_action(load_joint_state_broadcaster_cmd)
  #  ld.add_action(start_arm_controller_cmd)
 
    return ld