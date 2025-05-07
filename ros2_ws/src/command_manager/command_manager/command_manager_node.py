import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from surg_lamp_msgs.msg import UserCommand

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')

        self.mode = 0  # Default to Idle
        self.latest_orientation = None  # geometry_msgs/Quaternion
        self.latest_position = None     # geometry_msgs/Point
        self.latest_button_state = False  # Remote button state
        
        self.previous_light_mode = 0  # Stores user-selected mode (from GUI or remote)
        self.planning_active = False  # Tracks if we're currently expecting planning result


        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        self.user_command_pub = self.create_publisher(
            UserCommand, 'user_command', 10)

        # Subscriptions
        self.create_subscription(Int32, 'system_mode',
                                 self.mode_callback, 10)
        self.create_subscription(QuaternionStamped, 'remote_orientation',
                                 self.orientation_callback, 10)
        self.create_subscription(PoseStamped, 'remote_pose',
                                 self.remote_pose_callback, 10)
        self.create_subscription(Int32, 'light_mode_cmd',
                                 self.light_mode_callback, 10)
        self.create_subscription(PoseStamped, 'manual_pose_input',
                                 self.manual_pose_callback, 10)
        self.create_subscription(Float64MultiArray, 'manual_joint_input',
                                 self.manual_joint_callback, 10)
        self.create_subscription(UserCommand, 'remote_user_command',
                                 self.remote_user_callback, 10)
        self.create_subscription(Bool, 'planning_status', self.planning_status_callback, 10)

        self.create_subscription(String, 'demo_sequence_name', self.demo_sequence_callback, 10)
        

        # Timer for preset poses
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Command Manager Node Initialized")
    

        self.current_demo_index = 0
        self.demo_interval = 5.0  # seconds between poses
        self.last_demo_time = self.get_clock().now()
        
        self.demo_sequences = {
            "Basic": {
                "interval": 4.0,
                "poses": [
                    ([0.0, 0.0, 0.0, 0.0, 0.0], 0),
                    ([45.0, -45.0, 0.0, 0, 0], 0),
                    ([-45.0, 45.0, 0.0, 0, 0], 0),
                    ([0.0, 0.0, -17, 0.0, 0.0], 3),
                    ([0.0, 0.0, 0.0, -90, -90], 3),
                    ([0.0, 0.0, 0.0, 90, 90], 0),
                ]
            },
            "Spin": {
                "interval": 1.0,
                "poses": [
                    ([0.0, 0.0, 0.0, 0.0, 90], 3),
                    ([90.0, -90, 0.0, 0.0, 90], 3),
                    ([180.0, -180, 0.0, 0.0, 90], 3),
                    ([-90, 90, 0.0, 0.0, 90], 3),
                ]
            }
        }

        self.active_demo_name = list(self.demo_sequences.keys())[0]  # first available demo
        self.current_demo_index = 0



    def mode_callback(self, msg: Int32):
        self.mode = msg.data
        names = {
            0: 'Idle', 1: 'Track Remote', 2: 'Zeroing Mode',
            3: 'Manual Joint Control', 4: 'Manual Pose Control',
            5: 'Light-Only Mode', 6: 'System Demo'
        }
        self.get_logger().info(
            f"System mode set to {self.mode} ({names.get(self.mode, 'Unknown')})")

        if self.mode == 2:
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0] * 5
            self.joint_command_pub.publish(zero_msg)
            self.get_logger().info("Zeroing Mode: Sent all joints to 0 degrees")



    def orientation_callback(self, msg: QuaternionStamped):
        self.latest_orientation = msg.quaternion
        self._try_publish_remote_pose()

    def remote_pose_callback(self, msg: PoseStamped):
        self.latest_position = msg.pose.position
        self._try_publish_remote_pose()

    def remote_user_callback(self, msg: UserCommand):
        self.latest_button_state = msg.button_state

        # Handle reset request
        if msg.reset:
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0] * 5
            self.joint_command_pub.publish(zero_msg)
            self.get_logger().info("Reset requested: published zero joint commands")

        # Light mode override (only if in remote tracking mode)
        if self.mode == 1:
            cmd = UserCommand()
            cmd.button_state = False
            cmd.light_mode = msg.light_mode
            cmd.reset = False
            self.user_command_pub.publish(cmd)
            self.get_logger().info(f"Remote light-mode override: {msg.light_mode}")

    def light_mode_callback(self, msg: Int32):
        # Only allow GUI light mode when not tracking remote
        if self.mode != 1:
            cmd = UserCommand()
            cmd.button_state = False
            cmd.light_mode = msg.data
            cmd.reset = False
            self.user_command_pub.publish(cmd)
            self.get_logger().info(f"GUI light-mode command: {msg.data}")
        else:
            self.get_logger().debug("Ignored GUI light-mode during remote tracking")


    def _try_publish_remote_pose(self):
        if (self.mode == 1 and self.latest_orientation and
                self.latest_position and self.latest_button_state):
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'base_link'
            ps.pose.position = self.latest_position
            ps.pose.orientation = self.latest_orientation
            self.goal_pose_pub.publish(ps)
            self.get_logger().info("Published remote goal pose to /goal_pose (button pressed)")

            self.planning_active = True
            self._set_light_mode(2)  # Set light mode to 2 during planning
            self.latest_button_state = False



    def manual_pose_callback(self, msg: PoseStamped):
        if self.mode == 4:
            self.goal_pose_pub.publish(msg)
            self.get_logger().info("Manual pose published to /goal_pose")

    def manual_joint_callback(self, msg: Float64MultiArray):
        if self.mode == 3:
            if len(msg.data) >= 5:
                self.joint_command_pub.publish(msg)
                self.get_logger().info(
                    "Manual joint command published to /joint_commands")
            else:
                self.get_logger().warn(
                    "Manual joint input too short; requires 5 values.")

    def timer_callback(self):
        if self.mode == 6:  # System Demo
            now = self.get_clock().now()
            if (now - self.last_demo_time).nanoseconds / 1e9 >= self.demo_interval:
                self.publish_demo_pose()
                self.last_demo_time = now

        
    def planning_status_callback(self, msg: Bool):
        if not self.planning_active:
            return  # Ignore if we didn't initiate planning

        if msg.data:  # planning succeeded
            self._set_light_mode(self.previous_light_mode)
            self.get_logger().info("Planning succeeded, restoring previous light mode")
        else:  # planning failed
            self._set_light_mode(1)
            self.get_logger().warn("Planning failed, light mode set to 1")

        self.planning_active = False
        
    def _set_light_mode(self, mode):
        cmd = UserCommand()
        cmd.button_state = False
        cmd.reset = False
        cmd.light_mode = mode
        self.user_command_pub.publish(cmd)
        
    def publish_demo_pose(self):
        sequence_data = self.demo_sequences.get(self.active_demo_name, {})
        poses = sequence_data.get("poses", [])

        if not poses:
            self.get_logger().warn(f"No poses in sequence '{self.active_demo_name}'")
            return

        # Get the next pose and light mode
        angles, light_mode = poses[self.current_demo_index]

        # Publish joint angles
        joint_msg = Float64MultiArray()
        joint_msg.data = angles
        self.joint_command_pub.publish(joint_msg)

        # Set light mode
        self._set_light_mode(light_mode)

        # Log info
        self.get_logger().info(
            f"Demo pose {self.current_demo_index + 1} in '{self.active_demo_name}' sent (light mode {light_mode})")

        # Update index for next pose
        self.current_demo_index = (self.current_demo_index + 1) % len(poses)

        # Update interval in case it changed (optional, for dynamic interval updates)
        self.demo_interval = sequence_data.get("interval", self.demo_interval)


    def demo_sequence_callback(self, msg: String):
        if msg.data in self.demo_sequences:
            self.active_demo_name = msg.data
            self.current_demo_index = 0
            self.demo_interval = self.demo_sequences[msg.data]["interval"]
            self.get_logger().info(f"Demo sequence set to '{msg.data}' with interval {self.demo_interval:.1f}s")
        else:
            self.get_logger().warn(f"Unknown demo sequence: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
