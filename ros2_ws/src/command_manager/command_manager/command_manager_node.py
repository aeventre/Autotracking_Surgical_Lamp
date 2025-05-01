import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from surg_lamp_msgs.msg import UserCommand

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')

        self.mode = 0  # Default to Idle
        self.latest_orientation = None  # geometry_msgs/Quaternion
        self.latest_position = None     # geometry_msgs/Point
        self.latest_button_state = False  # Remote button state

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

        # Timer for preset poses
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Command Manager Node Initialized")

    def mode_callback(self, msg: Int32):
        self.mode = msg.data
        names = {
            0: 'Idle', 1: 'Track Remote', 2: 'Hold Position',
            3: 'Manual Joint Control', 4: 'Manual Pose Control',
            5: 'Preset Pose', 6: 'Light-Only Mode'
        }
        self.get_logger().info(
            f"System mode set to {self.mode} ({names.get(self.mode, 'Unknown')})")

    def orientation_callback(self, msg: QuaternionStamped):
        self.latest_orientation = msg.quaternion
        self._try_publish_remote_pose()

    def remote_pose_callback(self, msg: PoseStamped):
        self.latest_position = msg.pose.position
        self._try_publish_remote_pose()

    def remote_user_callback(self, msg: UserCommand):
        # Capture button state and reset
        self.latest_button_state = msg.button_state
        if msg.reset:
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0] * 5
            self.joint_command_pub.publish(zero_msg)
            self.get_logger().info(
                "Reset requested: published zero positions to /joint_commands")
        # Remote light-mode only in Track Remote
        if self.mode == 1:
            cmd = UserCommand()
            cmd.lightmode_remote_command = msg.lightmode_remote_command
            self.user_command_pub.publish(cmd)
            self.get_logger().info(
                f"Remote light-mode override: {msg.lightmode_remote_command}")

    def _try_publish_remote_pose(self):
        # Only in Track Remote when button is pressed
        if (self.mode == 1 and self.latest_orientation and
                self.latest_position and self.latest_button_state):
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'base_link'
            ps.pose.position = self.latest_position
            ps.pose.orientation = self.latest_orientation
            self.goal_pose_pub.publish(ps)
            self.get_logger().info(
                "Published remote goal pose to /goal_pose (button pressed)")

    def light_mode_callback(self, msg: Int32):
        # GUI light-mode only when not tracking remote
        if self.mode != 1:
            cmd = UserCommand()
            cmd.lightmode_remote_command = msg.data
            self.user_command_pub.publish(cmd)
            self.get_logger().info(f"GUI light-mode: {msg.data}")
        else:
            self.get_logger().debug(
                "Ignored GUI light-mode during remote tracking")

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

    def publish_preset_pose(self):
        if self.mode == 5:
            joint_msg = Float64MultiArray()
            joint_msg.data = [0.0, 45.0, 90.0, 90.0, 90.0]
            self.joint_command_pub.publish(joint_msg)
            self.get_logger().info("Preset pose command published")

    def timer_callback(self):
        self.publish_preset_pose()


def main(args=None):
    rclpy.init(args=args)
    node = CommandManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
