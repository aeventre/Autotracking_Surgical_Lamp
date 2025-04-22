import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from surg_lamp_msgs.msg import UserCommand

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')

        self.mode = 0  # Default to Idle

        # Publisher interfaces
        self.joint_command_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.user_command_pub = self.create_publisher(UserCommand, 'user_command', 10)

        # Subscriptions
        self.create_subscription(Int32, 'system_mode', self.mode_callback, 10)
        self.create_subscription(Int32, 'light_mode_cmd', self.light_mode_callback, 10)
        self.create_subscription(PoseStamped, 'manual_pose_input', self.manual_pose_callback, 10)
        self.create_subscription(Float64MultiArray, 'manual_joint_input', self.manual_joint_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose_input', self.auto_pose_callback, 10)

        self.get_logger().info("Command Manager Node Initialized")

    def mode_callback(self, msg: Int32):
        self.mode = msg.data
        mode_name = {
            0: "Idle",
            1: "Track Remote",
            2: "Hold Position",
            3: "Manual Joint Control",
            4: "Manual Pose Control",
            5: "Preset Pose",
            6: "Light-Only Mode"
        }.get(self.mode, "Unknown")
        self.get_logger().info(f"System mode set to {self.mode} ({mode_name})")

    def light_mode_callback(self, msg: Int32):
        if self.mode in [1, 6, 3, 4, 5]:  # Allow light changes in most active modes
            user_cmd = UserCommand()
            user_cmd.light_mode = msg.data
            self.user_command_pub.publish(user_cmd)
            self.get_logger().info(f"Published light mode: {msg.data}")
        else:
            self.get_logger().info("Light mode ignored due to current mode.")

    def auto_pose_callback(self, msg: PoseStamped):
        if self.mode == 1:
            self.goal_pose_pub.publish(msg)
            self.get_logger().info("Auto pose forwarded to /goal_pose")

    def manual_pose_callback(self, msg: PoseStamped):
        if self.mode == 4:
            self.goal_pose_pub.publish(msg)
            self.get_logger().info("Manual pose published to /goal_pose")

    def manual_joint_callback(self, msg: Float64MultiArray):
        if self.mode == 3:
            if len(msg.data) >= 5:
                self.joint_command_pub.publish(msg)
                self.get_logger().info("Manual joint command published to /joint_commands")
            else:
                self.get_logger().warn("Manual joint input too short; requires 5 values.")

    def publish_preset_pose(self):
        # Hardcoded pose for mode 5
        joint_msg = Float64MultiArray()
        joint_msg.data = [0.0, 45.0, 90.0, 90.0, 90.0]
        self.joint_command_pub.publish(joint_msg)
        self.get_logger().info("Preset pose command published")

    def timer_callback(self):
        if self.mode == 5:
            self.publish_preset_pose()

def main(args=None):
    rclpy.init(args=args)
    node = CommandManager()

    # Timer for periodic preset pose publishing (optional)
    node.create_timer(1.0, node.timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
