import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from surg_lamp_msgs.msg import LampJointCommands

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')

        # Internal state
        self.mode = 0  # Default to Idle
        self.MODES = {
            0: "Idle",
            1: "Track Remote",
            2: "Hold Position",
            3: "Manual Control",
            4: "Calibration",
            5: "Preset Pose",
            6: "Light-Only Mode"
        }

        # Publishers
        self.joint_command_pub = self.create_publisher(LampJointCommands, 'joint_commands', 10)
        self.light_mode_pub = self.create_publisher(Int32, 'light_mode', 10)

        # Subscriptions
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Int32, 'light_mode_cmd', self.light_mode_callback, 10)
        self.create_subscription(Int32, 'system_mode', self.mode_callback, 10)

        self.get_logger().info("Command Manager Node Initialized")

    def mode_callback(self, msg: Int32):
        self.mode = msg.data
        mode_name = self.MODES.get(self.mode, "Unknown")
        self.get_logger().info(f"Mode changed to {self.mode} ({mode_name})")

    def goal_pose_callback(self, msg: PoseStamped):
        if self.mode == 1:  # Track Remote
            # Eventually compute IK here
            joint_msg = LampJointCommands()
            joint_msg.joint0 = 0.0
            joint_msg.joint1 = 0.0
            joint_msg.joint2 = 90.0
            joint_msg.joint3 = 90.0
            joint_msg.joint4 = 90.0
            self.joint_command_pub.publish(joint_msg)
        elif self.mode == 2:  # Hold Position
            # Don’t publish new commands
            pass
        elif self.mode == 5:  # Preset Pose
            # Could hardcode pose or load from param
            joint_msg = LampJointCommands()
            joint_msg.joint0 = 10.0
            joint_msg.joint1 = 45.0
            joint_msg.joint2 = 90.0
            joint_msg.joint3 = 90.0
            joint_msg.joint4 = 90.0
            self.joint_command_pub.publish(joint_msg)
        # Add more behaviors as you expand!

    def light_mode_callback(self, msg: Int32):
        if self.mode in [1, 6]:  # Track Remote or Light-Only Mode
            self.light_mode_pub.publish(msg)
        else:
            self.get_logger().info("Light mode ignored due to current system mode")

def main(args=None):
    rclpy.init(args=args)
    node = CommandManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
