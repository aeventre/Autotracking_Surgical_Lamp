import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from surg_lamp_msgs.msg import LampJointCommands, LampCurrentAngles

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver_2d')

        # Declare and retrieve link lengths
        self.declare_parameter('L1', 0.32)
        self.declare_parameter('L2', 0.39)
        self.L1 = self.get_parameter('L1').value
        self.L2 = self.get_parameter('L2').value

        # Store current joint angles
        self.current_theta0 = 0.0
        self.current_theta1 = 0.0

        # Subscriptions
        self.create_subscription(Point, 'wrist_position', self.ik_callback, 10)
        self.create_subscription(LampCurrentAngles, 'current_angles', self.angles_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(LampJointCommands, 'joint_commands', 10)

        self.get_logger().info("IK Solver 2D Node with Camera Transform Initialized")

    def angles_callback(self, msg):
        self.current_theta0 = math.radians(msg.joint_0)
        self.current_theta1 = math.radians(msg.joint_1)

    def radians_to_deg360(self, rad):
        deg = math.degrees(rad) % 360.0
        return deg if deg >= 0 else deg + 360.0

    def ik_callback(self, msg):
        # Wrist position in camera frame (meters)
        x_cam = msg.y
        y_cam = -msg.x

        # Total camera angle = theta0 + theta1
        theta_cam = self.current_theta0 + self.current_theta1

        # Rotate wrist offset into world frame
        x_world_offset = math.cos(theta_cam) * x_cam - math.sin(theta_cam) * y_cam
        y_world_offset = math.sin(theta_cam) * x_cam + math.cos(theta_cam) * y_cam

        # Get camera (end-effector) position in world using FK
        cam_x = self.L1 * math.cos(self.current_theta0) + self.L2 * math.cos(theta_cam)
        cam_y = self.L1 * math.sin(self.current_theta0) + self.L2 * math.sin(theta_cam)

        # Absolute target position in world
        x = cam_x + x_world_offset
        y = cam_y + y_world_offset

        # Run inverse kinematics
        cos_theta1 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if abs(cos_theta1) > 1.0:
            self.get_logger().warn("Target out of reach!")
            return

        sin_theta1 = math.sqrt(1 - cos_theta1**2)
        theta1 = math.atan2(sin_theta1, cos_theta1)
        theta0 = math.atan2(y, x) - math.atan2(self.L2 * sin_theta1, self.L1 + self.L2 * cos_theta1)

        # Convert to [0, 360) degrees
        deg0 = self.radians_to_deg360(theta0)
        deg1 = self.radians_to_deg360(theta1)

        # Publish joint command
        cmd = LampJointCommands()
        cmd.joint_0 = deg0  # Base rotation
        cmd.joint_1 = deg1  # First arm joint
        cmd.joint_2 = 0.0
        cmd.joint_3 = 0.0
        cmd.joint_4 = 0.0

        self.publisher.publish(cmd)

        self.get_logger().info(
            f"Target (world): ({x:.3f}, {y:.3f}) → θ0={deg0:.1f}°, θ1={deg1:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
