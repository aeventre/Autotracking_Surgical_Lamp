import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        self.declare_parameter('L1', 0.2032)  # Arm segment 1 length (meters)
        self.declare_parameter('L2', 0.3175)  # Arm segment 2 length (meters)
        
        self.L1 = self.get_parameter('L1').value
        self.L2 = self.get_parameter('L2').value
        
        # Joint angle limits (in radians)
        self.theta1_min = math.radians(-90)
        self.theta1_max = math.radians(90)
        self.theta2_min = math.radians(-90)
        self.theta2_max = math.radians(90)
        
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.ik_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_angles',
            10
        )
        
    def ik_callback(self, msg):
        x, y = msg.x, msg.y

        # Compute theta2
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if abs(cos_theta2) > 1.0:
            self.get_logger().warn("Target out of reach!")
            return
        
        sin_theta2 = math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)

        # Compute theta1
        theta1 = math.atan2(y, x) - math.atan2(self.L2 * sin_theta2, self.L1 + self.L2 * cos_theta2)

        # Clamp joint angles to limits
        theta1 = max(self.theta1_min, min(self.theta1_max, theta1))
        theta2 = max(self.theta2_min, min(self.theta2_max, theta2))

        # Publish joint angles with three additional zero angles
        joint_msg = Float64MultiArray()
        joint_msg.data = [theta1, theta2, 0.0, 0.0, 0.0]
        self.publisher.publish(joint_msg)

        self.get_logger().info(f"Target: ({x:.2f}, {y:.2f}) -> Joint Angles: [{math.degrees(theta1):.2f}, {math.degrees(theta2):.2f}, 0.00, 0.00, 0.00]")

def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
