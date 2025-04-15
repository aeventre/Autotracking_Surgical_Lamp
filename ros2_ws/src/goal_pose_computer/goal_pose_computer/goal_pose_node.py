import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations

class GoalPoseNode(Node):
    def __init__(self):
        super().__init__('goal_pose_calculator')

        # Declare and initialize offset distance as a parameter
        self.declare_parameter('offset_distance', 0.3)
        self.offset_distance = self.get_parameter('offset_distance').get_parameter_value().double_value

        # Monitor parameter changes (optional)
        self.add_on_set_parameters_callback(self.param_callback)

        # Subscription to remote pose
        self.create_subscription(
            PoseStamped,
            '/remote_pose',
            self.remote_pose_callback,
            10
        )

        # Publisher for computed goal pose
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def param_callback(self, params):
        for param in params:
            if param.name == 'offset_distance' and param.type_ == param.Type.DOUBLE:
                self.offset_distance = param.value
                self.get_logger().info(f"Updated offset_distance to {self.offset_distance}")
        return rclpy.parameter.ParameterEventHandler().result(successful=True)

    def remote_pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Quaternion: [x, y, z, w]
        q = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert quaternion to 4x4 rotation matrix
        R = tf_transformations.quaternion_matrix(q)

        # Local offset along -X axis
        local_offset = np.array([-self.offset_distance, 0, 0, 0])  # homogeneous coordinates

        # Rotate into global frame
        global_offset = R @ local_offset

        # Compute new global position
        goal_position = np.array([position.x, position.y, position.z]) + global_offset[:3]

        # Create and publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.pose.position.x = goal_position[0]
        goal_msg.pose.position.y = goal_position[1]
        goal_msg.pose.position.z = goal_position[2]
        goal_msg.pose.orientation = orientation

        self.goal_pose_pub.publish(goal_msg)
        self.get_logger().info(f"Published goal pose at: {goal_position}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
