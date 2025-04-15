import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations # type: ignore

class GoalPoseNode(Node):
    def __init__(self):
        super().__init__('goal_pose_calculator')

        # Declare offset parameter
        self.declare_parameter('offset_distance', 0.3)
        self.offset_distance = self.get_parameter('offset_distance').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.param_callback)

        # Subscriptions and publishers
        self.create_subscription(PoseStamped, '/remote_pose', self.remote_pose_callback, 10)
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
        remote_pos = np.array([position.x, position.y, position.z])

        # Get rotation matrix from remote's quaternion
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        R = tf_transformations.quaternion_matrix(q)

        # Offset behind the remote along its local -X
        local_offset = np.array([-self.offset_distance, 0, 0, 0])
        offset_global = R @ local_offset
        goal_pos = remote_pos + offset_global[:3]

        # Vector from goal to remote (direction lamp should face)
        direction = remote_pos - goal_pos
        quat = self.quaternion_look_at(direction)

        # Build and publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.pose.position.x = goal_pos[0]
        goal_msg.pose.position.y = goal_pos[1]
        goal_msg.pose.position.z = goal_pos[2]
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        self.goal_pose_pub.publish(goal_msg)
        self.get_logger().info(f"Published goal pose behind remote at: {goal_pos}")

    def quaternion_look_at(self, target_vec, up_vec=np.array([0, 1, 0])):
        """
        Returns a quaternion that rotates +Z to point along target_vec.
        """
        z = target_vec / np.linalg.norm(target_vec)
        x = np.cross(up_vec, z)
        if np.linalg.norm(x) < 1e-6:
            # If up and z are collinear, use fallback
            up_vec = np.array([1, 0, 0])
            x = np.cross(up_vec, z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)

        rot_matrix = np.eye(4)
        rot_matrix[:3, 0] = x
        rot_matrix[:3, 1] = y
        rot_matrix[:3, 2] = z

        return tf_transformations.quaternion_from_matrix(rot_matrix)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
