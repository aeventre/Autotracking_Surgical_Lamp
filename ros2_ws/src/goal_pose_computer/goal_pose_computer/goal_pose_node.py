import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult


class GoalPoseNode(Node):
    def __init__(self):
        super().__init__('goal_pose_calculator')

        # Declare parameter for offset distance
        self.declare_parameter('offset_distance', 0.3)
        self.offset_distance = self.get_parameter('offset_distance').value
        self.add_on_set_parameters_callback(self.param_callback)

        # Set up subscriber and publisher
        self.create_subscription(PoseStamped, '/remote_pose', self.remote_pose_callback, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def param_callback(self, params):
        for param in params:
            if param.name == 'offset_distance' and param.type_ == param.Type.DOUBLE:
                self.offset_distance = param.value
                self.get_logger().info(f'Updated offset_distance to {self.offset_distance}')
        return SetParametersResult(successful=True)

    def remote_pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation

        # Convert quaternion to rotation matrix
        q = np.array([ori.x, ori.y, ori.z, ori.w])
        R = self.quaternion_to_rotation_matrix(q)

        # Offset along -X axis of the remote's frame
        offset_local = np.array([-self.offset_distance, 0.0, 0.0])
        offset_world = R @ offset_local
        goal_position = np.array([pos.x, pos.y, pos.z]) + offset_world

        # Compute direction vector (remote - goal), then build look-at quaternion so -Z points to remote
        direction = np.array([pos.x, pos.y, pos.z]) - goal_position
        quat = self.look_at_quaternion(direction)

        # Publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.pose.position.x = goal_position[0]
        goal_msg.pose.position.y = goal_position[1]
        goal_msg.pose.position.z = goal_position[2]
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        self.goal_pose_pub.publish(goal_msg)
        self.get_logger().info(f"Published goal pose: {goal_position}")

    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,           1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,           2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]
        ])

    def look_at_quaternion(self, direction, up=np.array([0.0, 1.0, 0.0])):
        """Compute quaternion where -Z points along `direction`."""
        z = -direction / np.linalg.norm(direction)
        x = np.cross(up, z)
        if np.linalg.norm(x) < 1e-6:  # if up and z are collinear
            up = np.array([1.0, 0.0, 0.0])
            x = np.cross(up, z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)

        rot = np.eye(3)
        rot[0, :] = x
        rot[1, :] = y
        rot[2, :] = z

        return self.rotation_matrix_to_quaternion(rot)

    def rotation_matrix_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to a quaternion [x, y, z, w]."""
        m00, m01, m02 = R[0]
        m10, m11, m12 = R[1]
        m20, m21, m22 = R[2]

        trace = m00 + m11 + m22
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m21 - m12) * s
            y = (m02 - m20) * s
            z = (m10 - m01) * s
        elif m00 > m11 and m00 > m22:
            s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
