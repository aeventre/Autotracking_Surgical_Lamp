# camera_calibration/camera_calibration/camera_calibrator_node.py

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf_transformations
import apriltag

class CameraCalibratorNode(Node):
    def __init__(self):
        super().__init__('camera_calibrator_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('tag_size', 0.045)  # 45 mm default

        # Get parameter values
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_size = self.get_parameter('tag_size').value

        self.bridge = CvBridge()

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.camera_matrix = None
        self.dist_coeffs = None

        # Initialize the AprilTag detector
        self.detector = apriltag.Detector()

        self.get_logger().info("CameraCalibratorNode initialized.")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Received camera intrinsics.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        # Convert to OpenCV grayscale image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # Detect AprilTags
        detections = self.detector.detect(cv_image)

        if len(detections) == 0:
            self.get_logger().warn("No AprilTags detected.")
            return

        obj_points = []  # 3D points in world
        img_points = []  # 2D points in image

        for detection in detections:
            tag_id = detection.tag_id

            try:
                # Lookup transform for tag frame
                trans = self.tf_buffer.lookup_transform('world', f'AprilTag{tag_id}_1', rclpy.time.Time())

                # Extract world position
                tag_translation = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                # Get world orientation (quaternion → rotation matrix)
                quat = trans.transform.rotation
                rotation_matrix = tf_transformations.quaternion_matrix([
                    quat.x,
                    quat.y,
                    quat.z,
                    quat.w
                ])[:3, :3]  # 3x3 rotation matrix

                # Define 3D object points (tag corners in tag local frame)
                half_size = self.tag_size / 2.0
                tag_local_corners = np.array([
                    [-half_size,  half_size, 0],  # Top-left
                    [ half_size,  half_size, 0],  # Top-right
                    [ half_size, -half_size, 0],  # Bottom-right
                    [-half_size, -half_size, 0],  # Bottom-left
                ])

                # Transform corners from tag local frame to world frame
                tag_world_corners = (rotation_matrix @ tag_local_corners.T).T + tag_translation

                # Add corresponding 2D image points (detected corners)
                for i in range(4):
                    obj_points.append(tag_world_corners[i])
                    img_points.append(detection.corners[i])

            except Exception as e:
                self.get_logger().warn(f"Could not find TF for AprilTag{tag_id}_1: {str(e)}")
                continue

        if len(obj_points) < 4:
            self.get_logger().warn("Not enough correspondences to solve PnP.")
            return

        obj_points = np.array(obj_points, dtype=np.float32)
        img_points = np.array(img_points, dtype=np.float32)

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)

        if not success:
            self.get_logger().error("PnP solution failed.")
            return

        # Convert rotation vector to quaternion
        R, _ = cv2.Rodrigues(rvec)
        quat = tf_transformations.quaternion_from_matrix(np.vstack((
            np.hstack((R, np.zeros((3,1)))), 
            np.array([0,0,0,1])
        )))

        # Publish the camera pose relative to world
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.camera_frame

        transform.transform.translation.x = tvec[0][0]
        transform.transform.translation.y = tvec[1][0]
        transform.transform.translation.z = tvec[2][0]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Published transform: world -> {self.camera_frame}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
