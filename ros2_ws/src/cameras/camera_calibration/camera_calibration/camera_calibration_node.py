import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
import yaml
import os

class CameraPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('camera_pose_estimator_node')

        self.declare_parameter('marker_length', 0.05)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_poses_yaml', '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data/camera_poses.yaml')

        self.marker_length = self.get_parameter('marker_length').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_poses_yaml = self.get_parameter('camera_poses_yaml').value

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))

        self.pose_pub = self.create_publisher(PoseStamped, 'camera_pose', 10)
        self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        self.camera_poses = {}

        self.get_logger().info('Camera Pose Estimator Node Initialized.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            self.get_logger().info(f"Detected markers with ids: {ids}")

            for i, marker_id in enumerate(ids.flatten()):
                # Estimate pose for THIS marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                self.get_logger().info(f"Marker ID {marker_id} detected.")
                self.get_logger().info(f"Translation: {tvec}")
                self.get_logger().info(f"Rotation (rvec): {rvec}")

                # Build marker-to-camera transformation
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                marker_to_camera = np.eye(4)
                marker_to_camera[:3, :3] = rotation_matrix
                marker_to_camera[:3, 3] = tvec

                # Try to invert to get camera-to-marker
                try:
                    camera_to_marker = np.linalg.inv(marker_to_camera)
                except np.linalg.LinAlgError as e:
                    self.get_logger().warn(f'Failed to invert transform for marker {marker_id}: {str(e)}')
                    continue  # Skip this marker safely

                # Extract camera position and orientation
                cam_position = camera_to_marker[:3, 3]
                cam_quat = tf_transformations.quaternion_from_matrix(camera_to_marker)

                # Save camera pose for this marker
                self.camera_poses[int(marker_id)] = {
                    'position': [float(cam_position[0]), float(cam_position[1]), float(cam_position[2])],
                    'orientation': [float(cam_quat[0]), float(cam_quat[1]), float(cam_quat[2]), float(cam_quat[3])]
                }

                # Publish the marker pose (still relative to the camera)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.camera_frame

                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])

                # For publishing: reuse marker rotation
                quat_for_pub = tf_transformations.quaternion_from_matrix(np.vstack([
                    np.hstack([rotation_matrix, np.array([[0], [0], [0]])]),
                    np.array([0, 0, 0, 1])
                ]))

                pose_msg.pose.orientation.x = quat_for_pub[0]
                pose_msg.pose.orientation.y = quat_for_pub[1]
                pose_msg.pose.orientation.z = quat_for_pub[2]
                pose_msg.pose.orientation.w = quat_for_pub[3]

                self.pose_pub.publish(pose_msg)

                # Draw marker axis
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            # After all markers processed, save the poses
            self.save_camera_poses_to_yaml()

        else:
            self.get_logger().info("No markers detected.")

        # Always show the frame
        cv2.imshow('Camera View', cv_image)
        cv2.waitKey(1)



    def save_camera_poses_to_yaml(self):
        os.makedirs(os.path.dirname(self.camera_poses_yaml), exist_ok=True)
        with open(self.camera_poses_yaml, 'w') as file:
            yaml.dump(self.camera_poses, file)
        self.get_logger().info(f"Camera poses saved to {os.path.abspath(self.camera_poses_yaml)}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()