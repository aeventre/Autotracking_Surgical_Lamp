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
from sensor_msgs.msg import CameraInfo


class CameraPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('camera_pose_estimator_node')
        
        self.camera_info_received = False
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_01/color/camera_info',  # Adjust as needed
            self.camera_info_callback,
            10
        )

        self.declare_parameter('marker_length', 0.05)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_poses_yaml', '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data/camera_poses.yaml')

        self.marker_length = self.get_parameter('marker_length').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_poses_yaml = self.get_parameter('camera_poses_yaml').value

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.pose_pub = self.create_publisher(PoseStamped, 'camera_pose', 10)
        self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        self.camera_poses = {}

        self.get_logger().info('Camera Pose Estimator Node Initialized.')
        
        self.collected_positions = []
        self.collected_orientations = []
        self.required_samples = 10  # <-- Number of detections before saving
        self.samples_collected = 0


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if not self.camera_info_received:
            self.get_logger().warn("Skipping frame: camera intrinsics not yet received.")
            return

        if ids is not None:
            self.get_logger().info(f"Detected markers with ids: {ids.flatten()}")

            for i, marker_id in enumerate(ids.flatten()):
                # Estimate pose for this marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                self.get_logger().info(f"Marker ID {marker_id} detected.")
                self.get_logger().info(f"Translation (marker to camera): {tvec}")
                self.get_logger().info(f"Rotation (rvec): {rvec}")

                # Build marker-to-camera transformation
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                marker_to_camera = np.eye(4)
                marker_to_camera[:3, :3] = rotation_matrix
                marker_to_camera[:3, 3] = tvec

                # Invert to get camera-to-marker
                try:
                    camera_to_marker = np.linalg.inv(marker_to_camera)
                except np.linalg.LinAlgError as e:
                    self.get_logger().warn(f"Failed to invert transform for marker {marker_id}: {str(e)}")
                    continue

                # Set known tag transform in base frame
                tag_id = int(marker_id)
                if tag_id == 0:
                    tag_translation = np.array([-0.0572, -0.041877, 0.0275])
                    tag_rotation_rpy = np.array([np.pi/2, 0.0, 0.0])  # (90°, 0°, 0°)
                elif tag_id == 1:
                    tag_translation = np.array([-0.0572, 0.023523, 0.0275])
                    tag_rotation_rpy = np.array([-np.pi/2, -np.pi, 0.0])  # (-90°, -180°, 0°)
                else:
                    self.get_logger().warn(f"Unknown tag ID {tag_id}, skipping...")
                    continue

                # Convert known tag RPY to matrix
                # Convert known tag RPY to matrix using extrinsic axes (URDF-style)
                tag_to_base = tf_transformations.euler_matrix(
                tag_rotation_rpy[0], tag_rotation_rpy[1], tag_rotation_rpy[2]
                )
                tag_to_base[0:3, 3] = tag_translation


                # Compute final camera-in-base
                camera_in_base = tag_to_base @ camera_to_marker

                # Extract camera position and orientation
                cam_position = camera_in_base[:3, 3]
                cam_quat = tf_transformations.quaternion_from_matrix(camera_in_base)

                # Collect for smoothing
                self.collected_positions.append(cam_position)
                self.collected_orientations.append(cam_quat)
                self.samples_collected += 1

                self.get_logger().info(f"Collected {self.samples_collected}/{self.required_samples} samples...")

                if self.samples_collected >= self.required_samples:
                    self.save_smoothed_camera_pose()
                    self.samples_collected = 0
                    self.collected_positions.clear()
                    self.collected_orientations.clear()

                # Publish marker pose (relative to camera for visualization)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.camera_frame

                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])

                quat_for_pub = tf_transformations.quaternion_from_matrix(np.vstack([
                    np.hstack([rotation_matrix, np.array([[0], [0], [0]])]),
                    np.array([0, 0, 0, 1])
                ]))

                pose_msg.pose.orientation.x = quat_for_pub[0]
                pose_msg.pose.orientation.y = quat_for_pub[1]
                pose_msg.pose.orientation.z = quat_for_pub[2]
                pose_msg.pose.orientation.w = quat_for_pub[3]

                self.pose_pub.publish(pose_msg)

                # Draw marker axes
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        else:
            self.get_logger().info("No markers detected.")


        cv2.imshow('Camera View', cv_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            if self.collected_positions:
                self.get_logger().info("Manual save triggered by 's' key.")
                self.save_smoothed_camera_pose()
                self.samples_collected = 0
                self.collected_positions.clear()
                self.collected_orientations.clear()
            else:
                self.get_logger().warn("No samples collected yet. Cannot save.")
        elif key == ord('q'):
            self.get_logger().info("Exiting due to 'q' key press.")
            rclpy.shutdown()


        
        
    def save_smoothed_camera_pose(self):
        # Average translation
        avg_position = np.mean(np.vstack(self.collected_positions), axis=0)

        # Average orientation (quaternions)
        quats = np.vstack(self.collected_orientations)
        quat_matrix = np.matmul(quats.T, quats)
        eig_vals, eig_vecs = np.linalg.eigh(quat_matrix)
        avg_quat = eig_vecs[:, np.argmax(eig_vals)]

        # Save averaged pose
        data = {
            'position': {
                'x': float(avg_position[0]),
                'y': float(avg_position[1]),
                'z': float(avg_position[2])
            },
            'orientation': {
                'x': float(avg_quat[0]),
                'y': float(avg_quat[1]),
                'z': float(avg_quat[2]),
                'w': float(avg_quat[3])
            }
        }

        os.makedirs(os.path.dirname(self.camera_poses_yaml), exist_ok=True)
        with open(self.camera_poses_yaml, 'w') as file:
            yaml.dump(data, file, sort_keys=False)

        self.get_logger().info(f"Smoothed camera pose saved to {os.path.abspath(self.camera_poses_yaml)}")
    
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32).reshape((-1, 1))
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics received and cached.")
            self.destroy_subscription(self.camera_info_sub)


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
