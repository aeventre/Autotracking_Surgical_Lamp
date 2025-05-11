import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations


class MarkerPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('marker_pose_estimator')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('tag_size', 0.045)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_size = self.get_parameter('tag_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.valid_tag_ids = [2, 3, 5, 6, 7]

        self.marker_to_base = {
            2: self.create_transform([0.0, -0.33, 0.0], [np.pi/2, 0, 0]),
            3: self.create_transform([0.0, 0.0, -0.33], [np.pi, 0, 0]),
            5: self.create_transform([0.0, 0.33, 0.0], [-np.pi/2, 0, 0]),
            6: self.create_transform([0.0, 0.0, 0.33], [0, 0, 0]),
            7: self.create_transform([0.33, 0.0, 0.0], [np.pi/2, 0, np.pi/2]),
        }

        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dict_map.get(aruco_dict_name, cv2.aruco.DICT_4X4_50)
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.get_logger().info('MarkerPoseEstimatorNode initialized.')

    def create_transform(self, translation, euler_angles):
        tf = np.eye(4)
        tf[:3, :3] = tf_transformations.euler_matrix(*euler_angles)[:3, :3]
        tf[:3, 3] = translation
        return tf

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Camera intrinsics loaded.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners)
            remote_poses = []

            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id not in self.valid_tag_ids:
                    continue

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.tag_size, self.camera_matrix, self.dist_coeffs)

                T_marker_to_cam = np.eye(4)
                T_marker_to_cam[:3, :3], _ = cv2.Rodrigues(rvec[0])
                T_marker_to_cam[:3, 3] = tvec[0][0]

                T_marker_to_remote = self.marker_to_base[marker_id]
                T_remote_to_cam = T_marker_to_cam @ T_marker_to_remote
                remote_poses.append(T_remote_to_cam)

                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            if remote_poses:
                avg_pose = np.mean(remote_poses, axis=0)
                pos = avg_pose[:3, 3]
                quat = tf_transformations.quaternion_from_matrix(avg_pose)

                cv2.putText(cv_image, f"x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow('Aruco Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseEstimatorNode()
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
