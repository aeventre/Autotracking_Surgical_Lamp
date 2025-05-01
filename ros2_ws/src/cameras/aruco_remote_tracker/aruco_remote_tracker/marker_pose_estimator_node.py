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

        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('tag_size', 0.045)  # meters

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_size = self.get_parameter('tag_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Detection filtering params
        self.alpha = 0.7
        self.last_position = None

        # Valid tag IDs to track (e.g. cube markers)
        self.valid_tag_ids = [2, 3, 5, 6, 7]

        # Load the specified ArUco dictionary
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
        self.aruco_params.adaptiveThreshWinSizeMin = 5
        self.aruco_params.adaptiveThreshWinSizeMax = 45
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.maxErroneousBitsInBorderRate = 0.03
        self.aruco_params.errorCorrectionRate = 0.6

        # Subscriptions
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.get_logger().info('MarkerPoseEstimatorNode initialized.')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Received camera intrinsics.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                self.get_logger().info(f"Detected markers with ids: {ids.flatten()}")

                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id not in self.valid_tag_ids:
                        self.get_logger().info(f"Ignoring marker ID {marker_id} (not in valid tag list)")
                        continue

                    # Check if marker is large enough (perimeter)
                    perimeter = cv2.arcLength(corners[i][0], True)
                    if perimeter < 5:
                        self.get_logger().info(f"Ignoring marker ID {marker_id} due to small perimeter")
                        continue

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], self.tag_size, self.camera_matrix, self.dist_coeffs
                    )

                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]

                    # Apply EMA filtering to position
                    if self.last_position is None:
                        self.last_position = tvec
                    else:
                        self.last_position = self.alpha * tvec + (1 - self.alpha) * self.last_position
                    smoothed_tvec = self.last_position

                    self.get_logger().info(f"Marker ID {marker_id} detected.")
                    self.get_logger().info(f"Smoothed Translation: {smoothed_tvec}")

                    # Convert rotation vector to matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quat = tf_transformations.quaternion_from_matrix(np.vstack([
                        np.hstack([rotation_matrix, np.array([[0], [0], [0]])]),
                        np.array([0, 0, 0, 1])
                    ]))

                    # Draw marker and axes
                    cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, smoothed_tvec, 0.03)

                    # Overlay position text
                    corner = corners[i][0][0]
                    x, y, z = smoothed_tvec
                    text = f"ID {marker_id}: x={x:.2f}, y={y:.2f}, z={z:.2f}"
                    cv2.putText(cv_image, text, (int(corner[0]), int(corner[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                self.get_logger().info("No markers detected.")

            cv2.imshow('Aruco Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image callback exception: {str(e)}")


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
