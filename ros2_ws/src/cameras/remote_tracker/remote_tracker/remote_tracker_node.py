import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix


class RemoteTrackerNode(Node):
    def __init__(self):
        super().__init__('remote_tracker')
        
        self.declare_parameter('reprojection_threshold', 10.0)
        self.reprojection_threshold = self.get_parameter('reprojection_threshold').value

        self.declare_parameter('hsv_lower', [38, 101, 102])
        self.declare_parameter('hsv_upper', [74, 189, 248])
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)

        self.bridge = CvBridge()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Used later in process_camera_data
        self.cam1_frame = 'camera_01/color_optical_frame'
        self.cam2_frame = 'camera_02/color_optical_frame'
        self.base_frame = 'base_link'


        self.cam_info_cam1 = None
        self.cam_info_cam2 = None

        # Subscribers
        self.color_sub_cam1 = message_filters.Subscriber(self, CompressedImage, '/camera_01/color/image_raw/compressed')
        self.depth_sub_cam1 = message_filters.Subscriber(self, Image, '/camera_01/depth/image_raw')
        self.info_sub_cam1 = self.create_subscription(CameraInfo, '/camera_01/color/camera_info', self.cam_info_callback_cam1, 10)

        self.color_sub_cam2 = message_filters.Subscriber(self, CompressedImage, '/camera_02/color/image_raw/compressed')
        self.depth_sub_cam2 = message_filters.Subscriber(self, Image, '/camera_02/depth/image_raw')
        self.info_sub_cam2 = self.create_subscription(CameraInfo, '/camera_02/color/camera_info', self.cam_info_callback_cam2, 10)

        self.sync_cam1 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam1, self.depth_sub_cam1], queue_size=10, slop=0.5, allow_headerless=True
        )
        self.sync_cam1.registerCallback(self.camera_callback_cam1)

        self.sync_cam2 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam2, self.depth_sub_cam2], queue_size=10, slop=0.05, allow_headerless=True
        )
        self.sync_cam2.registerCallback(self.camera_callback_cam2)

        # Kalman filter
        self.kf_initialized = False
        self.kf_state = np.zeros((6, 1))
        self.kf_P = np.eye(6) * 1.0
        self.kf_Q = np.eye(6) * 0.01
        self.kf_R = np.eye(3) * 0.05
        self.kf_H = np.eye(3, 6)
        self.last_kf_time = time.time()

        # Pose output
        self.pose_pub = self.create_publisher(PoseStamped, '/remote_pose', 10)
        self.position_cam1 = None
        self.position_cam2 = None

        self.timer = self.create_timer(0.05, self.publish_fused_pose)

    def cam_info_callback_cam1(self, msg):
        if self.cam_info_cam1 is None:
            self.cam_info_cam1 = msg
            self.get_logger().info("Cached cam1 intrinsics.")
            self.destroy_subscription(self.info_sub_cam1)

    def cam_info_callback_cam2(self, msg):
        if self.cam_info_cam2 is None:
            self.cam_info_cam2 = msg
            self.get_logger().info("Cached cam2 intrinsics.")
            self.destroy_subscription(self.info_sub_cam2)

    def camera_callback_cam1(self, color_msg, depth_msg):
        if self.cam_info_cam1 is None:
            return
        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        position, uv, image = self.process_camera_data(color, depth_msg, self.cam_info_cam1, self.cam1_frame)
        self.position_cam1 = position
        display_image = image if image is not None else color
        cv2.imshow("Camera 1 View", display_image)
        cv2.waitKey(1)

    def camera_callback_cam2(self, color_msg, depth_msg):
        if self.cam_info_cam2 is None:
            return
        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        position, uv, image = self.process_camera_data(color, depth_msg, self.cam_info_cam2, self.cam2_frame)
        self.position_cam2 = position
        display_image = image if image is not None else color
        cv2.imshow("Camera 2 View", display_image)
        cv2.waitKey(1)

    def process_camera_data(self, color, depth_msg, cam_info, camera_frame_id):
        try:
            # Lookup transform from camera frame to base_link
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=camera_frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            trans = np.array([t.x, t.y, t.z])
            quat = np.array([q.x, q.y, q.z, q.w])
            cam_to_base = quaternion_matrix(quat)
            cam_to_base[0:3, 3] = trans
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None, color

        # Process image and depth
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        display = color.copy()
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.GaussianBlur(mask, (9, 9), 2)
        mask = cv2.medianBlur(mask, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, None, display
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            return None, None, display

        cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        cv2.circle(display, (cx, cy), 10, (0, 0, 255), 2)

        win = 2
        y0, y1 = max(cy - win, 0), min(cy + win + 1, depth.shape[0])
        x0, x1 = max(cx - win, 0), min(cx + win + 1, depth.shape[1])
        patch = depth[y0:y1, x0:x1].astype(np.float32)

        vals = patch[(patch > 100) & (patch < 2000) & (~np.isnan(patch))]
        if vals.size == 0:
            return None, None, display
        d = float(np.median(vals))

        fx, fy = cam_info.k[0], cam_info.k[4]
        cx0, cy0 = cam_info.k[2], cam_info.k[5]
        x_norm = (cx - cx0) / fx
        y_norm = (cy - cy0) / fy
        ray = np.array([x_norm, y_norm, 1.0])
        ray /= np.linalg.norm(ray)
        cam_pt = ray * (d / 1000.0)  # convert mm → meters

        base_pt = cam_to_base @ np.append(cam_pt, 1.0)
        label = f"x:{base_pt[0]:.2f} y:{base_pt[1]:.2f} z:{base_pt[2]:.2f}"
        cv2.putText(display, label, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return base_pt[:3], (cx, cy), display

    def publish_fused_pose(self):
        fused = None
        if self.position_cam1 is not None and self.position_cam2 is not None:
            fused = (self.position_cam1 + self.position_cam2) / 2.0
        elif self.position_cam1 is not None:
            fused = self.position_cam1
        elif self.position_cam2 is not None:
            fused = self.position_cam2
        else:
            return

        if not self.kf_initialized:
            self.kf_state[0:3] = np.array(fused).reshape((3, 1))
            self.kf_state[3:6] = 0.0
            self.kf_initialized = True

        filt = self.run_kalman_filter(fused)
        if filt is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(filt[0])
        msg.pose.position.y = float(filt[1])
        msg.pose.position.z = float(filt[2])
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def run_kalman_filter(self, position):
        z = np.array(position).reshape((3, 1))
        dt = time.time() - self.last_kf_time
        self.last_kf_time = time.time()

        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        predicted = F @ self.kf_state
        predicted_pos = predicted[0:3].flatten()
        if np.linalg.norm(predicted_pos - position) > 200:
            return predicted_pos

        self.kf_state = predicted
        self.kf_P = F @ self.kf_P @ F.T + self.kf_Q

        y = z - self.kf_H @ self.kf_state
        S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
        K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)

        self.kf_state += K @ y
        self.kf_P = (np.eye(6) - K @ self.kf_H) @ self.kf_P
        return self.kf_state[0:3].flatten()


def main(args=None):
    rclpy.init(args=args)
    node = RemoteTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
