import os
import time
import yaml

import cv2
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
import message_filters
from cv_bridge import CvBridge
from tf_transformations import quaternion_matrix


class WristTrackerNode(Node):
    def __init__(self):
        super().__init__('wrist_tracker')

        # --- CvBridge and MediaPipe Hands setup ---
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.hand_detector = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # --- Load extrinsics ---
        base_path = '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data'
        self.cam1_to_base = self.load_transform(os.path.join(base_path, 'camera_01_poses.yaml'))
        self.cam2_to_base = self.load_transform(os.path.join(base_path, 'camera_02_poses.yaml'))

        # --- CameraInfo storage ---
        self.cam_info_cam1 = None
        self.cam_info_cam2 = None

        # --- Subscribers with ApproximateTimeSynchronizer ---
        self.color_sub_cam1 = message_filters.Subscriber(self, CompressedImage, '/camera_01/color/image_raw/compressed')
        self.depth_sub_cam1 = message_filters.Subscriber(self, Image, '/camera_01/depth/image_raw')
        self.info_sub_cam1  = self.create_subscription(CameraInfo, '/camera_01/color/camera_info', self.cam_info_callback_cam1, 10)

        self.color_sub_cam2 = message_filters.Subscriber(self, CompressedImage, '/camera_02/color/image_raw/compressed')
        self.depth_sub_cam2 = message_filters.Subscriber(self, Image, '/camera_02/depth/image_raw')
        self.info_sub_cam2  = self.create_subscription(CameraInfo, '/camera_02/color/camera_info', self.cam_info_callback_cam2, 10)

        sync1 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam1, self.depth_sub_cam1],
            queue_size=10, slop=0.5, allow_headerless=True)
        sync1.registerCallback(self.camera_callback_cam1)

        sync2 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam2, self.depth_sub_cam2],
            queue_size=10, slop=0.5, allow_headerless=True)
        sync2.registerCallback(self.camera_callback_cam2)

        # --- Kalman filter setup ---
        self.kf_initialized = False
        self.kf_state = np.zeros((6, 1))   # [x, y, z, vx, vy, vz]
        self.kf_P     = np.eye(6) * 1.0
        self.kf_Q     = np.eye(6) * 0.01
        self.kf_R     = np.eye(3) * 0.05
        self.kf_H     = np.zeros((3, 6))
        np.fill_diagonal(self.kf_H[:3, :3], 1.0)
        self.last_kf_time = time.time()

        # --- Publishers & storage ---
        self.pose_pub     = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        self.position_cam1 = None
        self.position_cam2 = None

        # --- Timer to fuse and publish at 20 Hz ---
        self.create_timer(0.05, self.publish_fused_pose)

        self.get_logger().info('WristTrackerNode initialized.')

    def load_transform(self, filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        t = np.array([data['position'][axis] for axis in ('x','y','z')])
        q = np.array([data['orientation'][axis] for axis in ('x','y','z','w')])
        mat = quaternion_matrix(q)
        mat[0:3, 3] = t
        return mat

    def cam_info_callback_cam1(self, msg: CameraInfo):
        self.cam_info_cam1 = msg

    def cam_info_callback_cam2(self, msg: CameraInfo):
        self.cam_info_cam2 = msg

    def camera_callback_cam1(self, color_msg, depth_msg):
        if self.cam_info_cam1 is None:
            return
        pos, vis = self.process_camera(color_msg, depth_msg, self.cam_info_cam1, self.cam1_to_base)
        self.position_cam1 = pos
        cv2.imshow('WristTracker Cam1', vis); cv2.waitKey(1)

    def camera_callback_cam2(self, color_msg, depth_msg):
        if self.cam_info_cam2 is None:
            return
        pos, vis = self.process_camera(color_msg, depth_msg, self.cam_info_cam2, self.cam2_to_base)
        self.position_cam2 = pos
        cv2.imshow('WristTracker Cam2', vis); cv2.waitKey(1)

    def process_camera(self, color_msg, depth_msg, cam_info, cam_to_base):
    # Decode compressed color image
        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert depth image to OpenCV
        depth_np = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Copy for visualization
        vis_img = color_np.copy()

        h, w = color_np.shape[:2]
        rgb = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)
        results = self.hand_detector.process(rgb)

        # No hands detected?
        if not results.multi_hand_landmarks:
            return None, vis_img

        # Find the wrist (landmark 0) with smallest valid depth
        closest = None
        min_d = float('inf')
        for hand_lms in results.multi_hand_landmarks:
            lm = hand_lms.landmark[0]  # wrist landmark
            u = int(lm.x * w)
            v = int(lm.y * h)
            u = np.clip(u, 0, w - 1)
            v = np.clip(v, 0, h - 1)

            # Scale into depth image coordinates
            scaled_u = int(u * depth_np.shape[1] / w)
            scaled_v = int(v * depth_np.shape[0] / h)
            scaled_u = np.clip(scaled_u, 0, depth_np.shape[1] - 1)
            scaled_v = np.clip(scaled_v, 0, depth_np.shape[0] - 1)

            # Lookup depth
            d = depth_np[scaled_v, scaled_u]
            if d <= 0 or np.isnan(d):
                continue

            if d < min_d:
                min_d = d
                closest = (u, v, d)

        # No valid wrist depth found?
        if closest is None:
            return None, vis_img

        u, v, depth = closest

        # Back-project to 3D in camera frame
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx_i = cam_info.k[2]
        cy_i = cam_info.k[5]

        x = (u - cx_i) * depth / fx
        y = (v - cy_i) * depth / fy
        z = depth
        point_cam = np.array([x, y, z, 1.0])

        # Transform into base frame
        point_base = cam_to_base @ point_cam

        # Draw marker and annotation
        cv2.circle(vis_img, (u, v), 8, (0, 255, 0), 2)
        text = f"{point_base[0]:.2f}, {point_base[1]:.2f}, {point_base[2]:.2f} m"
        cv2.putText(vis_img, text, (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return point_base[:3], vis_img


    def publish_fused_pose(self):
        # nothing to do if neither camera has seen a wrist
        if self.position_cam1 is None and self.position_cam2 is None:
            return

        # Fuse positions: average if both present, otherwise take whichever is non-None
        if self.position_cam1 is not None and self.position_cam2 is not None:
            fused = (self.position_cam1 + self.position_cam2) / 2.0
        elif self.position_cam1 is not None:
            fused = self.position_cam1
        else:
            fused = self.position_cam2

        # Initialize Kalman filter state on first valid measurement
        if not self.kf_initialized:
            self.kf_state[0:3] = fused.reshape(3, 1)
            self.kf_initialized = True

        # Run the Kalman filter
        filtered = self.run_kalman_filter(fused)
        if np.isnan(filtered).any():
            self.get_logger().warn("Filtered output contains NaNs. Skipping publish.")
            return

        # Publish PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(filtered[0])
        msg.pose.position.y = float(filtered[1])
        msg.pose.position.z = float(filtered[2])
        msg.pose.orientation.w = 1.0  # no orientation for now
        self.pose_pub.publish(msg)


    def run_kalman_filter(self, zpos):
        z = np.array(zpos).reshape(3,1)
        now = time.time()
        dt  = now - self.last_kf_time
        self.last_kf_time = now

        # Predict
        F = np.eye(6)
        F[0,3] = F[1,4] = F[2,5] = dt
        self.kf_state = F @ self.kf_state
        self.kf_P     = F @ self.kf_P @ F.T + self.kf_Q

        # Update
        y = z - self.kf_H @ self.kf_state
        S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
        K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)
        self.kf_state += K @ y
        self.kf_P     = (np.eye(6) - K @ self.kf_H) @ self.kf_P

        return self.kf_state[:3].flatten()

    def destroy_node(self):
        # cleanup Mediapipe
        self.hand_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristTrackerNode()
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
