import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
import message_filters

import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
import os
from tf_transformations import quaternion_matrix
import time


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

        # Paths to your extrinsic calibration files
        base_path = '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data'
        cam1_pose_path = os.path.join(base_path, 'camera_01_poses.yaml')
        cam2_pose_path = os.path.join(base_path, 'camera_02_poses.yaml')

        self.cam1_to_base = self.load_transform(cam1_pose_path)
        self.cam2_to_base = self.load_transform(cam2_pose_path)

        self.cam_info_cam1 = None
        self.cam_info_cam2 = None
        

        # Subscribers (using TimeSynchronizer to pair color and depth images)
        self.color_sub_cam1 = message_filters.Subscriber(self, CompressedImage, '/camera_01/color/image_raw/compressed')
        self.depth_sub_cam1 = message_filters.Subscriber(self, Image, '/camera_01/depth/image_raw')
        self.info_sub_cam1 = self.create_subscription(CameraInfo, '/camera_01/color/camera_info', self.cam_info_callback_cam1, 10)

        self.color_sub_cam2 = message_filters.Subscriber(self, CompressedImage, '/camera_02/color/image_raw/compressed')
        self.depth_sub_cam2 = message_filters.Subscriber(self, Image, '/camera_02/depth/image_raw')
        self.info_sub_cam2 = self.create_subscription(CameraInfo, '/camera_02/color/camera_info', self.cam_info_callback_cam2, 10)


        self.sync_cam1 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam1, self.depth_sub_cam1], 
            queue_size=10, 
            slop=0.5, 
            allow_headerless=True
        )
        self.sync_cam1.registerCallback(self.camera_callback_cam1)

        self.sync_cam2 = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_cam2, self.depth_sub_cam2], 
            queue_size=10, 
            slop=0.05, 
            allow_headerless=True
        )
        self.sync_cam2.registerCallback(self.camera_callback_cam2)
        
                # Kalman filter initialization
        self.kf_initialized = False
        self.kf_state = np.zeros((6, 1))  # [x, y, z, vx, vy, vz]
        self.kf_P = np.eye(6) * 1.0
        self.kf_Q = np.eye(6) * 0.01  # process noise
        self.kf_R = np.eye(3) * 0.05  # measurement noise
        self.kf_H = np.zeros((3, 6))
        self.kf_H[0, 0] = 1
        self.kf_H[1, 1] = 1
        self.kf_H[2, 2] = 1

        self.last_kf_time = time.time()
        




        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/remote_pose', 10)

        # Storage
        self.position_cam1 = None
        self.position_cam2 = None

        # Timer to fuse and publish
        self.timer = self.create_timer(0.05, self.publish_fused_pose)  # 20Hz
        
        
    def cam_info_callback_cam1(self, msg):
        self.get_logger().info("CAM1 callback hit")
        if self.cam_info_cam1 is None:
            self.cam_info_cam1 = msg
            self.get_logger().info("Cached cam1 intrinsics.")
            self.destroy_subscription(self.info_sub_cam1)



    def cam_info_callback_cam2(self, msg):
        self.get_logger().info("CAM2 callback hit")
        if self.cam_info_cam2 is None:
            self.cam_info_cam2 = msg
            self.get_logger().info("Cached cam2 intrinsics.")
            self.destroy_subscription(self.info_sub_cam2)



    def load_transform(self, filepath):
            try:
                with open(filepath, 'r') as f:
                    data = yaml.safe_load(f)
                t = np.array([data['position'][k] for k in ('x', 'y', 'z')])
                q = np.array([data['orientation'][k] for k in ('x', 'y', 'z', 'w')])
                M = quaternion_matrix(q)
                M[0:3, 3] = t

                if np.linalg.norm(t) > 5.0:
                    self.get_logger().warn(f"Transform from {filepath} has large translation: {t}")

                return M
            except Exception as e:
                self.get_logger().error(f"Failed to load transform from {filepath}: {e}")
                return np.eye(4)


    def camera_callback_cam1(self, color_msg, depth_msg):
        self.get_logger().info("CAM1 image/depth callback hit")

        if self.cam_info_cam1 is None:
            self.get_logger().warn("cam_info_cam1 is None! Skipping processing.")
            return

        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        position, uv, image = self.process_camera_data(color, depth_msg, self.cam_info_cam1, self.cam1_to_base)
        self.position_cam1 = position
        self.uv_cam1 = uv

        # Always show a window, fallback to color if image is None
        display_image = image if image is not None else color
        cv2.imshow("Camera 1 View", display_image)
        cv2.waitKey(1)





    def camera_callback_cam2(self, color_msg, depth_msg):
        self.get_logger().info("CAM2 image/depth callback hit")

        if self.cam_info_cam2 is None:
            self.get_logger().warn("cam_info_cam2 is None! Skipping processing.")
            return

        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        position, uv, image = self.process_camera_data(color, depth_msg, self.cam_info_cam2, self.cam2_to_base)
        self.position_cam2 = position
        self.uv_cam2 = uv

        # Always show a window, fallback to color if image is None
        display_image = image if image is not None else color
        cv2.imshow("Camera 2 View", display_image)
        cv2.waitKey(1)







    def process_camera_data(self, color, depth_msg, cam_info, cam_to_base):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        display = color.copy()
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        # Threshold for green color
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

        # Apply depth value filtering
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
        cam_pt = ray * d

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
            self.get_logger().warn("No valid position data from either camera.")
            return

        if not self.kf_initialized:
            self.kf_state[0:3] = np.array(fused).reshape((3, 1))
            self.kf_state[3:6] = 0.0
            self.kf_initialized = True


        filt = self.run_kalman_filter(fused)
        if filt is None:
            return  # skipped due to large jump

        self.get_logger().info(f"Raw fused: {fused}, Filtered: {filt}")

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(filt[0]) / 1000.0
        msg.pose.position.y = float(filt[1]) / 1000.0
        msg.pose.position.z = float(filt[2]) / 1000.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)




        
    def run_kalman_filter(self, position):
        z = np.array(position).reshape((3, 1))  # measurement

        current_time = time.time()
        dt = current_time - self.last_kf_time
        self.last_kf_time = current_time

        # State transition matrix
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Predict
        predicted_state = F @ self.kf_state
        predicted_pos = predicted_state[0:3].flatten()

        # Reject large jumps (>20 cm)
        if np.linalg.norm(predicted_pos - position) > 200:
            self.get_logger().warn(f"Skipping Kalman update: large input jump from {predicted_pos} to {position}")
            return predicted_pos

        self.kf_state = predicted_state
        self.kf_P = F @ self.kf_P @ F.T + self.kf_Q

        # Update
        y = z - self.kf_H @ self.kf_state  # innovation
        S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
        K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)  # Kalman gain

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
