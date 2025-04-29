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

class RemoteTrackerNode(Node):
    def __init__(self):
        super().__init__('remote_tracker')

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
            slop=0.5, 
            allow_headerless=True
        )
        self.sync_cam2.registerCallback(self.camera_callback_cam2)


        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/remote_pose', 10)

        # Storage
        self.position_cam1 = None
        self.position_cam2 = None

        # Timer to fuse and publish
        self.timer = self.create_timer(0.05, self.publish_fused_pose)  # 20Hz

    def load_transform(self, filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        translation = np.array([
            data['position']['x'],
            data['position']['y'],
            data['position']['z']
        ])
        quaternion = np.array([
            data['orientation']['x'],
            data['orientation']['y'],
            data['orientation']['z'],
            data['orientation']['w']
        ])

        matrix = quaternion_matrix(quaternion)
        matrix[0:3, 3] = translation
        return matrix

    def cam_info_callback_cam1(self, msg):
        self.cam_info_cam1 = msg

    def cam_info_callback_cam2(self, msg):
        self.cam_info_cam2 = msg

    def camera_callback_cam1(self, color_msg, depth_msg):
        print("[RemoteTrackerNode] Camera 1 callback triggered!")
        if self.cam_info_cam1:
            position, image = self.process_camera(color_msg, depth_msg, self.cam_info_cam1, self.cam1_to_base)
            self.position_cam1 = position
            cv2.imshow('Camera 1 View', image)
            cv2.waitKey(1)

    def camera_callback_cam2(self, color_msg, depth_msg):
        print("[RemoteTrackerNode] Camera 2 callback triggered!")
        if self.cam_info_cam2:
            position, image = self.process_camera(color_msg, depth_msg, self.cam_info_cam2, self.cam2_to_base)
            self.position_cam2 = position
            cv2.imshow('Camera 2 View', image)
            cv2.waitKey(1)

    def process_camera(self, color_msg, depth_msg, cam_info, cam_to_base):
        # Decode compressed color image
        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Depth image (still raw)
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        display_image = color_image.copy()

        # Detect green ball
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # lower_green = np.array([40, 40, 40])
        # upper_green = np.array([80, 255, 255])
        
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([20, 255, 255])
        
        # Threshold
       # mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Reduce random speckle noise
        mask = cv2.GaussianBlur(mask, (9, 9), 2)

        # Further clean noise, preserve real blobs
        mask = cv2.medianBlur(mask, 5)

        # Final cleanup with a smooth elliptical kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, display_image

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M['m00'] == 0:
            return None, display_image

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Draw circle around ball
        cv2.circle(display_image, (cx, cy), 10, (0, 0, 255), 2)
        
        # Clamp cy to depth image size
        cy = np.clip(cy, 0, depth_image.shape[0] - 1)
        cx = np.clip(cx, 0, depth_image.shape[1] - 1)


        # Depth lookup
        # Scale (cx, cy) from color image size to depth image size
        scaled_cx = int(cx * (depth_image.shape[1] / color_image.shape[1]))
        scaled_cy = int(cy * (depth_image.shape[0] / color_image.shape[0]))

        # Make sure indices are safe
        scaled_cx = np.clip(scaled_cx, 0, depth_image.shape[1] - 1)
        scaled_cy = np.clip(scaled_cy, 0, depth_image.shape[0] - 1)

        # Now access depth at the scaled position
        depth = depth_image[scaled_cy, scaled_cx]

        if np.isnan(depth) or depth <= 0.0:
            return None, display_image

        # Deproject to 3D
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx_intrinsic = cam_info.k[2]
        cy_intrinsic = cam_info.k[5]

        x = (cx - cx_intrinsic) * depth / fx
        y = (cy - cy_intrinsic) * depth / fy
        z = depth

        point_cam = np.array([x, y, z, 1.0])  # homogeneous

        # Transform to base frame
        point_base = cam_to_base @ point_cam

        # Draw the 3D coordinates text
        text = f"x: {point_base[0]:.2f}  y: {point_base[1]:.2f}  z: {point_base[2]:.2f}"
        cv2.putText(display_image, text, (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return point_base[0:3], display_image




    def publish_fused_pose(self):
        if self.position_cam1 is None and self.position_cam2 is None:
            return

        if self.position_cam1 is not None and self.position_cam2 is not None:
            fused = (self.position_cam1 + self.position_cam2) / 2.0
        elif self.position_cam1 is not None:
            fused = self.position_cam1
        else:
            fused = self.position_cam2

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # or whatever your base frame is

        msg.pose.position.x = fused[0]
        msg.pose.position.y = fused[1]
        msg.pose.position.z = fused[2]

        # No orientation needed yet (so just default 0,0,0,1)
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)

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
    cv2.destroyAllWindows()