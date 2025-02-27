import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import mediapipe as mp
import urllib.request

class HandTrackerNode(Node):
    def __init__(self):
        super().__init__('hand_tracker')
        self.publisher_ = self.create_publisher(Point, 'hand_position', 10)
        self.image_publisher_ = self.create_publisher(Image, 'hand_image', 10)
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        self.esp32_url = 'http://192.168.4.1/capture'  # Change to your ESP32-CAM IP
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Hand Tracker Node Initialized")

    def process_frame(self):
        try:
            resp = urllib.request.urlopen(self.esp32_url)
            img_array = np.asarray(bytearray(resp.read()), dtype=np.uint8)
            frame = cv2.imdecode(img_array, -1)
        except Exception as e:
            self.get_logger().error(f"Failed to get image: {e}")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
                wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
                hand_position = Point(x=wrist.x, y=wrist.y, z=wrist.z)
                self.publisher_.publish(hand_position)

        self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
