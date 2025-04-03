import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HandPositionPublisher(Node):
    def __init__(self):
        super().__init__('hand_position_publisher')
        self.publisher_ = self.create_publisher(Point, 'wrist_position', 10)

        self.cap = cv2.VideoCapture(32)  # Use your working camera index here

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,  # <- Limit to just one hand detection
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def publish_position(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published wrist position: x={x:.1f}, y={y:.1f}')

    def run(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to grab frame")
                continue

            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb_frame)

            height, width, _ = frame.shape
            center_x, center_y = width // 2, height // 2

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]  # <- Just the first hand
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                wrist_x = int(wrist.x * width)
                wrist_y = int(wrist.y * height)

                self.publish_position(wrist_x, wrist_y)

                # Draw stuff
                cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"({wrist_x}, {wrist_y})", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                self.mp_drawing.draw_landmarks(frame, hand_landmarks,
                                               self.mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandPositionPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
