import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QSlider, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer

from std_msgs.msg import Int32, Float64MultiArray

import sys

class LampGui(Node, QWidget):
    def __init__(self):
        rclpy.init(args=None)
        Node.__init__(self, 'lamp_gui')
        QWidget.__init__(self)

        self.setWindowTitle('Surgical Lamp GUI')
        self.setGeometry(200, 200, 400, 300)

        layout = QVBoxLayout()

        # System Mode
        self.mode_dropdown = QComboBox()
        self.mode_dropdown.addItems([
            "Idle", "Track Remote", "Hold Position", "Manual Joint Control", "Manual Pose Control", "Preset Pose", "Light-Only Mode"
        ])
        self.mode_dropdown.currentIndexChanged.connect(self.publish_mode)
        layout.addWidget(QLabel("System Mode"))
        layout.addWidget(self.mode_dropdown)

        # Light Mode
        light_layout = QHBoxLayout()
        layout.addWidget(QLabel("Light Mode"))
        for i in range(4):  # Example: Off, Low, Medium, High
            btn = QPushButton(f"Mode {i}")
            btn.clicked.connect(lambda _, mode=i: self.publish_light_mode(mode))
            light_layout.addWidget(btn)
        layout.addLayout(light_layout)

        # Joint Sliders
        layout.addWidget(QLabel("Manual Joint Angles"))
        self.sliders = []
        for i in range(5):
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            layout.addWidget(QLabel(f"Joint {i}"))
            layout.addWidget(slider)
            self.sliders.append(slider)

        # Timer to periodically publish joint angles in manual mode
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joint_angles)
        self.timer.start(500)

        self.setLayout(layout)

        # ROS publishers
        self.mode_pub = self.create_publisher(Int32, 'system_mode', 10)
        self.light_pub = self.create_publisher(Int32, 'light_mode_cmd', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, 'manual_joint_input', 10)

        self.show()

    def publish_mode(self):
        msg = Int32()
        msg.data = self.mode_dropdown.currentIndex()
        self.mode_pub.publish(msg)

    def publish_light_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.light_pub.publish(msg)

    def publish_joint_angles(self):
        if self.mode_dropdown.currentIndex() == 3:  # Manual Joint Control
            msg = Float64MultiArray()
            msg.data = [slider.value() for slider in self.sliders]
            self.joint_pub.publish(msg)

def main():
    app = QApplication(sys.argv)
    gui = LampGui()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
