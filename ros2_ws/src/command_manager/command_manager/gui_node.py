import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from surg_lamp_msgs.msg import UserCommand

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QPushButton,
                             QComboBox, QSlider, QHBoxLayout, QLineEdit, QFormLayout,
                             QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import Int32, Float64MultiArray, Bool
import sys

class LampGui(Node, QWidget):
    def __init__(self):
        # Initialize ROS
        rclpy.init(args=None)
        Node.__init__(self, 'lamp_gui')
        QWidget.__init__(self)

        # UI setup
        self.setWindowTitle('Surgical Lamp GUI')
        self.setGeometry(200, 200, 400, 350)
        layout = QVBoxLayout()

        # Status label for remote tracking and planning
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("font-weight: bold; padding: 4px;")
        layout.addWidget(self.status_label)

        # System Mode dropdown
        self.mode_dropdown = QComboBox()
        self.mode_dropdown.addItems([
            "Idle", "Track Remote", "Hold Position",
            "Manual Joint Control", "Manual Pose Control",
            "Preset Pose", "Light-Only Mode"
        ])
        self.mode_dropdown.currentIndexChanged.connect(self.publish_mode)
        layout.addWidget(QLabel("System Mode"))
        layout.addWidget(self.mode_dropdown)

        # Light Mode buttons
        light_layout = QHBoxLayout()
        layout.addWidget(QLabel("Light Mode"))
        for i in range(4):
            btn = QPushButton(f"Mode {i}")
            btn.clicked.connect(lambda _, mode=i: self.publish_light_mode(mode))
            light_layout.addWidget(btn)
        layout.addLayout(light_layout)

        # Joint sliders
        layout.addWidget(QLabel("Manual Joint Angles"))
        self.sliders = []
        for i in range(5):
            layout.addWidget(QLabel(f"Joint {i}"))
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            self.sliders.append(slider)
            layout.addWidget(slider)

        # Joint publish timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joint_angles)
        self.timer.start(500)

        # Manual Pose Input
        pose_group = QGroupBox("Manual Pose Input")
        pose_layout = QFormLayout()
        self.pose_fields = {}
        for field in ["x","y","z","qx","qy","qz","qw"]:
            le = QLineEdit(); le.setPlaceholderText("0.0")
            self.pose_fields[field] = le
            pose_layout.addRow(field.upper(), le)
        pose_btn = QPushButton("Send Pose")
        pose_btn.clicked.connect(self.publish_manual_pose)
        pose_layout.addRow(pose_btn)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)

        self.setLayout(layout)

        # ROS publishers
        self.mode_pub = self.create_publisher(Int32, 'system_mode', 10)
        self.light_pub = self.create_publisher(Int32, 'light_mode_cmd', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, 'manual_joint_input', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'manual_pose_input', 10)

        # ROS subscribers for UX feedback
        self.create_subscription(UserCommand, 'remote_user_command',
                                 self.on_remote_user, 10)
        self.create_subscription(PoseStamped, 'goal_pose',
                                 self.on_goal_pose, 10)
        # New: subscribe to planning status
        self.create_subscription(Bool, 'planning_status',
                                 self.on_planning_status, 10)

        # Need to spin ROS in background
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)

        self.show()

    def ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    def on_remote_user(self, msg: UserCommand):
        # Update reset status or button state
        if msg.reset:
            self.status_label.setText("Status: Resetting")
        else:
            state = "Pressed" if msg.button_state else "Released"
            self.status_label.setText(f"Remote Button: {state}")

    def on_goal_pose(self, msg: PoseStamped):
        # Indicate tracking active
        self.status_label.setText("Tracking Remote Pose")

    def on_planning_status(self, msg: Bool):
        # Show planning success/failure
        if msg.data:
            self.status_label.setText("Planning: Succeeded")
        else:
            self.status_label.setText("Planning: Failed")

    def publish_mode(self):
        m = Int32(); m.data = self.mode_dropdown.currentIndex()
        self.mode_pub.publish(m)

    def publish_light_mode(self, mode):
        m = Int32(); m.data = mode
        self.light_pub.publish(m)

    def publish_joint_angles(self):
        if self.mode_dropdown.currentIndex() == 3:
            msg = Float64MultiArray()
            msg.data = [s.value() for s in self.sliders]
            self.joint_pub.publish(msg)

    def publish_manual_pose(self):
        if self.mode_dropdown.currentIndex() != 4:
            self.get_logger().info("Not in Manual Pose Control mode; pose not sent.")
            return
        try:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            for f in ['x','y','z']:
                setattr(pose.pose.position, f, float(self.pose_fields[f].text()))
            for f in ['qx','qy','qz','qw']:
                setattr(pose.pose.orientation, f, float(self.pose_fields[f].text()))
            self.pose_pub.publish(pose)
            self.get_logger().info("Manual pose sent.")
        except ValueError:
            self.get_logger().warn("Invalid pose input. Please enter valid floats.")


def main():
    app = QApplication(sys.argv)
    gui = LampGui()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
