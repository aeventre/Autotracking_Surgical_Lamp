import sys
import signal
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton,
    QComboBox, QSlider, QHBoxLayout, QLineEdit,
    QFormLayout, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import Int32, Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from surg_lamp_msgs.msg import UserCommand

class LampGui(Node, QWidget):
    def __init__(self):
        # Initialize both ROS node and Qt widget
        Node.__init__(self, 'lamp_gui')
        QWidget.__init__(self)

        # UI setup
        self.setWindowTitle('Surgical Lamp GUI')
        self.setGeometry(200, 200, 400, 350)
        layout = QVBoxLayout()

        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("font-weight: bold; padding: 4px;")
        layout.addWidget(self.status_label)

        # System Mode dropdown
        layout.addWidget(QLabel("System Mode"))
        self.mode_dropdown = QComboBox()
        self.mode_dropdown.addItems([
            "Idle", "Track Remote", "Hold Position",
            "Manual Joint Control", "Manual Pose Control",
            "Preset Pose", "Light-Only Mode"
        ])
        self.mode_dropdown.currentIndexChanged.connect(self.publish_mode)
        layout.addWidget(self.mode_dropdown)

        # Light Mode buttons
        layout.addWidget(QLabel("Light Mode"))
        light_layout = QHBoxLayout()
        for i in range(4):
            btn = QPushButton(f"Mode {i}")
            btn.clicked.connect(lambda _, m=i: self.publish_light_mode(m))
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

        # Timer to publish joint angles
        self.timer = QTimer(self)
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

        # ROS subscriptions for UX feedback
        self.create_subscription(UserCommand, 'remote_user_command', self.on_remote_user, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal_pose, 10)
        self.create_subscription(Bool, 'planning_status', self.on_planning_status, 10)

        # Qt timer to spin ROS
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.01))
        self.ros_timer.start(50)

        # Ensure closeEvent is called
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.show()

    def closeEvent(self, event):
        # Clean shutdown: stop timer, destroy node, shutdown ROS
        self.ros_timer.stop()
        try:
            self.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        event.accept()

    def on_remote_user(self, msg: UserCommand):
        if msg.reset:
            self.status_label.setText("Status: Resetting")
        else:
            state = "Pressed" if msg.button_pressed else "Released"
            self.status_label.setText(f"Remote Button: {state}")

    def on_goal_pose(self, msg: PoseStamped):
        self.status_label.setText("Tracking Remote Pose")

    def on_planning_status(self, msg: Bool):
        text = "Planning: Succeeded" if msg.data else "Planning: Failed"
        self.status_label.setText(text)

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
    # Initialize ROS and Qt
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *args: app.quit())

    gui = LampGui()
    exit_code = app.exec_()

    # Ensure clean shutdown if window closed without closeEvent
    if rclpy.ok():
        try:
            gui.close()
        except Exception:
            pass
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
