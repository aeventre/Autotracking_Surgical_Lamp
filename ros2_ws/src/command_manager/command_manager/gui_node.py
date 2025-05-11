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
from std_msgs.msg import Int32, Float64MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped
from surg_lamp_msgs.msg import UserCommand


class LampGui(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'lamp_gui')
        QWidget.__init__(self)

        self.setWindowTitle('Surgical Lamp GUI')
        self.setGeometry(200, 200, 400, 600)
        layout = QVBoxLayout()

        # Status
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("font-weight: bold; padding: 4px;")
        layout.addWidget(self.status_label)

        # System Mode
        layout.addWidget(QLabel("System Mode"))
        self.mode_dropdown = QComboBox()
        self.mode_dropdown.addItems([
            "Idle", "Track Remote", "Zeroing Mode",
            "Manual Joint Control", "Manual Pose Control",
            "Light-Only Mode", "System Demo"
        ])
        self.mode_dropdown.currentIndexChanged.connect(self.publish_mode)
        layout.addWidget(self.mode_dropdown)

        # Demo Sequence Selector
        layout.addWidget(QLabel("Demo Sequence"))
        self.demo_dropdown = QComboBox()
        self.demo_dropdown.addItems(["Basic", "Spin"])
        self.demo_dropdown.currentTextChanged.connect(self.publish_demo_sequence_name)
        layout.addWidget(self.demo_dropdown)

        # Light Mode Buttons
        layout.addWidget(QLabel("Light Mode"))
        light_layout = QHBoxLayout()
        for i in range(4):
            btn = QPushButton(f"Mode {i}")
            btn.clicked.connect(lambda _, m=i: self.publish_light_mode(m))
            light_layout.addWidget(btn)
        layout.addLayout(light_layout)

        # Manual Joint Sliders
        layout.addWidget(QLabel("Manual Joint Angles (degrees)"))
        self.sliders = []
        self.slider_displays = []
        self.joint_limits_deg = [
            (-180, 180),     # joint0
            (-180, 180),     # joint1
            (-90, 10),       # joint2 (-1.5708 to 0.1745 rad)
            (-130, 130),     # joint3 (-2.2689 to 2.2689 rad)
            (-130, 130),     # joint4
        ]
        for i, (low, high) in enumerate(self.joint_limits_deg):
            joint_layout = QHBoxLayout()
            joint_layout.addWidget(QLabel(f"Joint {i}"))

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(low))
            slider.setMaximum(int(high))
            slider.setValue(0)
            slider.setSingleStep(1)
            joint_layout.addWidget(slider)

            display = QLineEdit("0")
            display.setFixedWidth(50)
            display.setAlignment(Qt.AlignRight)
            joint_layout.addWidget(display)

            slider.valueChanged.connect(lambda val, d=display: d.setText(str(val)))
            display.editingFinished.connect(lambda s=slider, d=display: s.setValue(int(float(d.text() or 0))))

            self.sliders.append(slider)
            self.slider_displays.append(display)
            layout.addLayout(joint_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_joint_angles)
        self.timer.start(500)

        # Manual Pose Input
        pose_group = QGroupBox("Manual Pose Input")
        pose_layout = QFormLayout()
        self.pose_fields = {}
        for field in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
            le = QLineEdit()
            le.setPlaceholderText("0.0")
            self.pose_fields[field] = le
            pose_layout.addRow(field.upper(), le)
        pose_btn = QPushButton("Send Pose")
        pose_btn.clicked.connect(self.publish_manual_pose)
        pose_layout.addRow(pose_btn)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)

        self.setLayout(layout)

        # ROS Publishers
        self.mode_pub = self.create_publisher(Int32, 'system_mode', 10)
        self.light_pub = self.create_publisher(Int32, 'light_mode_cmd', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, 'manual_joint_input', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'manual_pose_input', 10)
        self.demo_seq_pub = self.create_publisher(String, 'demo_sequence_name', 10)

        # ROS Subscriptions
        self.create_subscription(UserCommand, 'remote_user_command', self.on_remote_user, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal_pose, 10)
        self.create_subscription(Bool, 'planning_status', self.on_planning_status, 10)

        # ROS spin
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.01))
        self.ros_timer.start(50)

        self.setAttribute(Qt.WA_DeleteOnClose)
        self.show()

    def closeEvent(self, event):
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
            state = "Pressed" if msg.button_state else "Released"
            self.status_label.setText(f"Remote Button: {state}")

    def on_goal_pose(self, msg: PoseStamped):
        self.status_label.setText("Tracking Remote Pose")

    def on_planning_status(self, msg: Bool):
        self.status_label.setText("Planning: Succeeded" if msg.data else "Planning: Failed")

    def publish_mode(self):
        m = Int32()
        m.data = self.mode_dropdown.currentIndex()
        self.mode_pub.publish(m)

    def publish_light_mode(self, mode):
        m = Int32(); m.data = mode
        self.light_pub.publish(m)

    def publish_demo_sequence_name(self, name):
        msg = String()
        msg.data = name
        self.demo_seq_pub.publish(msg)

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

            for f in ['x', 'y', 'z']:
                setattr(pose.pose.position, f, float(self.pose_fields[f].text()))

            for gui_f, ros_f in zip(['qx', 'qy', 'qz', 'qw'], ['x', 'y', 'z', 'w']):
                setattr(pose.pose.orientation, ros_f, float(self.pose_fields[gui_f].text()))

            self.pose_pub.publish(pose)
            self.get_logger().info("Manual pose sent.")
        except ValueError as e:
            self.get_logger().warn(f"Invalid pose input: {e}")


def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *args: app.quit())
    gui = LampGui()
    exit_code = app.exec_()
    if rclpy.ok():
        try:
            gui.close()
        except Exception:
            pass
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
