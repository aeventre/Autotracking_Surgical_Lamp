import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import QuaternionStamped
from surg_lamp_msgs.msg import UserCommand

class ControllerComsNode(Node):
    def __init__(self):
        super().__init__('controller_coms')

        # Serial port configuration
        self.mcu1_port = '/dev/ttyUSB0'
        self.mcu2_port = '/dev/ttyACM0'
        self.remote_port = '/dev/ttyUSB1'
        self.baud_rate = 115200
        self.light_mode = 0  # updated via user_command subscription

        # Serial connections
        self.mcu1_serial = self.init_serial(self.mcu1_port)
        self.mcu2_serial = self.init_serial(self.mcu2_port)
        self.remote_serial = self.init_serial(self.remote_port)

        # ROS publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.mcu_status_pub  = self.create_publisher(String, 'mcu_status', 10)
        # Remote input commands (buttons, light, reset)
        self.user_command_pub      = self.create_publisher(UserCommand, 'remote_user_command', 10)
        # Separated orientation stream
        self.remote_orientation_pub = self.create_publisher(QuaternionStamped, 'remote_orientation', 10)

        # ROS subscribers
        self.create_subscription(Float64MultiArray, 'joint_commands', self.joint_command_callback, 10)
        self.create_subscription(UserCommand, 'user_command', self.user_command_callback, 10)

        # Timers
        self.create_timer(2.0, self.check_mcu_status)
        self.create_timer(0.1, self.read_remote_serial)

        self.get_logger().info("Controller Communications Node is Running (Half-Duplex Mode).")

    def init_serial(self, port):
        try:
            ser = serial.Serial(port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"Connected to {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return None

    def joint_command_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 5:
            self.get_logger().warn("Received joint command array with insufficient values.")
            return

        joint_angles = [0.0] * 5

        # MCU1 handles joints 1–4 and light mode
        if self.mcu1_serial:
            try:
                mcu1_cmd = f"<{msg.data[1]},{msg.data[2]},{msg.data[3]},{msg.data[4]},{self.light_mode}>\n"
                self.mcu1_serial.reset_input_buffer()
                self.mcu1_serial.write(mcu1_cmd.encode())
                self.get_logger().debug(f"Sent to MCU1: {mcu1_cmd.strip()}")

                response = self.read_serial_line(self.mcu1_serial)
                if response:
                    values = response.split(',')
                    if len(values) == 5:
                        joint_angles[1:5] = [float(v) for v in values[:4]]
                    else:
                        self.get_logger().warn(f"Unexpected MCU1 response: {response}")
                else:
                    self.get_logger().warn("No response from MCU1.")
            except Exception as e:
                self.get_logger().error(f"Error communicating with MCU1: {e}")

        # MCU2 handles joint 0
        if self.mcu2_serial:
            try:
                mcu2_cmd = f"<{msg.data[0]}>\n"
                self.mcu2_serial.reset_input_buffer()
                self.mcu2_serial.write(mcu2_cmd.encode())
                self.get_logger().debug(f"Sent to MCU2: {mcu2_cmd.strip()}")

                response = self.read_serial_line(self.mcu2_serial)
                if response:
                    joint_angles[0] = float(response)
                else:
                    self.get_logger().warn("No response from MCU2.")
            except Exception as e:
                self.get_logger().error(f"Error communicating with MCU2: {e}")

        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i}' for i in range(5)]
        joint_state_msg.position = joint_angles
        self.joint_state_pub.publish(joint_state_msg)

    def user_command_callback(self, msg: UserCommand):
        try:
            self.light_mode = int(msg.light_mode)
            self.get_logger().info(f"Received light mode from command manager: {self.light_mode}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse light mode from user_command: {e}")

    def read_remote_serial(self):
        if not self.remote_serial or not self.remote_serial.is_open:
            return

        try:
            line = self.remote_serial.readline().decode().strip()
            if not (line.startswith('<') and line.endswith('>')):
                return

            values = line[1:-1].split(',')
            if len(values) != 7:
                self.get_logger().warn(f"Invalid remote serial format: {line}")
                return

            # Parse quaternion & normalize
            x, y, z, w = map(float, values[:4])
            norm = (x*x + y*y + z*z + w*w) ** 0.5
            if norm == 0:
                self.get_logger().warn("Received zero quaternion.")
                return
            x, y, z, w = x/norm, y/norm, z/norm, w/norm

            # Publish orientation separately
            quat_msg = QuaternionStamped()
            quat_msg.header.stamp = self.get_clock().now().to_msg()
            quat_msg.header.frame_id = 'remote_frame'
            quat_msg.quaternion.x = x
            quat_msg.quaternion.y = y
            quat_msg.quaternion.z = z
            quat_msg.quaternion.w = w
            self.remote_orientation_pub.publish(quat_msg)

            # Parse other fields
            buttonstate     = int(values[4])
            desired_light   = int(values[5])
            reset_flag      = int(values[6])

            # Publish UserCommand without quaternion
            cmd = UserCommand()
            cmd.button_pressed    = bool(buttonstate)
            cmd.reset_requested   = bool(reset_flag)
            cmd.light_mode        = desired_light
            self.user_command_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f"Failed to read from remote serial: {e}")

    def read_serial_line(self, ser):
        try:
            line = ser.readline().decode().strip()
            if line.startswith('<') and line.endswith('>'):
                return line[1:-1]
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")
        return ""

    def check_mcu_status(self):
        status = []
        if not self.mcu1_serial or not self.mcu1_serial.is_open:
            status.append("MCU1 disconnected")
        if not self.mcu2_serial or not self.mcu2_serial.is_open:
            status.append("MCU2 disconnected")
        if not self.remote_serial or not self.remote_serial.is_open:
            status.append("Remote disconnected")

        msg_text = "OK" if not status else ", ".join(status)
        if msg_text != "OK":
            self.get_logger().warn(f"MCU status: {msg_text}")

        status_msg = String()
        status_msg.data = msg_text
        self.mcu_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerComsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
