import rclpy
from rclpy.node import Node
import serial
from surg_lamp_msgs.msg import LampJointCommands, LampCurrentAngles
from std_msgs.msg import Int32, String


class ControllerComsNode(Node):
    def __init__(self):
        super().__init__('controller_coms')

        self.mcu1_port = '/dev/ttyUSB1'
        self.mcu2_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.light_mode = 0

        # Serial connections
        self.mcu1_serial = self.init_serial(self.mcu1_port)
        self.mcu2_serial = self.init_serial(self.mcu2_port)

        # ROS interfaces
        self.current_angles_pub = self.create_publisher(LampCurrentAngles, 'current_angles', 10)
        self.mcu_status_pub = self.create_publisher(String, 'mcu_status', 10)

        self.create_subscription(LampJointCommands, 'joint_commands', self.joint_command_callback, 10)
        self.create_subscription(Int32, 'light_mode', self.light_mode_callback, 10)

        self.create_timer(2.0, self.check_mcu_status)

        self.get_logger().info("Controller Communications Node is Running (Half-Duplex Mode).")

    def init_serial(self, port):
        try:
            ser = serial.Serial(port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"Connected to {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return None

    def joint_command_callback(self, msg):
        angles = LampCurrentAngles()

        # Send to MCU1: <joint1,joint2,joint3,joint4,lightmode>
        if self.mcu1_serial:
            try:
                mcu1_cmd = f"<{msg.joint_1},{msg.joint_2},{msg.joint_3},{msg.joint_4},{self.light_mode}>\n"
                self.mcu1_serial.reset_input_buffer()
                self.mcu1_serial.write(mcu1_cmd.encode())
                self.get_logger().debug(f"Sent to MCU1: {mcu1_cmd.strip()}")

                response = self.read_serial_line(self.mcu1_serial)
                if response:
                    values = response.split(',')
                    if len(values) == 5:
                        angles.joint_1 = float(values[0])
                        angles.joint_2 = float(values[1])
                        angles.joint_3 = float(values[2])
                        angles.joint_4 = float(values[3])
                        # values[4] is light mode feedback (optional to store)
                    else:
                        self.get_logger().warn(f"Unexpected MCU1 response: {response}")
                else:
                    self.get_logger().warn("No response from MCU1.")

            except Exception as e:
                self.get_logger().error(f"Error communicating with MCU1: {e}")

        # Send to MCU2: <joint0>
        if self.mcu2_serial:
            try:
                mcu2_cmd = f"<{msg.joint_0}>\n"
                self.mcu2_serial.reset_input_buffer()
                self.mcu2_serial.write(mcu2_cmd.encode())
                self.get_logger().debug(f"Sent to MCU2: {mcu2_cmd.strip()}")

                response = self.read_serial_line(self.mcu2_serial)
                if response:
                    angles.joint_0 = float(response)
                else:
                    self.get_logger().warn("No response from MCU2.")

            except Exception as e:
                self.get_logger().error(f"Error communicating with MCU2: {e}")

        # Publish merged result
        self.current_angles_pub.publish(angles)

    def read_serial_line(self, ser):
        try:
            line = ser.readline().decode().strip()
            if line.startswith('<') and line.endswith('>'):
                return line[1:-1]
            return ""
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")
            return ""

    def light_mode_callback(self, msg):
        try:
            self.light_mode = int(msg.data)
            self.get_logger().info(f"Light mode set to {self.light_mode}")
        except ValueError as e:
            self.get_logger().error(f"Invalid light mode: {e}")

    def check_mcu_status(self):
        status = []
        if not self.mcu1_serial or not self.mcu1_serial.is_open:
            status.append("MCU1 disconnected")
        if not self.mcu2_serial or not self.mcu2_serial.is_open:
            status.append("MCU2 disconnected")

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
