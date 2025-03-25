import rclpy
from rclpy.node import Node
import serial
import threading
from surg_lamp_msgs.msg import LampJointCommands, LampCurrentAngles
from std_msgs.msg import Int32, String

class ControllerComsNode(Node):
    def __init__(self):
        super().__init__('controller_coms')

        self.mcu1_port = '/dev/ttyUSB1'
        self.mcu2_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.light_mode = 0  # Default light mode
        
        self.mcu_status_pub = self.create_publisher(String, 'mcu_status', 10)
        self.create_timer(2.0, self.check_mcu_status)


        # Initialize serial connections
        self.mcu1_serial = self.init_serial(self.mcu1_port)
        self.mcu2_serial = self.init_serial(self.mcu2_port)

        # ROS interfaces
        self.current_angles_pub = self.create_publisher(LampCurrentAngles, 'current_angles', 10)

        self.create_subscription(LampJointCommands, 'joint_commands', self.joint_command_callback, 10)
        self.create_subscription(Int32, 'light_mode', self.light_mode_callback, 10)

        # Start serial reader threads
        threading.Thread(target=self.read_from_mcu, args=(self.mcu1_serial, "mcu1"), daemon=True).start()
        threading.Thread(target=self.read_from_mcu, args=(self.mcu2_serial, "mcu2"), daemon=True).start()

        self.get_logger().info("Controller Communications Node is Running.")

    def init_serial(self, port):
        try:
            ser = serial.Serial(port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return None

    def joint_command_callback(self, msg):
        try:
            mcu1_cmd = f"<{msg.joint_0},{msg.joint_1}>\n"
            mcu2_cmd = f"<{msg.joint_2},{msg.joint_3},{msg.joint_4},{self.light_mode}>\n"

            if self.mcu1_serial:
                self.mcu1_serial.write(mcu1_cmd.encode())
                self.get_logger().debug(f"Sent to MCU1: {mcu1_cmd.strip()}")

            if self.mcu2_serial:
                self.mcu2_serial.write(mcu2_cmd.encode())
                self.get_logger().debug(f"Sent to MCU2: {mcu2_cmd.strip()}")

        except Exception as e:
            self.get_logger().error(f"Error sending joint commands: {e}")

    def light_mode_callback(self, msg):
        try:
            self.light_mode = int(msg.data)
            self.get_logger().info(f"Light mode set to {self.light_mode}")
        except ValueError as e:
            self.get_logger().error(f"Invalid light mode: {e}")

    def read_from_mcu(self, ser, mcu_id):
        while rclpy.ok():
            try:
                if ser and ser.in_waiting:
                    line = ser.readline().decode().strip()

                    if line.startswith("<") and line.endswith(">"):
                        values = line[1:-1].split(',')
                        msg = LampCurrentAngles()

                        # Update only the relevant fields and cache
                        if mcu_id == "mcu1" and len(values) == 2:
                            self.last_mcu1_values = (float(values[0]), float(values[1]))
                        elif mcu_id == "mcu2" and len(values) == 3:
                            self.last_mcu2_values = (float(values[0]), float(values[1]), float(values[2]))
                        else:
                            self.get_logger().warn(f"Unexpected data from {mcu_id}: {line}")
                            continue

                        # Combine latest known values from both MCUs
                        if hasattr(self, 'last_mcu1_values'):
                            msg.joint_0, msg.joint_1 = self.last_mcu1_values
                        if hasattr(self, 'last_mcu2_values'):
                            msg.joint_2, msg.joint_3, msg.joint_4 = self.last_mcu2_values

                        self.current_angles_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error reading from {mcu_id}: {e}")
                
    def check_mcu_status(self):
        status = []
        if not self.mcu1_serial or not self.mcu1_serial.is_open:
            status.append("MCU1 disconnected")
        if not self.mcu2_serial or not self.mcu2_serial.is_open:
            status.append("MCU2 disconnected")

        if not status:
            msg = "OK"
        else:
            msg = ", ".join(status)

        self.get_logger().warn(f"MCU status: {msg}") if msg != "OK" else None

        status_msg = String()
        status_msg.data = msg
        self.mcu_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerComsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
