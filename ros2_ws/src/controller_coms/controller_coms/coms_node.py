import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String


# Format for sending to mcu1 <commandAngle1,commandAngle2>
# Format for recieving from mcu1 <currentAngle0,currentAngle1>
# Format for sending to mcu2 <commandAngle2,commandAngle3,commandAngle4,lightmode>
# Lightmode is a number from 0 to 2
# Format for recieving from mcu2 <currentAngle2,currentAngle3,currentAngle4>

class ControllerComsNode(Node):
    def __init__(self):
        super().__init__('controller_coms')

        # Serial Port Setup
        self.mcu1_port = '/dev/ttyUSB0'
        self.mcu2_port = '/dev/ttyUSB1'
        self.baud_rate = 115200

        # Initialize Serial Connections
        self.mcu1_serial = self.init_serial(self.mcu1_port)
        self.mcu2_serial = self.init_serial(self.mcu2_port)

        # ROS Publishers (Feedback from MCUs)
        self.publisher_mcu1 = self.create_publisher(String, 'mcu1_feedback', 10)
        self.publisher_mcu2 = self.create_publisher(String, 'mcu2_feedback', 10)

        # ROS Subscribers (Sending Commands)
        self.subscription_mcu1 = self.create_subscription(
            String, 'mcu1_command', self.send_to_mcu1, 10
        )
        self.subscription_mcu2 = self.create_subscription(
            String, 'mcu2_command', self.send_to_mcu2, 10
        )

        # Start Threads to Continuously Read from MCUs
        threading.Thread(target=self.read_from_mcu, args=(self.mcu1_serial, self.publisher_mcu1), daemon=True).start()
        threading.Thread(target=self.read_from_mcu, args=(self.mcu2_serial, self.publisher_mcu2), daemon=True).start()

        self.get_logger().info("Controller Communications Node is Running.")

    def init_serial(self, port):
        """Initialize Serial Connection."""
        try:
            ser = serial.Serial(port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {port}")
            return ser
        except serial.SerialException:
            self.get_logger().error(f"Failed to open {port}")
            return None

    def send_to_mcu1(self, msg):
        """Send formatted command to MCU1."""
        if self.mcu1_serial:
            try:
                command = msg.data.strip()
                self.mcu1_serial.write((command + '\n').encode())
                self.get_logger().info(f"Sent to MCU1: {command}")
            except Exception as e:
                self.get_logger().error(f"Error sending to MCU1: {e}")

    def send_to_mcu2(self, msg):
        """Send formatted command to MCU2."""
        if self.mcu2_serial:
            try:
                command = msg.data.strip()
                self.mcu2_serial.write((command + '\n').encode())
                self.get_logger().info(f"Sent to MCU2: {command}")
            except Exception as e:
                self.get_logger().error(f"Error sending to MCU2: {e}")

    def read_from_mcu(self, ser, publisher):
        """Read and publish formatted feedback from MCU."""
        while rclpy.ok():
            try:
                if ser and ser.in_waiting:
                    data = ser.readline().decode().strip()
                    msg = String()
                    msg.data = data
                    publisher.publish(msg)
                    self.get_logger().info(f"Received: {data}")
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerComsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
