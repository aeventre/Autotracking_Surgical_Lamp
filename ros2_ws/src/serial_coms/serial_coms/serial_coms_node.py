import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class SerialComs(Node):

    def __init__(self):
        super().__init__('serial_coms')

        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Initialize serial communication
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial connected: {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None  # Prevent further writes if failed

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_angles',
            self.send_serial,
            10
        )

    def send_serial(self, joint_angles_msg):
        if self.ser:  # Ensure serial is open before sending
            angles_str = ','.join(f"{angle:.2f}" for angle in joint_angles_msg.data)
            self.get_logger().info(f"Sending to serial: {angles_str}")
            self.ser.write((angles_str + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    serial_coms = SerialComs()

    try:
        rclpy.spin(serial_coms)
    except KeyboardInterrupt:
        pass
    finally:
        if serial_coms.ser:
            serial_coms.ser.close()  # Close serial connection on shutdown
        serial_coms.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
