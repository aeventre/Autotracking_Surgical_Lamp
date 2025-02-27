import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

class SerialComs(Node):

    def __init__(self):
        super().__init__('serial_coms')
        self.subscription = self.create_subscription(
            Float64MultiArray,  # Match the message type from IKSolver
            '/joint_angles',  # Same topic name
            self.send_serial,
            10)
        self.subscription  

    def send_serial(self, joint_angles_msg):
        # Convert list of floats to a comma-separated string
        angles_str = ','.join(f"{angle:.2f}" for angle in joint_angles_msg.data)
        self.get_logger().info(f"Sending to serial: {angles_str}")  # Debug log
        ser.write((angles_str + '\n').encode())  # Encode to bytes and send

def main(args=None):
    rclpy.init(args=args)
    serial_coms = SerialComs()
    
    try:
        rclpy.spin(serial_coms)
    except KeyboardInterrupt:
        pass
    
    serial_coms.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
