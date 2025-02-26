import rclpy
from rclpy.node import Node
from std_msgs import String

import serial
ser = serial.Serial('/dev/ttyUSB0')

class SerialComs(Node):

    def __init__(self):
        super().__init__('serial_coms')
        self.subscription = self.create_subscription(
            String,
            'joint_angles', # subscribes to joint angle topic
            self.send_serial, # runs this function every time data is recieved
            10)
        self.subscription
        
    def send_serial(self,joint_angles):
        ser.write(joint_angles)
        

def main(args=None):

    rclpy.init(args=args)
    serial_coms = SerialComs()
    rclpy.spin(serial_coms)

    serial_coms.destroy_node()
    rclpy.shutdown



if __name__ == '__main__':
    main()