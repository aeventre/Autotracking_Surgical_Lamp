import rclpy
from rclpy.node import Node
from std_msgs import String


class Serial_Coms(Node):

    def __init__(self):
        super.__init__('serial_coms')


def main(args=None):
    rclpy.init(args=args)
    serial_coms = Serial_Coms()
    rclpy.spin(serial_coms)

    serial_coms.destroy_node()
    rclpy.shutdown



if __name__ == '__main__':
    main()