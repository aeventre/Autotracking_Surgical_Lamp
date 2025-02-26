import rclpy
from rclpy.node import Node
from std_msgs import String


class Serial_Coms(Node):

    def __init__(self):
        super.__init__('serial_coms')