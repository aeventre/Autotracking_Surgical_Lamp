import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize
import sys

class MoveItPlannerNode(Node):
    def __init__(self):
        super().__init__('moveit_planner_node')

        # Initialize MoveIt Commander
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("lamp_arm")  # <-- Change this to match your planning group name

        self.get_logger().info("MoveIt planner node initialized")

        # Subscribes to pose targets
        self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)

        # Publishes joint angle plan
        self.plan_pub = self.create_publisher(Float64MultiArray, '/planned_joint_angles', 10)

    def target_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received target pose:\n{msg.pose}")

        self.group.set_pose_target(msg)

        plan = self.group.plan()
        if plan and plan[0]:  # Plan succeeded
            self.get_logger().info("Motion plan successful")

            # Extract final joint goal from plan
            joint_values = plan[1].joint_trajectory.points[-1].positions

            # Publish as Float64MultiArray
            joint_array = Float64MultiArray()
            joint_array.data = list(joint_values)
            self.plan_pub.publish(joint_array)
        else:
            self.get_logger().warn("Motion plan failed")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
