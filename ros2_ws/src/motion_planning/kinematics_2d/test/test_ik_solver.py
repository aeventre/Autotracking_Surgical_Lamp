import rclpy
import pytest
import math
from unittest.mock import MagicMock
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from kinematics_2d.ik_node_2d import IKSolver

@pytest.fixture
def node():
    """Fixture to initialize and return the IKSolver node."""
    rclpy.init()
    node = IKSolver()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_ik_valid_target(node):
    """Test if the IK solver correctly calculates joint angles for a reachable target."""
    target = Point()
    target.x = 0.3  # Adjust based on realistic reachable values
    target.y = 0.2

    # Mock the publisher
    node.publisher.publish = MagicMock()

    # Call IK solver
    node.ik_callback(target)

    # Get the published angles
    published_msg = node.publisher.publish.call_args[0][0]
    theta1, theta2, theta3, theta4, theta5 = published_msg.data

    # Expected angles (approximate, adjust if needed)
    expected_theta1 = math.atan2(0.2, 0.3) - math.atan2(
        node.L2 * math.sqrt(1 - ((0.3**2 + 0.2**2 - node.L1**2 - node.L2**2) / (2 * node.L1 * node.L2))**2),
        node.L1 + node.L2 * ((0.3**2 + 0.2**2 - node.L1**2 - node.L2**2) / (2 * node.L1 * node.L2))
    )
    expected_theta2 = math.atan2(
        math.sqrt(1 - ((0.3**2 + 0.2**2 - node.L1**2 - node.L2**2) / (2 * node.L1 * node.L2))**2),
        (0.3**2 + 0.2**2 - node.L1**2 - node.L2**2) / (2 * node.L1 * node.L2)
    )

    # Check computed angles with tolerance (consider adjusting tolerance if needed)
    assert math.isclose(theta1, expected_theta1, abs_tol=0.1)
    assert math.isclose(theta2, expected_theta2, abs_tol=0.1)
    assert theta3 == 0.0
    assert theta4 == 0.0
    assert theta5 == 0.0

def test_ik_target_out_of_reach(node):
    """Test if the solver correctly handles out-of-reach targets."""
    target = Point()
    target.x = 10.0  # Clearly out of reach
    target.y = 10.0

    node.publisher.publish = MagicMock()
    # Mock logger
    node.get_logger = MagicMock()

    node.ik_callback(target)

    # Ensure no message was published
    node.publisher.publish.assert_not_called()
    # Ensure the warning was logged
    node.get_logger().warn.assert_called_with("Target out of reach!")
