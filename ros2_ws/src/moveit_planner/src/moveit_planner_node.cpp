#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

class MoveItPlannerNode : public rclcpp::Node
{
public:
  MoveItPlannerNode() : Node("moveit_planner_node")
  {
    RCLCPP_INFO(this->get_logger(), "MoveIt planner node started.");

    // Create planning interface for the group controlling your lamp
    moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "lamp_arm");  // <- change this if needed

    // Set a target pose (hardcoded)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.3;
    target_pose.orientation.w = 1.0;  // No rotation

    move_group.setPoseTarget(target_pose);

    // Plan to the target
    auto const [success, plan] = move_group.plan();

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Motion plan succeeded!");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Motion plan failed.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
