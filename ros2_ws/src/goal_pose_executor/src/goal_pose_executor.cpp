#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm_control");

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(2.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Define a reachable target pose
  auto const arm_target_pose = [&node] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.1;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = -0.32;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;  // Make sure this is a valid quaternion
    return msg;
  }();

  // Set a path constraint that only cares about Z-axis alignment
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = arm_group_interface.getEndEffectorLink();
  ocm.header.frame_id = arm_target_pose.header.frame_id;
  ocm.orientation = arm_target_pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = M_PI;  // allow full roll
  ocm.absolute_y_axis_tolerance = M_PI;  // allow full pitch
  ocm.absolute_z_axis_tolerance = 0.1;   // slightly care about Z-axis
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  arm_group_interface.setPathConstraints(constraints);

  // Apply the pose target
  arm_group_interface.setPoseTarget(arm_target_pose);

  // Plan
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute
  if (success)
  {
    RCLCPP_INFO(logger, "Planning succeeded. Executing...");
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
