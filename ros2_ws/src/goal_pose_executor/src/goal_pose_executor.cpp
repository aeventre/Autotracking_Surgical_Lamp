#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <map>

class GoalPoseExecutor
{
public:
  GoalPoseExecutor(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    RCLCPP_INFO(node_->get_logger(), "Initializing MoveGroupInterface...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "arm_control");

    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(2.0);
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    joint_command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/joint_commands", 10);

    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GoalPoseExecutor::goalPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "GoalPoseExecutor is ready and listening on /goal_pose.");
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received new goal pose.");

    // Orientation constraint on Z axis
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group_->getEndEffectorLink();
    ocm.header.frame_id = msg->header.frame_id;
    ocm.orientation = msg->pose.orientation;
    ocm.absolute_x_axis_tolerance = M_PI;
    ocm.absolute_y_axis_tolerance = M_PI;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_->setPathConstraints(constraints);
    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning succeeded. Publishing joint commands...");

      if (plan.trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory plan is empty. Nothing to publish.");
        return;
      }

      const auto& point = plan.trajectory.joint_trajectory.points.back();
      const auto& names = plan.trajectory.joint_trajectory.joint_names;
      const auto& positions = point.positions;

      std::map<std::string, double> joint_map;
      for (size_t i = 0; i < names.size(); ++i)
      {
        joint_map[names[i]] = positions[i];
      }

      // Fill joint values in the correct order
      std_msgs::msg::Float64MultiArray joint_array;
      joint_array.data = {
        joint_map["joint_0"],
        joint_map["joint_1"],
        joint_map["joint_2"],
        joint_map["joint_3"],
        joint_map["joint_4"]
      };

      joint_command_pub_->publish(joint_array);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
    }

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("goal_pose_executor");
  GoalPoseExecutor executor(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}