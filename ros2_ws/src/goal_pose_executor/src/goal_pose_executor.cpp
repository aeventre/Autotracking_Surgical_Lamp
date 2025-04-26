#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>
#include <string>

class GoalPoseExecutor
{
public:
  GoalPoseExecutor(const rclcpp::Node::SharedPtr &node)
    : node_(node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_control");

    move_group_->setPlanningPipelineId("stomp");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(10);                 // ⬆ Increased planning time
    move_group_->setNumPlanningAttempts(10);           // ⬆ Increased attempts
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setGoalPositionTolerance(0.02);        // ⬆ Increased position tolerance
    move_group_->setGoalOrientationTolerance(0.5);      // ⬆ Increased orientation tolerance

    joint_command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands", 10);
    planning_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/planning_status", 10);

    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GoalPoseExecutor::goalPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "GoalPoseExecutor initialized.");
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received new goal pose.");

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group_->getEndEffectorLink();
    ocm.header.frame_id = msg->header.frame_id;
    ocm.orientation = msg->pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.3;  // ⬆ Slightly more lenient
    ocm.absolute_y_axis_tolerance = 0.3;
    ocm.absolute_z_axis_tolerance = 0.3;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_->setPathConstraints(constraints);

    move_group_->setPoseTarget(*msg, move_group_->getEndEffectorLink());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    std_msgs::msg::Bool planning_result;
    planning_result.data = success;
    planning_status_pub_->publish(planning_result);

    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning succeeded, publishing final joint positions.");
      std_msgs::msg::Float64MultiArray joint_array;

      for (const auto &position : plan.trajectory.joint_trajectory.points.back().positions)
      {
        double deg = position * (180.0 / M_PI);
        joint_array.data.push_back(std::round(deg * 100.0) / 100.0);
      }

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("goal_pose_executor");
  GoalPoseExecutor executor(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
