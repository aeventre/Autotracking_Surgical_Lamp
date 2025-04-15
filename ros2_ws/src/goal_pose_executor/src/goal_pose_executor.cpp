#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>


class GoalPoseExecutor : public rclcpp::Node
{
public:
    GoalPoseExecutor() : Node("goal_pose_executor")
    {
        using std::placeholders::_1;

        // Set up MoveGroupInterface for your MoveIt planning group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "lamp_arm");

        // Subscriber to goal pose
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&GoalPoseExecutor::goalPoseCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Goal Pose Executor initialized.");
    }

private:
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal pose. Attempting IK...");

        // Set the target pose
        move_group_->setPoseTarget(*msg);

        // Solve IK
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "IK successful. Executing...");
            move_group_->execute(plan);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "IK failed for the given pose.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPoseExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
