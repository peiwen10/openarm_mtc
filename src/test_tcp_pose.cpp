#include <cstddef>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "test_tcp_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    moveit::planning_interface::MoveGroupInterface move_group(node, "left_arm");

    move_group.setPoseReferenceFrame("world");
    move_group.setEndEffectorLink("openarm_left_hand");

    // Test target pose. Start with the current TCP pose to verify IK/planning.
    geometry_msgs::msg::Pose target;
    target.position.x = 0.318;
    target.position.y = 0.154;
    target.position.z = 0.455;
    target.orientation.x = 0.732;
    target.orientation.y = 0.002;
    target.orientation.z = 0.681;
    target.orientation.w = 0.001;

    move_group.setPoseTarget(target);

    RCLCPP_INFO(node->get_logger(),
                "Planning to TCP pose: [%.3f, %.3f, %.3f]",
                target.position.x,
                target.position.y,
                target.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto result = move_group.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
                "Planning succeeded. Trajectory points: %zu",
                plan.trajectory_.joint_trajectory.points.size());

    RCLCPP_INFO(node->get_logger(), "Executing...");

    result = move_group.execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Execution failed.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Execution succeeded.");

    rclcpp::shutdown();
    return 0;
}