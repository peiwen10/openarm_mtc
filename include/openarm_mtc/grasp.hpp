#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "openarm_mtc/task_types.hpp"

namespace openarm_mtc
{

    // A single grasp candidate produced by a GraspProvider.
    struct Grasp
    {
        geometry_msgs::msg::PoseStamped pre_grasp_pose;
        geometry_msgs::msg::PoseStamped grasp_pose;
        double approach_distance{ 0.15 };
        double retract_distance{ 0.10 };
        double score{ 0.0 };  // optional confidence, e.g. from anygrasp
    };

    // Shared IK reachability check used to filter grasp candidates. Uses the
    // /compute_ik service with the current robot state as the seed, and caches
    // service readiness and the seed so repeated calls are cheap.
    class IKValidator
    {
    public:
        IKValidator(const std::string& arm_group, const std::string& eef_frame);

        // If avoid_collisions is true, the state must also be collision-free
        // (used for the pre-grasp); otherwise only kinematics are checked.
        bool ikReachable(const geometry_msgs::msg::PoseStamped& hand_pose,
                         bool avoid_collisions = false) const;

    private:
        std::string arm_group_;
        std::string eef_frame_;

        mutable std::shared_ptr<rclcpp::Node> ik_node_;
        mutable std::shared_ptr<rclcpp::Client<moveit_msgs::srv::GetPositionIK>> ik_client_;
        mutable std::shared_ptr<rclcpp::Client<moveit_msgs::srv::GetPlanningScene>> scene_client_;
        mutable sensor_msgs::msg::JointState seed_joint_state_;
        mutable bool services_ready_ = false;
    };

    // Source of grasp candidates. The pick pipeline only depends on this
    // interface, so swapping the grasp source (hardcoded today, anygrasp later)
    // requires no changes to the task stages.
    class GraspProvider
    {
    public:
        virtual ~GraspProvider() = default;

        // Return grasp candidates for the given object, already filtered for
        // gripper opening width and (optionally) IK reachability.
        virtual std::vector<Grasp> generate(const PickObject& object) const = 0;
    };

    // Generates candidates by sweeping approach directions about world Z (yaw)
    // and rolling the gripper frame in fixed increments around its forward
    // approach axis. Candidates are filtered against the maximum gripper opening
    // width and then by a quick IK reachability check, and returned in generation
    // order (front approach first) as the ranking.
    class HardcodedGraspProvider : public GraspProvider
    {
    public:
        HardcodedGraspProvider(
            const std::string& arm_group,
            const std::string& eef_frame);

        std::vector<Grasp> generate(const PickObject& object) const override;

    private:
        std::string arm_group_;
        std::string eef_frame_;
        mutable IKValidator validator_;
    };

    // Consumes grasp poses published by a perception/grasp-detection node
    // (e.g. anygrasp) on /grasp_poses as a geometry_msgs::msg::PoseArray. Each
    // pose is the gripper frame at the grasp; the pre-grasp is derived by
    // offsetting back along the grasp's approach axis (hand +Z). Poses must be
    // in the same frame as the object (the planning frame); candidates are
    // filtered for IK reachability like the hardcoded provider.
    class AnyGraspProvider : public GraspProvider
    {
    public:
        AnyGraspProvider(
            const rclcpp::Node::SharedPtr& node,
            const std::string& arm_group,
            const std::string& eef_frame);

        std::vector<Grasp> generate(const PickObject& object) const override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::string arm_group_;
        std::string eef_frame_;
        mutable IKValidator validator_;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_sub_;
        mutable geometry_msgs::msg::PoseArray latest_grasps_;
    };

} // namespace openarm_mtc