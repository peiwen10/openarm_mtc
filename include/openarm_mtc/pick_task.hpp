#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>

#include "openarm_mtc/grasp.hpp"
#include "openarm_mtc/task_types.hpp"

namespace mtc = moveit::task_constructor;

namespace openarm_mtc
{

    enum class Arm
    {
        LEFT,
        RIGHT
    };

    class PickTask
    {
    public:
        PickTask(
            const rclcpp::Node::SharedPtr &node,
            Arm arm,
            const PickObject &object,
            const std::shared_ptr<GraspProvider> &grasp_provider);

        bool planAndExecute();

    private:
        // Configure the MoveIt groups and end-effector for the selected arm.
        void configureArm();

        // Add the detected object to the MoveIt PlanningScene.
        bool setupPlanningScene();

        // Build the complete MTC pick pipeline.
        mtc::Task createTask();

        rclcpp::Node::SharedPtr node_;
        Arm arm_;
        PickObject object_;
        std::shared_ptr<GraspProvider> grasp_provider_;

        std::string arm_group_;
        std::string gripper_group_;
        std::string eef_frame_;
        std::string eef_name_;

        mtc::Task task_;
    };

} // namespace openarm_mtc