#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace openarm_mtc
{

    struct PickParameters
    {
        geometry_msgs::msg::PoseStamped object_pose;

        std::string arm_group;
        std::string gripper_group;
        std::string eef_frame;
    };

    // Object information passed from perception to the pick task.
    struct PickObject
    {
        std::string id = "object";
        geometry_msgs::msg::PoseStamped pose;
        double size_x = 0.05;
        double size_y = 0.05;
        double size_z = 0.10;
    };

} // namespace openarm_mtc