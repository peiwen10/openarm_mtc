#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>

namespace openarm_mtc
{

    class MTCCommon
    {

    public:
        explicit MTCCommon(
            const rclcpp::Node::SharedPtr &node);

        std::shared_ptr<
            moveit::task_constructor::solvers::PipelinePlanner>
        pipelinePlanner();

        std::shared_ptr<
            moveit::task_constructor::solvers::CartesianPath>
        cartesianPlanner();

    private:
        rclcpp::Node::SharedPtr node_;
    };

}