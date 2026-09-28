#include "openarm_mtc/mtc_common.hpp"

namespace mtc = moveit::task_constructor;
namespace openarm_mtc
{
    MTCCommon::MTCCommon(
        const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
    }
    std::shared_ptr<mtc::solvers::PipelinePlanner>
    MTCCommon::pipelinePlanner()
    {
        return std::make_shared<
            mtc::solvers::PipelinePlanner>(node_);
    }

    std::shared_ptr<mtc::solvers::CartesianPath>
    MTCCommon::cartesianPlanner()
    {
        auto planner =
            std::make_shared<
                mtc::solvers::CartesianPath>();

        planner->setMaxVelocityScalingFactor(0.5);
        planner->setMaxAccelerationScalingFactor(0.5);
        planner->setStepSize(0.01);

        return planner;
    }
}