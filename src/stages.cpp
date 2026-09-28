#include "openarm_mtc/stages.hpp"

namespace mtc = moveit::task_constructor;

namespace openarm_mtc
{

    std::unique_ptr<mtc::stages::MoveTo>
    createMoveToStage(
        const std::string &name,
        const std::shared_ptr<
            mtc::solvers::PlannerInterface> &planner,
        const std::string &group,
        const std::string &goal)
    {

        auto stage =
            std::make_unique<mtc::stages::MoveTo>(
                name,
                planner);

        stage->setGroup(group);

        stage->setGoal(goal);

        return stage;
    }

    std::unique_ptr<mtc::stages::MoveTo>
    createCartesianMoveStage(
        const std::string &name,
        const std::shared_ptr<
            mtc::solvers::CartesianPath> &planner,
        const std::string &group)
    {

        auto stage =
            std::make_unique<mtc::stages::MoveTo>(
                name,
                planner);

        stage->setGroup(group);

        return stage;
    }

}