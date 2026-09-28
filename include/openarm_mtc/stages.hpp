#pragma once

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

namespace openarm_mtc
{

    std::unique_ptr<
        moveit::task_constructor::stages::MoveTo>
    createMoveToStage(
        const std::string &name,
        const std::shared_ptr<
            moveit::task_constructor::solvers::PlannerInterface> &planner,
        const std::string &group,
        const std::string &goal);

    std::unique_ptr<
        moveit::task_constructor::stages::MoveTo>
    createCartesianMoveStage(
        const std::string &name,
        const std::shared_ptr<
            moveit::task_constructor::solvers::CartesianPath> &planner,
        const std::string &group);

}