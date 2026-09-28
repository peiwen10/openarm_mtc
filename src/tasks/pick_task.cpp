#include "openarm_mtc/pick_task.hpp"

#include <memory>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("openarm_mtc.pick_task");

namespace openarm_mtc
{

    PickTask::PickTask(
        const rclcpp::Node::SharedPtr &node,
        Arm arm,
        const PickObject &object,
        const std::shared_ptr<GraspProvider> &grasp_provider)
        : node_(node), arm_(arm), object_(object), grasp_provider_(grasp_provider)
    {
        configureArm();
    }

    // Map the generic LEFT/RIGHT selection to the actual OpenArm MoveIt configuration.
    void PickTask::configureArm()
    {
        if (arm_ == Arm::LEFT)
        {
            arm_group_ = "left_arm";
            gripper_group_ = "left_gripper";
            eef_frame_ = "openarm_left_hand";
            eef_name_ = "left_ee";
        }
        else
        {
            arm_group_ = "right_arm";
            gripper_group_ = "right_gripper";
            eef_frame_ = "openarm_right_hand";
            eef_name_ = "right_ee";
        }
    }

    // Add the detected object as a box to the MoveIt PlanningScene.
    bool PickTask::setupPlanningScene()
    {
        if (object_.pose.header.frame_id.empty())
        {
            RCLCPP_ERROR(LOGGER, "Object pose has no frame_id.");
            return false;
        }

        moveit_msgs::msg::CollisionObject object;
        object.id = object_.id;
        object.header.frame_id = object_.pose.header.frame_id;

        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        object.primitives[0].dimensions = {
            object_.size_x, object_.size_y, object_.size_z};

        object.primitive_poses.resize(1);
        object.primitive_poses[0] = object_.pose.pose;

        moveit::planning_interface::PlanningSceneInterface
            planning_scene_interface;

        // Clear any copy left behind by a previous pick (world or still
        // attached to the end effector) so the new object is added to a clean
        // scene.
        planning_scene_interface.removeCollisionObjects({ object_.id });

        // Only detach if the object is actually attached, otherwise move_group
        // logs "Attached body not found".
        const auto attached_objects =
            planning_scene_interface.getAttachedObjects();
        if (attached_objects.find(object_.id) != attached_objects.end())
        {
            moveit_msgs::msg::AttachedCollisionObject detach;
            detach.link_name = eef_frame_;
            detach.object.id = object_.id;
            detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            auto detach_pub = node_->create_publisher<
                moveit_msgs::msg::AttachedCollisionObject>(
                "/attached_collision_object", 10);
            detach_pub->publish(detach);
        }

        // Give the planning scene monitor time to process the removal before
        // the task reads the scene.
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        planning_scene_interface.applyCollisionObject(object);

        RCLCPP_INFO(
            LOGGER, "Added object '%s' to planning scene.",
            object_.id.c_str());

        return true;
    }

    // Plan and execute the complete MTC pick task.
    bool PickTask::planAndExecute()
    {
        if (!setupPlanningScene())
        {
            return false;
        }

        task_ = createTask();

        try
        {
            task_.init();
        }
        catch (mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(
                LOGGER, "MTC task initialization failed:\n"
                            << e);
            return false;
        }

        RCLCPP_INFO(LOGGER, "Planning pick task...");

        if (!task_.plan(5))
        {
            RCLCPP_ERROR(LOGGER, "Pick task planning failed.");
            return false;
        }

        RCLCPP_INFO(LOGGER, "Pick task planning succeeded.");

        task_.introspection().publishSolution(
            *task_.solutions().front());

        RCLCPP_INFO(LOGGER, "Executing pick task...");

        auto result = task_.execute(*task_.solutions().front());

        if (result.val !=
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(
                LOGGER, "Pick task execution failed. Error code: %d",
                result.val);
            return false;
        }

        RCLCPP_INFO(LOGGER, "Pick task completed successfully.");
        return true;
    }

    // Build the MTC pipeline: open -> pre-grasp -> approach -> close -> attach -> retract.
    mtc::Task PickTask::createTask()
    {
        mtc::Task task;
        task.stages()->setName("OpenArm pick task");
        task.loadRobotModel(node_);

        task.setProperty("group", arm_group_);
        task.setProperty("eef", eef_name_);
        task.setProperty("ik_frame", eef_frame_);

        // Planners used for joint-space and Cartesian motions.
        auto pipeline_planner =
            std::make_shared<mtc::solvers::PipelinePlanner>(node_);

        auto interpolation_planner =
            std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto cartesian_planner =
            std::make_shared<mtc::solvers::CartesianPath>();

        cartesian_planner->setMaxVelocityScalingFactor(0.3);
        cartesian_planner->setMaxAccelerationScalingFactor(0.3);
        cartesian_planner->setStepSize(0.01);

        // Start from the robot's current state.
        auto current_state =
            std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(current_state));

        // Open the gripper using the SRDF "open" named state.
        auto open_gripper =
            std::make_unique<mtc::stages::MoveTo>(
                "open gripper", interpolation_planner);
        open_gripper->setGroup(gripper_group_);
        open_gripper->setGoal("open");
        task.add(std::move(open_gripper));

        // Generate grasp candidates and build one alternative branch per candidate.
        // MTC's Alternatives container evaluates every branch and keeps the first
        // complete solution, so the solver effectively tries the whole ranked
        // list of grasp poses (which is exactly how anygrasp's list will plug in
        // later).
        std::vector<Grasp> candidates =
            grasp_provider_->generate(object_);
        if (candidates.empty())
        {
            RCLCPP_ERROR(LOGGER, "No grasp candidates generated.");
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Generated %zu grasp candidate(s).",
                        candidates.size());
        }

        // Allow the end-effector assembly to collide with the object during
        // grasping. Placed before the alternatives so every branch shares it;
        // the pre-grasp is far from the object, so this does not affect it.
        auto allow_collision =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow hand object collision");

        std::vector<std::string> hand_links;
        // Allow the object to collide with the whole end-effector assembly: the
        // wrist link (link7), the hand, and every link attached to the wrist
        // (both fingers - one is on a passive joint that MoveIt does not add to
        // the gripper group - plus the TCP frame).
        const moveit::core::LinkModel* hand =
            task.getRobotModel()->getLinkModel(eef_frame_);
        if (const moveit::core::JointModel* hand_parent =
                hand ? hand->getParentJointModel() : nullptr)
        {
            if (const moveit::core::LinkModel* wrist =
                    hand_parent->getParentLinkModel())
            {
                hand_links.push_back(wrist->getName());
                for (const auto* joint : wrist->getChildJointModels())
                    if (const auto* child = joint->getChildLinkModel())
                        hand_links.push_back(child->getName());
            }
        }
        hand_links.push_back(eef_frame_);

        std::sort(hand_links.begin(), hand_links.end());
        hand_links.erase(std::unique(hand_links.begin(), hand_links.end()), hand_links.end());

        allow_collision->allowCollisions(object_.id, hand_links, true);

        task.add(std::move(allow_collision));

        auto alternatives =
            std::make_unique<mtc::Alternatives>("grasp candidates");

        for (size_t i = 0; i < candidates.size(); ++i)
        {
            const Grasp& g = candidates[i];
            auto branch = std::make_unique<mtc::SerialContainer>(
                "candidate " + std::to_string(i));

            // Move to the pre-grasp pose of this candidate.
            auto move_to_pre_grasp =
                std::make_unique<mtc::stages::MoveTo>(
                    "move to pre-grasp", pipeline_planner);
            move_to_pre_grasp->setGroup(arm_group_);
            move_to_pre_grasp->setIKFrame(eef_frame_);
            move_to_pre_grasp->setGoal(g.pre_grasp_pose);
            branch->insert(std::move(move_to_pre_grasp));

            // Straight Cartesian approach from the pre-grasp to the grasp pose.
            const double dx = g.grasp_pose.pose.position.x -
                              g.pre_grasp_pose.pose.position.x;
            const double dy = g.grasp_pose.pose.position.y -
                              g.pre_grasp_pose.pose.position.y;
            const double dz = g.grasp_pose.pose.position.z -
                              g.pre_grasp_pose.pose.position.z;
            const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = object_.pose.header.frame_id;
            dir.vector.x = dx / dist;
            dir.vector.y = dy / dist;
            dir.vector.z = dz / dist;

            auto approach =
                std::make_unique<mtc::stages::MoveRelative>(
                    "approach object", cartesian_planner);
            approach->setGroup(arm_group_);
            approach->setIKFrame(eef_frame_);
            approach->setDirection(dir);
            approach->setMinMaxDistance(dist, dist);
            branch->insert(std::move(approach));

            // Close the gripper using the SRDF "closed" named state.
            auto close_gripper =
                std::make_unique<mtc::stages::MoveTo>(
                    "close gripper", interpolation_planner);
            close_gripper->setGroup(gripper_group_);
            close_gripper->setGoal("closed");
            branch->insert(std::move(close_gripper));

            // Attach the object to the end effector so it moves with the robot.
            auto attach_object =
                std::make_unique<mtc::stages::ModifyPlanningScene>(
                    "attach object");
            attach_object->attachObject(object_.id, eef_frame_);
            branch->insert(std::move(attach_object));

            alternatives->insert(std::move(branch));
        }

        if (candidates.empty())
        {
            // Keep the Alternatives container non-empty (otherwise task init
            // fails with "no children"). Add a single branch whose goal is far
            // outside the workspace, so the task initialises but cannot plan; the
            // "No grasp candidates generated" message above explains the cause.
            auto branch =
                std::make_unique<mtc::SerialContainer>("no candidates");
            auto unreachable =
                std::make_unique<mtc::stages::MoveTo>(
                    "unreachable", pipeline_planner);
            unreachable->setGroup(arm_group_);
            unreachable->setIKFrame(eef_frame_);
            geometry_msgs::msg::PoseStamped far;
            far.header.frame_id = object_.pose.header.frame_id;
            far.pose.position.x = 10.0;
            far.pose.position.y = 10.0;
            far.pose.position.z = 10.0;
            far.pose.orientation.w = 1.0;
            unreachable->setGoal(far);
            branch->insert(std::move(unreachable));
            alternatives->insert(std::move(branch));
        }

        task.add(std::move(alternatives));

        // Move straight upward after grasping (once, on the winning branch).
        auto retract =
            std::make_unique<mtc::stages::MoveRelative>(
                "retract", cartesian_planner);
        retract->setGroup(arm_group_);
        retract->setIKFrame(eef_frame_);
        retract->setMinMaxDistance(0.10, 0.10);

        geometry_msgs::msg::Vector3Stamped retract_direction;
        retract_direction.header.frame_id = object_.pose.header.frame_id;
        retract_direction.vector.z = 1.0;
        retract->setDirection(retract_direction);
        task.add(std::move(retract));

        return task;
    }

} // namespace openarm_mtc