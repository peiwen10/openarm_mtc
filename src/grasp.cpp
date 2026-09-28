#include "openarm_mtc/grasp.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace openarm_mtc
{

    IKValidator::IKValidator(
        const std::string &arm_group,
        const std::string &eef_frame)
        : arm_group_(arm_group), eef_frame_(eef_frame)
    {
    }

    bool IKValidator::ikReachable(
        const geometry_msgs::msg::PoseStamped &hand_pose,
        bool avoid_collisions) const
    {
        if (!ik_node_)
        {
            ik_node_ = std::make_shared<rclcpp::Node>("grasp_provider_ik");
            ik_client_ = ik_node_->create_client<
                moveit_msgs::srv::GetPositionIK>("/compute_ik");
            scene_client_ = ik_node_->create_client<
                moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        }

        // One-time setup: wait for services and fetch the current robot state to
        // use as the IK seed (avoids move_group "empty JointState" warnings and
        // keeps the solver near the actual configuration).
        if (!services_ready_)
        {
            if (!ik_client_->wait_for_service(std::chrono::seconds(2)) ||
                !scene_client_->wait_for_service(std::chrono::seconds(2)))
            {
                return false;
            }
            auto scene_req = std::make_shared<
                moveit_msgs::srv::GetPlanningScene::Request>();
            scene_req->components.components =
                moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;
            auto scene_future = scene_client_->async_send_request(scene_req);
            if (rclcpp::spin_until_future_complete(
                    ik_node_, scene_future, std::chrono::seconds(3)) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                seed_joint_state_ =
                    scene_future.get()->scene.robot_state.joint_state;
            }
            services_ready_ = true;
        }

        auto request = std::make_shared<
            moveit_msgs::srv::GetPositionIK::Request>();
        request->ik_request.group_name = arm_group_;
        request->ik_request.pose_stamped = hand_pose;
        request->ik_request.ik_link_name = eef_frame_;
        request->ik_request.avoid_collisions = avoid_collisions;
        // Short timeout: reachable poses are found fast from the seed, and
        // unreachable ones are rejected quickly.
        request->ik_request.timeout.sec = 0;
        request->ik_request.timeout.nanosec = 100'000'000;
        if (!seed_joint_state_.name.empty())
            request->ik_request.robot_state.joint_state = seed_joint_state_;

        auto future = ik_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(
                ik_node_, future, std::chrono::seconds(2)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }
        return future.get()->error_code.val ==
               moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }

    HardcodedGraspProvider::HardcodedGraspProvider(
        const std::string &arm_group,
        const std::string &eef_frame)
        : arm_group_(arm_group), eef_frame_(eef_frame), validator_(arm_group, eef_frame)
    {
    }

    std::vector<Grasp> HardcodedGraspProvider::generate(
        const PickObject &object) const
    {
        // Maximum finger-to-finger opening: fingers at +-0.049 m when fully open.
        constexpr double kMaxGripperOpening = 0.098;
        constexpr double kSafetyFactor = 0.92;
        // Pre-grasp: the TCP (fingertip centre) is this far in front of the object
        // surface along the approach direction.
        constexpr double kApproachDistance = 0.15;
        // Hand frame -> TCP offset along hand +Z (the forward approach axis).
        constexpr double kTcpOffset = 0.0835;
        constexpr double kRollStepDeg = 15.0;
        // Cap the number of candidates so MTC does not spend excessive planning
        // time evaluating every branch; candidates are returned in generation
        // order (front approach first), which is also the ranking.
        constexpr size_t kMaxCandidates = 5;

        const Eigen::Vector3d object_center(
            object.pose.pose.position.x,
            object.pose.pose.position.y,
            object.pose.pose.position.z);

        // Object rotation (for projecting the closure axis into the object frame).
        Eigen::Matrix3d obj_rot(
            Eigen::Quaterniond(object.pose.pose.orientation.x,
                               object.pose.pose.orientation.y,
                               object.pose.pose.orientation.z,
                               object.pose.pose.orientation.w));

        std::vector<Grasp> candidates;

        // Explore one approach direction: probe reachability cheaply, then sample
        // rolls about the approach axis. Returns true when the cap is reached.
        auto consider_direction = [&](const Eigen::Vector3d &dir) -> bool {
            // Object half-extent along the approach direction (the surface the
            // hand approaches).
            const double extent =
                (object.size_x * std::abs(dir.x()) +
                 object.size_y * std::abs(dir.y()) +
                 object.size_z * std::abs(dir.z())) / 2.0;

            // Canonical frame mapping hand +Z (forward approach axis) -> dir.
            Eigen::Quaterniond base(
                Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir));

            // Cheap probe: try a few representative rolls; if none of their pre-grasps
            // is IK-reachable, skip the whole direction before sampling all rolls.
            bool any_reachable = false;
            for (const double probe_roll : { 0.0, 90.0, 180.0, 270.0 })
            {
                Eigen::Matrix3d ori =
                    base.toRotationMatrix() *
                    Eigen::AngleAxisd(probe_roll * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();
                Eigen::Vector3d pre =
                    object_center - kTcpOffset * ori.col(2) -
                    (kApproachDistance + extent) * dir;
                geometry_msgs::msg::PoseStamped probe;
                probe.header.frame_id = object.pose.header.frame_id;
                probe.pose.position.x = pre.x();
                probe.pose.position.y = pre.y();
                probe.pose.position.z = pre.z();
                Eigen::Quaterniond quat(ori);
                probe.pose.orientation.x = quat.x();
                probe.pose.orientation.y = quat.y();
                probe.pose.orientation.z = quat.z();
                probe.pose.orientation.w = quat.w();
                if (validator_.ikReachable(probe))
                {
                    any_reachable = true;
                    break;
                }
            }
            if (!any_reachable)
                return false;

            for (double deg = 0.0; deg < 360.0; deg += kRollStepDeg)
            {
                // Roll the gripper about its forward approach axis.
                Eigen::Matrix3d orientation =
                    base.toRotationMatrix() *
                    Eigen::AngleAxisd(deg * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();

                // Object thickness along the finger-closure axis (hand +Y).
                Eigen::Vector3d closure_world = orientation.col(1);
                Eigen::Vector3d closure_obj = obj_rot.transpose() * closure_world;
                const double thickness =
                    object.size_x * std::abs(closure_obj.x()) +
                    object.size_y * std::abs(closure_obj.y()) +
                    object.size_z * std::abs(closure_obj.z());
                if (thickness > kMaxGripperOpening * kSafetyFactor)
                    continue;

                // The grasp places the fingertips (TCP) at the object centre, so
                // the hand frame is offset back along hand +Z by the TCP offset.
                Eigen::Vector3d hand_z = orientation.col(2);
                Eigen::Vector3d grasp_pos =
                    object_center - kTcpOffset * hand_z;

                // The pre-grasp places the TCP kApproachDistance in front of the
                // object surface along the approach direction.
                Eigen::Vector3d pre_pos =
                    grasp_pos - (kApproachDistance + extent) * dir;

                Grasp g;
                g.grasp_pose.header.frame_id = object.pose.header.frame_id;
                g.grasp_pose.pose.position.x = grasp_pos.x();
                g.grasp_pose.pose.position.y = grasp_pos.y();
                g.grasp_pose.pose.position.z = grasp_pos.z();
                const Eigen::Quaterniond quat(orientation);
                g.grasp_pose.pose.orientation.x = quat.x();
                g.grasp_pose.pose.orientation.y = quat.y();
                g.grasp_pose.pose.orientation.z = quat.z();
                g.grasp_pose.pose.orientation.w = quat.w();

                g.pre_grasp_pose = g.grasp_pose;
                g.pre_grasp_pose.pose.position.x = pre_pos.x();
                g.pre_grasp_pose.pose.position.y = pre_pos.y();
                g.pre_grasp_pose.pose.position.z = pre_pos.z();
                g.approach_distance = kApproachDistance + extent;

                // The grasp is the harder-to-reach pose, so check it first; if it is
                // unreachable the pre-grasp is not checked (saves an IK call).
                if (!validator_.ikReachable(g.grasp_pose))
                    continue;
                // The pre-grasp must be reachable AND collision-free (it would
                // otherwise fail OMPL's goal sampling later).
                if (!validator_.ikReachable(g.pre_grasp_pose, true))
                    continue;

                candidates.push_back(std::move(g));
                if (candidates.size() >= kMaxCandidates)
                    return true;
            }
            return false;
        };

        // Yaw sweep in the horizontal plane (approach directions about world Z),
        // starting at +X (front) which keeps the ranking front-first.
        constexpr double kYawStepDeg = 30.0;
        for (double yaw = 0.0; yaw < 360.0; yaw += kYawStepDeg)
        {
            const double yaw_rad = yaw * M_PI / 180.0;
            Eigen::Vector3d dir(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
            if (consider_direction(dir))
                return candidates;
        }

        // Top-down approach (not in the XY-plane).
        consider_direction(Eigen::Vector3d(0, 0, -1));

        return candidates;
    }

    AnyGraspProvider::AnyGraspProvider(
        const rclcpp::Node::SharedPtr &node,
        const std::string &arm_group,
        const std::string &eef_frame)
        : node_(node), arm_group_(arm_group), eef_frame_(eef_frame),
          validator_(arm_group, eef_frame)
    {
        grasp_sub_ = node_->create_subscription<
            geometry_msgs::msg::PoseArray>(
            "/grasp_poses", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                latest_grasps_ = *msg;
            });
    }

    std::vector<Grasp> AnyGraspProvider::generate(
        const PickObject &object) const
    {
        constexpr double kApproachDistance = 0.15;
        constexpr size_t kMaxCandidates = 5;

        std::vector<Grasp> candidates;
        const std::string &target_frame = object.pose.header.frame_id;

        for (const auto &pose : latest_grasps_.poses)
        {
            if (latest_grasps_.header.frame_id != target_frame)
            {
                RCLCPP_WARN_ONCE(
                    rclcpp::get_logger("openarm_mtc.anygrasp"),
                    "Grasp poses are in frame '%s' but the object is in '%s'; "
                    "skipping. (Frame transform is not implemented in this "
                    "skeleton yet.)",
                    latest_grasps_.header.frame_id.c_str(), target_frame.c_str());
                break;
            }

            // The published grasp pose is the hand frame at the grasp.
            Grasp g;
            g.grasp_pose.header.frame_id = target_frame;
            g.grasp_pose.pose = pose;

            // Derive the pre-grasp: offset back along the grasp's forward axis.
            Eigen::Quaterniond quat(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z);
            Eigen::Vector3d hand_z = quat * Eigen::Vector3d::UnitZ();
            g.pre_grasp_pose = g.grasp_pose;
            g.pre_grasp_pose.pose.position.x -= hand_z.x() * kApproachDistance;
            g.pre_grasp_pose.pose.position.y -= hand_z.y() * kApproachDistance;
            g.pre_grasp_pose.pose.position.z -= hand_z.z() * kApproachDistance;
            g.approach_distance = kApproachDistance;

            if (!validator_.ikReachable(g.grasp_pose))
                continue;
            if (!validator_.ikReachable(g.pre_grasp_pose, true))
                continue;

            candidates.push_back(std::move(g));
            if (candidates.size() >= kMaxCandidates)
                break;
        }

        return candidates;
    }

} // namespace openarm_mtc