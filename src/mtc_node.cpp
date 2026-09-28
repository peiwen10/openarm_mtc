#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "openarm_mtc/pick_task.hpp"
#include "openarm_mtc/grasp.hpp"

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("openarm_mtc");

class OpenArmMTCNode
{
public:
    OpenArmMTCNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp::Node>("openarm_mtc", options))
    {
        object_pose_sub_ =
            node_->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/object_pose", 10,
                std::bind(
                    &OpenArmMTCNode::objectPoseCallback,
                    this, std::placeholders::_1));

        // Grasp source selection: "hardcoded" (generated from the object pose,
        // for simulation) or "anygrasp" (consumes /grasp_poses from a grasp
        // detector, for the real robot).
        if (!node_->has_parameter("grasp_provider_type"))
            node_->declare_parameter("grasp_provider_type", "hardcoded");
        std::string provider_type;
        node_->get_parameter("grasp_provider_type", provider_type);
        if (provider_type == "anygrasp")
        {
            grasp_provider_ =
                std::make_shared<openarm_mtc::AnyGraspProvider>(
                    node_, "left_arm", "openarm_left_hand");
            RCLCPP_INFO(LOGGER, "Using AnyGraspProvider (reads /grasp_poses).");
        }
        else
        {
            grasp_provider_ =
                std::make_shared<openarm_mtc::HardcodedGraspProvider>(
                    "left_arm", "openarm_left_hand");
            RCLCPP_INFO(LOGGER, "Using HardcodedGraspProvider.");
        }
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

private:
    // Receive an object pose and start the reusable pick task.
    void objectPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (planning_)
        {
            RCLCPP_WARN(
                LOGGER,
                "Pick task is already running. Ignoring new object pose.");
            return;
        }

        planning_ = true;

        RCLCPP_INFO(
            LOGGER,
            "Received object pose: x=%.3f y=%.3f z=%.3f frame=%s",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->header.frame_id.c_str());

        openarm_mtc::PickObject object;
        object.id = "object";
        object.pose = *msg;

        // Object dimensions in metres.
        object.size_x = 0.05;
        object.size_y = 0.05;
        object.size_z = 0.10;

        // Select which OpenArm performs the pick.
        openarm_mtc::PickTask pick_task(
            node_,
            openarm_mtc::Arm::LEFT,
            object,
            grasp_provider_);

        if (!pick_task.planAndExecute())
        {
            RCLCPP_ERROR(LOGGER, "Pick task failed.");
        }

        planning_ = false;
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<openarm_mtc::GraspProvider> grasp_provider_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;

    bool planning_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<OpenArmMTCNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->getNodeBaseInterface());

    RCLCPP_INFO(LOGGER, "OpenArm MTC node started.");
    RCLCPP_INFO(LOGGER, "Waiting for /object_pose ...");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}