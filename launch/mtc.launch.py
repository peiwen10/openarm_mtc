import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_arm_config(arm_type_str):
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    if any(x in arm_type_str for x in ("2.0", "20", "2_0")):
        return "openarm_v2.0", "openarm_v20.urdf.xacro"
    raise ValueError(f"Invalid arm_type: {arm_type_str}")


def create_mtc_node(context: LaunchContext, arm_type, grasp_provider_type):
    arm_type_str = context.perform_substitution(arm_type)
    grasp_provider_str = context.perform_substitution(grasp_provider_type)

    description_pkg = get_package_share_directory("openarm_description")
    moveit_pkg = get_package_share_directory(
        "openarm_bimanual_moveit_config")

    config_dir, urdf_file = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        description_pkg,
        "assets", "robot", config_dir,
        "urdf", urdf_file,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": "true",
            "ros2_control": "true",
        },
    ).toprettyxml(indent="  ")

    srdf_path = os.path.join(
        moveit_pkg,
        "config", config_dir,
        "openarm_bimanual.srdf",
    )

    kinematics_path = os.path.join(
        moveit_pkg,
        "config", config_dir,
        "kinematics.yaml",
    )

    import yaml

    with open(kinematics_path, "r") as f:
        kinematics = yaml.safe_load(f)

    planning_pipelines = {
    "ompl": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
    },
    "planning_pipelines": ["ompl"],
    "default_planning_pipeline": "ompl",
    }

    parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": open(srdf_path).read(),
        "robot_description_kinematics": kinematics,
        "grasp_provider_type": grasp_provider_str,
        **planning_pipelines,
    }

    return [
        Node(
            package="openarm_mtc",
            executable="mtc_node",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    arm_type = LaunchConfiguration("arm_type")
    grasp_provider_type = LaunchConfiguration("grasp_provider_type")

    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_type",
            default_value="v1.0",
            description="OpenArm model type",
        ),
        DeclareLaunchArgument(
            "grasp_provider_type",
            default_value="hardcoded",
            description="Grasp source: 'hardcoded' (generated) or 'anygrasp' "
                        "(reads /grasp_poses)",
        ),
        OpaqueFunction(
            function=create_mtc_node,
            args=[arm_type, grasp_provider_type],
        ),
    ])