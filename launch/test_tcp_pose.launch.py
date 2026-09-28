import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    description_dir = get_package_share_directory("openarm_description")
    moveit_dir = get_package_share_directory("openarm_bimanual_moveit_config")

    xacro_path = os.path.join(
        description_dir,
        "assets", "robot", "openarm_v1.0",
        "urdf", "openarm_v10.urdf.xacro")

    srdf_path = os.path.join(
        moveit_dir,
        "config", "openarm_v1.0",
        "openarm_bimanual.srdf")

    kinematics_path = os.path.join(
        moveit_dir,
        "config", "openarm_v1.0",
        "kinematics.yaml")

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v1.0",
            "bimanual": "true",
            "use_fake_hardware": "true",
            "ros2_control": "true",
        }).toxml()

    with open(srdf_path, "r") as f:
        robot_description_semantic = f.read()

    with open(kinematics_path, "r") as f:
        robot_description_kinematics = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package="openarm_mtc",
            executable="test_tcp_pose",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": robot_description_kinematics,
            }],
        ),
    ])