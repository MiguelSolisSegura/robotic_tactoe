# -*- coding: utf-8 -*-
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Function to load a YAML file
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None

# Main function to generate the launch description
def generate_launch_description():
    declared_arguments = []

    # Declare launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mycobot_280_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="mycobot_280.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed, then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="mycobot_280.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )

    # Initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Define paths for the configuration files
    joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "mycobot_280",
            "joint_limits.yaml",
        ]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "mycobot_280",
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "mycobot_280",
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "mycobot_280",
            "visual_parameters.yaml",
        ]
    )

    # Generate the robot description command
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "name:=",
            "mycobot_280",
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Generate the robot description semantic command
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "mycobot_280",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Load the kinematics configuration YAML file
    kinematics_yaml = load_yaml("mycobot_280_description", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Define the RViz configuration file path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription(declared_arguments + [rviz_node])

