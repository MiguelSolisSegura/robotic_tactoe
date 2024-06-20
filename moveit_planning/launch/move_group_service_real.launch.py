import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mycobot_280", package_name="moveit_config_real").to_moveit_configs()

    moveit_cpp_node = Node(
        name="move_group_service",
        package="moveit_planning",
        executable="move_group_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )