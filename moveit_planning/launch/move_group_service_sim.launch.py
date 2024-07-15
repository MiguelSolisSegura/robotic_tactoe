import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="moveit_config").to_moveit_configs()

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )
    
    # Arm move service
    moveit_cpp_node = Node(
        name="move_group_service",
        package="moveit_planning",
        executable="move_group_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True,
             'orientation_x': 0.0,
             'orientation_y': 0.0,
             'orientation_z': 0.0,
             'orientation_w': 1.0},
        ],
    )

    # RViz interface
    rviz_node = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        [move_group_node, moveit_cpp_node, rviz_node]
    )


