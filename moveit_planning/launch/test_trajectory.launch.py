import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="moveit_config").to_moveit_configs()

    # Declare the launch arguments
    declare_orientation_x = DeclareLaunchArgument('orientation_x', default_value='0.0', description='Orientation x')
    declare_orientation_y = DeclareLaunchArgument('orientation_y', default_value='0.0', description='Orientation y')
    declare_orientation_z = DeclareLaunchArgument('orientation_z', default_value='0.0', description='Orientation z')
    declare_orientation_w = DeclareLaunchArgument('orientation_w', default_value='0.0', description='Orientation w')
    declare_position_x = DeclareLaunchArgument('position_x', default_value='0.0', description='Position x')
    declare_position_y = DeclareLaunchArgument('position_y', default_value='0.0', description='Position y')
    declare_position_z = DeclareLaunchArgument('position_z', default_value='0.0', description='Position z')

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="test_trajectory",
        package="moveit_planning",
        executable="test_trajectory",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
            {'orientation_x': LaunchConfiguration('orientation_x')},
            {'orientation_y': LaunchConfiguration('orientation_y')},
            {'orientation_z': LaunchConfiguration('orientation_z')},
            {'orientation_w': LaunchConfiguration('orientation_w')},
            {'position_x': LaunchConfiguration('position_x')},
            {'position_y': LaunchConfiguration('position_y')},
            {'position_z': LaunchConfiguration('position_z')},
        ],
    )

    return LaunchDescription([
        declare_orientation_x,
        declare_orientation_y,
        declare_orientation_z,
        declare_orientation_w,
        declare_position_x,
        declare_position_y,
        declare_position_z,
        moveit_cpp_node,
    ])
