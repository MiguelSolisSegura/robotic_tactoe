from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Arm service, move group and RViz
    arm_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('moveit_planning'),
            'launch',
            'move_group_service_sim.launch.py')))

    # Get board state service
    get_board_state = Node(
        package='image_processing',
        executable='get_board_state',
        name='get_board_state',
        output='screen',
        parameters=[
            {'sim': True}
        ],
    )

    # Compute best move service
    compute_best_move = Node(
        package='image_processing',
        executable='compute_best_move',
        name='compute_best_move',
        output='screen')

    # Orchestration service
    orchestration = Node(
        package='moveit_planning',
        executable='tic_tac_toe',
        name='tic_tac_toe',
        output='screen',
        parameters=[{'sim': True}])

    return LaunchDescription([
        arm_planning,
        get_board_state,
        compute_best_move,
        orchestration
    ])