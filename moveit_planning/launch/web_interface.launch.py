from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ROS bridge
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch',
            'rosbridge_websocket_launch.xml')))

    # Video server
    video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen')

    # Webpage server
    webpage_server = ExecuteProcess(
        cmd=[os.path.join(os.getcwd(), 'start_server.sh')],
        output='screen')

    return LaunchDescription([
        rosbridge,
        video_server,
        webpage_server
    ])