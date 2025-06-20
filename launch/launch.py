from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('bbox2marker'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='bbox2marker',
            executable='bbox2marker_node',
            name='bbox2marker_node',
            output='screen',
            parameters=[params_file]
        )
    ])
