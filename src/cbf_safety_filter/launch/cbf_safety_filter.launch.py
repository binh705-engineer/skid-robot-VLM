from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('cbf_safety_filter'),
        'config', 'cbf_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='cbf_safety_filter',
            executable='cbf_safety_filter_node',
            name='cbf_safety_filter_node',
            output='screen',
            parameters=[params_file],
        )
    ])
