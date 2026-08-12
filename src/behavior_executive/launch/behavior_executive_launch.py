import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("behavior_executive")
    params_file = os.path.join(share_dir, "config", "behavior_executive_params.yaml")

    behavior_executor_node = Node(
        package="behavior_executive",
        executable="behavior_executor_node",
        name="behavior_executor_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        behavior_executor_node,
    ])
