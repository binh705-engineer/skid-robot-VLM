import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_localization = get_package_share_directory('ekf')
    
    # 1. Sửa đường dẫn trỏ tới file ekf_isaac_ros.yaml
    robot_localization_file_path = os.path.join(pkg_localization, 'config/ekf_isaac_ros.yaml')

    # Khởi tạo Node cho Local EKF
    start_local_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[robot_localization_file_path],
        remappings=[('/odometry/filtered', '/odometry/local')]) # Thêm remap

    # Khởi tạo Node cho Global EKF
    start_global_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[robot_localization_file_path],
        remappings=[('/odometry/filtered', '/odometry/global')]) # Thêm remap
   
    # Trả về cả 2 node để chạy đồng thời
    return LaunchDescription([
        start_local_ekf_cmd,
        start_global_ekf_cmd
    ])
