import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ========================================================================
    # NODE 1: POINTCLOUD TO LASERSCAN 
    # ========================================================================
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/velodyne_points'),  # Hứng dữ liệu 3D từ Velodyne
            ('scan', '/scan')                  # Nhả dữ liệu 2D ra topic /scan
        ],
        parameters=[{
            'target_frame': 'velodyne',        # Hệ quy chiếu gốc của cảm biến
            'transform_tolerance': 0.01,
            'min_height': -0.5,                # CHIỀU CAO CẮT DƯỚI (Tính từ mắt Velodyne)
            'max_height': 1.0,                 # CHIỀU CAO CẮT TRÊN (Tính từ mắt Velodyne)
            'angle_min': -3.14159,             # Quét đủ 360 độ (Trừ pi)
            'angle_max': 3.14159,              # Quét đủ 360 độ (Cộng pi)
            'angle_increment': 0.0087,         # Độ phân giải góc
            'scan_time': 0.1,                  # Tốc độ quét (10Hz của Velodyne)
            'range_min': 0.2,                  # Bỏ qua các điểm quá sát cảm biến (< 20cm)
            'range_max': 50.0,                 # Nhìn xa tối đa 50m
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # ========================================================================
    # NODE 2: SLAM TOOLBOX 
    # ========================================================================
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
    )

    # Trả về để ROS 2 chạy đồng thời cả 2 tiến trình
    return LaunchDescription([
        pointcloud_to_laserscan_node,
        slam_toolbox_launch
    ])

