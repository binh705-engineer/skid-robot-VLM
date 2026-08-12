import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Khai báo tham số truyền vào từ terminal
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Tên của file bản đồ sẽ được lưu (không cần đuôi .yaml)'
    )

    # Đường dẫn lưu file động dựa theo tên nhập từ terminal
    save_path = [
        '/workspaces/isaac_ros-dev/src/skid_robot/maps/',
        LaunchConfiguration('map_name')
    ]

    # Lấy đường dẫn tuyệt đối đến file nvblox_base.yaml 
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'nvblox_base_params.yaml'
    )

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            config_file_path,  # Nạp toàn bộ thông số từ file YAML
            {'after_shutdown_map_save_path': save_path}
        ],
        remappings=[
            ('/pointcloud', '/velodyne_points'),
        ]
    )

    return LaunchDescription([
        map_name_arg,
        nvblox_node
    ])
