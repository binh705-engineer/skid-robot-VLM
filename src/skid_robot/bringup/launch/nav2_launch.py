import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, GroupAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. Tham số truyền vào
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/workspaces/isaac_ros-dev/src/skid_robot/maps/map.yaml',
        description='Đường dẫn tuyệt đối tới file .yaml của bản đồ 2D đã quét'
    )
    map_yaml_config = LaunchConfiguration('map_yaml_path')

    # Tham số chọn phương án costmap: false = traditional (2D), true = Isaac ROS Nvblox (3D)
    use_nvblox_arg = DeclareLaunchArgument(
        'use_nvblox',
        default_value='false',
        description='True để dùng nav2_isaac_ros_params.yaml (Nvblox 3D), False để dùng nav2_traditional_params.yaml (2D)'
    )
    use_nvblox = LaunchConfiguration('use_nvblox')

    # --- ĐƯỜNG DẪN CẤU HÌNH ---
    bringup_share = get_package_share_directory('bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    cbf_safety_filter_share = get_package_share_directory('cbf_safety_filter')

    traditional_params_path = os.path.join(bringup_share, 'config', 'nav2_traditional_params.yaml')
    isaac_ros_params_path = os.path.join(bringup_share, 'config', 'nav2_isaac_ros_params.yaml')

    # Chọn file params dựa trên use_nvblox tại thời điểm launch (runtime substitution)
    nav2_custom_params_path = PythonExpression([
        "'", isaac_ros_params_path, "' if '", use_nvblox, "' == 'true' else '", traditional_params_path, "'"
    ])

    # 2. Node chuẩn hoá pointcloud
    pointcloud_filter_node = Node(
        package='scripts',
        executable='pointcloud_xyz_filter',
        name='pointcloud_xyz_filter',
        parameters=[{
            'input_topic': '/velodyne_points',
            'output_topic': '/velodyne_points/xyz',
        }],
        output='screen'
    )

    # 3. Cụm Container NVIDIA Isaac ROS (Ép 2D + Định vị toàn cục)
    isaac_ros_container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode',
                name='pointcloud_to_flatscan',
                parameters=[{'min_range': 0.2, 'max_range': 50.0}],
                remappings=[('pointcloud', '/velodyne_points/xyz')]
            ),
            ComposableNode(
                package='isaac_ros_occupancy_grid_localizer',
                plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
                name='occupancy_grid_localizer',
                parameters=[
                    map_yaml_config,
                    {
                        'loc_result_frame': 'map',
                        'map_yaml_path': map_yaml_config,
                        'max_points': 3500,
                        'min_scan_fov_degrees': 230.0
                    }
                ],
                remappings=[
                    ('flatscan', '/flatscan'),
                    ('localization_result', '/initialpose'),  # Đẩy kết quả thẳng vào AMCL
                ]
            )
        ],
        output='screen'
    )

    # 4. Giao phó toàn bộ Nav2 Core, AMCL, và Map Server cho gói bringup của APT
    #    params_file được chọn tự động theo use_nvblox
    #
    #    BỌC trong GroupAction + SetRemap: mọi node bên trong bringup_launch.py
    #    (velocity_smoother, behavior_server, controller_server, ...) publish/subscribe
    #    '/cmd_vel' sẽ tự động bị đổi thành '/cmd_vel_smoothed'. Node CBF phía dưới
    #    sẽ sub '/cmd_vel_smoothed' và publish ra '/cmd_vel' thật (đi tới MCU).
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='cmd_vel_raw'),
            SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_final'), 
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'False',
                    'map': map_yaml_config,  # Đưa map vào để bringup tự chạy map_server
                    'params_file': nav2_custom_params_path,
                    'autostart': 'True'
                }.items()
            )
        ]
    )

    # 4b. CBF safety filter layer — sub '/cmd_vel_smoothed', publish '/cmd_vel' thật
    #     (params bên trong package tự khai cmd_vel_nav_topic / cmd_vel_out_topic,
    #     đảm bảo chúng khớp '/cmd_vel_smoothed' -> '/cmd_vel')
    cbf_safety_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cbf_safety_filter_share, 'launch', 'cbf_safety_filter.launch.py')
        )
    )

    # 5. Bootstrap tự động
    trigger_bootstrap = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=isaac_ros_container,
            on_start=[
                TimerAction(
                    period=7.0,  # Tăng lên 7 giây để chờ AMCL từ nav2_bringup lên hẳn rồi mới mồi /initialpose
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'service', 'call', '/trigger_grid_search_localization', 'std_srvs/srv/Empty', '{}'],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        map_yaml_arg,
        use_nvblox_arg,
        pointcloud_filter_node,
        isaac_ros_container,
        nav2_bringup_launch,       # bringup đã được bọc SetRemap('/cmd_vel' -> '/cmd_vel_smoothed')
        cbf_safety_filter_launch,  # CBF: sub /cmd_vel_smoothed, pub /cmd_vel
        trigger_bootstrap
    ])
