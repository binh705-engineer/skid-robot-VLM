import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    
    # ========================================================================
    # 1. ĐỌC FILE MÔ HÌNH 3D (URDF)
    # ========================================================================
    urdf_file_name = 'skid_robot_v3.urdf' 
    urdf_path = os.path.join(
        get_package_share_directory('description'),
        'urdf',
        urdf_file_name
    )
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    # Node tự động phát toàn bộ TF tĩnh từ file URDF (Robot State Publisher)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    return LaunchDescription([
        
        # ---------------- KHỐI 1: KHUNG XƯƠNG (URDF & TF Tĩnh) ----------------
        robot_state_publisher_node,
        # ---------------- KHỐI 2: CHÂN TAY (Low Control) + MICRO-ROS AGENT (ESP32) ----------------
        # micro_ros_agent giờ nằm trong low_level_control.launch.py, khởi chạy sớm
        # để giữ cổng USB (/dev/ttyUSB0) trước khi các node khác cần dữ liệu encoder
        IncludeLaunchDescription(
            FindPackageShare('low_control').find('low_control') + '/launch/low_level_control.launch.py'
        ),
        # ---------------- KHỐI 4: GIÁC QUAN (Cảm biến) ----------------
        IncludeLaunchDescription(
            FindPackageShare('microstrain_inertial_driver').find('microstrain_inertial_driver') + '/launch/microstrain_launch.py'
        ),
        IncludeLaunchDescription(
            FindPackageShare('velodyne').find('velodyne') + '/launch/velodyne-all-nodes-VLP16-launch.py'
        ),
        # ---------------- KHỐI 5: ĐỊNH VỊ (Odom & EKF) ----------------
        # Trễ 5s sau khi micro-ROS agent khởi chạy, để ESP32 kịp kết nối
        # Modbus tới ZLAC8015D và bắt đầu gửi dữ liệu /zlac_encoder ổn định
        # trước khi node odom cần dùng dữ liệu này.
        TimerAction(
            period=5.0,
            actions=[
                # 5.1. Chuyển đổi bánh xe sang Tốc độ (Odom)
                # Lưu ý: tham số đã đổi tên theo driver ESP32 mới (topic /zlac_encoder)
                Node(
                    package='odom',
                    executable='odom',
                    name='odom',
                    parameters=[
                        {'encoder_feedback_topic': 'zlac_encoder'},
                        {'odom_topic': 'odom'},
                        {'wheel_base': 0.48},
                        {'wheel_radius': 0.098}
                    ]
                ),
                # 5.2. Bộ lọc Kalman (Trộn IMU + Odom)
                TimerAction(
                    period=1.0,
                    actions=[
                        IncludeLaunchDescription(
                            FindPackageShare('ekf').find('ekf') + '/launch/ekf_launch.py'
                        )
                    ]
                ),
            ]
        ),
    ])
