import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 0. MICRO-ROS AGENT (ESP32)
    # Khởi chạy sớm để giữ cổng USB (/dev/ttyUSB0) và kết nối ZLAC8015D
    # qua ESP32 trước khi các node khác (odom, kinematic) cần dữ liệu encoder/gửi lệnh
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0']
    )

    # 1. Node tính toán động học (Kinematic)
    kinematic_node = Node(
        package='low_control',
        executable='kinematic',
        name='kinematic',
        output='screen',
        parameters=[
            {'input_topic': 'cmd_vel'},
            {'output_topic': 'wheel_rpm'},
            {'khoang_cach_banh': 0.48}, # Trong Python, bạn có thể truyền thẳng số Float thay vì String
            {'r_banh': 0.098}
        ]
    )

    return LaunchDescription([
        micro_ros_agent_node,
        kinematic_node,
    ])
