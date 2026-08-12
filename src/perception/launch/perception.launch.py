import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_dir = get_package_share_directory("perception")
    params_file = os.path.join(
        share_dir,
        "config",
        "perception_params.yaml"
    )
    container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            # 1. USB Camera Component
            ComposableNode(
                package="perception",
                plugin="perception::UsbCamComponent",
                name="usb_cam_component",
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
            # 2. YOLOv8 TensorRT Component
            ComposableNode(
                package="perception",
                plugin="perception::Yolov8TrtComponent",
                name="yolov8_trt_component",
                parameters=[
                    params_file
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
            # 3. ByteTrack Tracking Component
            ComposableNode(
                package="perception",
                plugin="perception::ByteTrackComponent",
                name="bytetrack_component",
                parameters=[
                    params_file
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
            # 4. Depth Projector Component
            # Nhận bbox2D+track_id (từ ByteTrack) + pointcloud (từ Velodyne)
            # -> xuất track_id + vị trí 3D + ô BEV grid
            ComposableNode(
                package="perception",
                plugin="perception::DepthProjectorComponent",
                name="depth_projector_component",
                parameters=[
                    params_file
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
    )

    # 5. Visualizer Node (Python script, perception/scripts/visualizer_node.py)
    # Chua ro node nay lam gi / can param gi (chua doc noi dung file) -- tam
    # thoi khong truyen params_file. Neu no can doc block rieng trong
    # perception_params.yaml, gui noi dung file de bo sung.
    # GIA DINH: entry_point ten "visualizer_node" -- kiem tra lai trong setup.py.
    visualizer_node = Node(
        package="perception",
        executable="visualizer_node.py",
        name="visualizer_node",
        output="screen",
    )

    return LaunchDescription([
        container,
        visualizer_node,
    ])
