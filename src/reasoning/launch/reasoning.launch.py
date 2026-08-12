import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("reasoning")
    params_file = os.path.join(
        share_dir,
        "config",
        "reasoning_params.yaml"
    )

    vlm_client_node = Node(
        package="reasoning",
        executable="vlm_client_node",
        name="vlm_client_node",
        output="screen",
        parameters=[
            params_file
        ],
    )

    # Node test thu cong: doc lenh nguoi dung tu terminal, publish len
    # /vlm/trigger. Chay trong cung 1 terminal se bi nuot output cua
    # vlm_client_node -- neu can vua go lenh vua xem log, nen chay
    # trigger_input_node RIENG BANG TAY (ros2 run reasoning trigger_input_node)
    # o 1 terminal khac, thay vi dua vao launch file nay.
    trigger_input_node = Node(
        package="reasoning",
        executable="trigger_input_node",
        name="trigger_input_node",
        output="screen",
        parameters=[
            params_file
        ],
        # QUAN TRONG: input() can stdin — launch se giu terminal, nhung neu
        # chay chung voi node khac trong cung LaunchDescription, output cua
        # 2 node se dan xen nhau, kho go lenh. Neu gap van de, comment dong
        # nay va nen chay trigger_input_node rieng bang tay.
    )

    coordinate_mapper_node = Node(
        package="reasoning",
        executable="coordinate_mapper_node",
        name="coordinate_mapper_node",
        output="screen",
        parameters=[
            params_file
        ],
        # QUAN TRONG: input() can stdin — launch se giu terminal, nhung neu
        # chay chung voi node khac trong cung LaunchDescription, output cua
        # 2 node se dan xen nhau, kho go lenh. Neu gap van de, comment dong
        # nay va nen chay trigger_input_node rieng bang tay.
    )

    return LaunchDescription([
        vlm_client_node,
        trigger_input_node,
    ])
