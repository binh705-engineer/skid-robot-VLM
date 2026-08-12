import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Whether to bring up the nav2 stack (map localization, '
                     'planners, controllers, etc). Set to false when you only '
                     'want to test/tune the VLM perception+reasoning pipeline '
                     'without the robot navigation stack running.'
    )
    enable_nav2 = LaunchConfiguration('enable_nav2')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/workspaces/isaac_ros-dev/src/skid_robot/maps/map.yaml',
        description='Absolute path to the .yaml file of the pre-scanned 2D map. '
                     'Only used when enable_nav2:=true.'
    )

    # NOTE: adjust these two if your llama.cpp build / model paths differ,
    # or if you move this repo to a different workspace root.
    llama_server_bin_arg = DeclareLaunchArgument(
        'llama_server_bin',
        default_value='/workspaces/isaac_ros-dev/src/llama.cpp/build/bin/llama-server',
        description='Path to the llama-server binary.'
    )
    llama_model_arg = DeclareLaunchArgument(
        'llama_model_path',
        default_value='/workspaces/isaac_ros-dev/models/Qwen3VL-2B-Instruct-Q4_K_M.gguf',
        description='Path to the main GGUF model file.'
    )
    llama_mmproj_arg = DeclareLaunchArgument(
        'llama_mmproj_path',
        default_value='/workspaces/isaac_ros-dev/models/mmproj-Qwen3VL-2B-Instruct-F16.gguf',
        description='Path to the mmproj GGUF file (vision projector for the VLM).'
    )
    llama_server_port_arg = DeclareLaunchArgument(
        'llama_server_port',
        default_value='8080',
        description='Port llama-server listens on.'
    )
    # Real readiness check: instead of a fixed sleep, poll llama-server's
    # /health endpoint over HTTP until it returns 200, then start reasoning.
    # NOTE: llama-server binds --host 0.0.0.0, but we poll via 127.0.0.1
    # since this check runs on the same machine.
    llama_health_host_arg = DeclareLaunchArgument(
        'llama_server_health_host',
        default_value='127.0.0.1',
        description='Host used to poll llama-server /health (curl target).'
    )
    health_check_max_tries_arg = DeclareLaunchArgument(
        'health_check_max_tries',
        default_value='60',
        description='Max number of /health poll attempts before giving up.'
    )
    health_check_interval_sec_arg = DeclareLaunchArgument(
        'health_check_interval_sec',
        default_value='2.0',
        description='Seconds to sleep between /health poll attempts. '
                     'max_tries * interval = worst-case wait time '
                     '(default 60 * 2s = 120s).'
    )

    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    bringup_share = get_package_share_directory('bringup')
    perception_share = get_package_share_directory('perception')
    reasoning_share = get_package_share_directory('reasoning')

    # ------------------------------------------------------------------
    # 1. Nav2 (optional, controlled by enable_nav2)
    # ------------------------------------------------------------------
    # ASSUMPTION: nav2_launch.py lives at
    # <bringup_share>/launch/nav2_launch.py and exposes the 'map_yaml_path'
    # argument, matching the file you shared.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2_launch.py')
        ),
        launch_arguments={
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }.items(),
        condition=IfCondition(enable_nav2),
    )

    # ------------------------------------------------------------------
    # 2. Perception (VLM input side: camera, YOLOv8, ByteTrack, depth projector)
    # ------------------------------------------------------------------
    # ASSUMPTION: perception launch file is saved as
    # <perception_share>/launch/perception_launch.py.
    # Rename this path if your actual filename differs.
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')
        )
    )

    # ------------------------------------------------------------------
    # 3. Load server (llama-server hosting the VLM: Qwen3VL-2B)
    # ------------------------------------------------------------------
    # ASSUMPTION: llama-server is run directly as a binary (not a ROS node),
    # so it's wrapped as an ExecuteProcess. cwd is set to the build dir the
    # same way you run it manually.
    load_server_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration('llama_server_bin'),
            '-m', LaunchConfiguration('llama_model_path'),
            '--mmproj', LaunchConfiguration('llama_mmproj_path'),
            '-ngl', '999',
            '-fa', 'on',
            '-t', '4',
            '-c', '1800',
            '--parallel', '1',
            '--port', LaunchConfiguration('llama_server_port'),
            '--host', '0.0.0.0',
            '--cache-ram', '0',
            '--image-min-tokens', '1024',
        ],
        cwd='/workspaces/isaac_ros-dev/src/llama.cpp/build',
        name='llama_server',
        output='screen',
    )

    # ------------------------------------------------------------------
    # 4. Reasoning (vlm_client_node + trigger_input_node)
    # ------------------------------------------------------------------
    # ASSUMPTION: reasoning launch file is saved as
    # <reasoning_share>/launch/reasoning_launch.py.
    # NOTE: trigger_input_node uses input() and needs an interactive stdin.
    # If its output gets interleaved with vlm_client_node's logs and makes
    # typing commands hard, comment it out inside reasoning_launch.py and
    # run it manually instead: `ros2 run reasoning trigger_input_node`.
    reasoning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(reasoning_share, 'launch', 'reasoning.launch.py')
        )
    )

    # ------------------------------------------------------------------
    # 5. Health check: poll llama-server's /health endpoint until it
    #    returns HTTP 200 (i.e. the model has finished loading and the
    #    server is ready to accept requests), or until max_tries is hit.
    # ------------------------------------------------------------------
    # This runs as a small bash loop instead of a fixed sleep, so reasoning
    # starts as soon as the server is actually ready (fast machine/GPU) and
    # still waits correctly on a slow one, instead of guessing a delay.
    health_check_script = [
        'tries=0; ',
        'max=', LaunchConfiguration('health_check_max_tries'), '; ',
        'interval=', LaunchConfiguration('health_check_interval_sec'), '; ',
        'url=http://', LaunchConfiguration('llama_server_health_host'),
        ':', LaunchConfiguration('llama_server_port'), '/health; ',
        'echo "[health_check] waiting for llama-server at $url ..."; ',
        'while [ "$tries" -lt "$max" ]; do ',
        'code=$(curl -s -o /dev/null -w "%{http_code}" "$url" 2>/dev/null); ',
        'if [ "$code" = "200" ]; then ',
        'echo "[health_check] llama-server is ready (HTTP 200) after ${tries} tries"; ',
        'exit 0; ',
        'fi; ',
        'tries=$((tries+1)); ',
        'sleep "$interval"; ',
        'done; ',
        'echo "[health_check] TIMED OUT waiting for llama-server after ${max} tries. '
        'Starting reasoning nodes anyway, but they will likely fail to connect '
        'until the server finishes loading."; ',
        'exit 1',
    ]

    health_check_process = ExecuteProcess(
        cmd=['bash', '-c', health_check_script],
        name='llama_server_health_check',
        output='screen',
    )

    # Only start the health check once llama-server has actually been
    # launched (process started, not necessarily ready yet).
    start_health_check_after_server = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=load_server_process,
            on_start=[health_check_process],
        )
    )

    # Start reasoning as soon as the health check process exits, whether it
    # succeeded (server ready) or timed out (exit code 1, see message above).
    # If you'd rather NOT start reasoning on a timeout, wrap reasoning_launch
    # with a launch.conditions check on the health check's exit code instead.
    start_reasoning_after_health_check = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=health_check_process,
            on_exit=[reasoning_launch],
        )
    )

    return LaunchDescription([
        # args
        enable_nav2_arg,
        map_yaml_arg,
        llama_server_bin_arg,
        llama_model_arg,
        llama_mmproj_arg,
        llama_server_port_arg,
        llama_health_host_arg,
        health_check_max_tries_arg,
        health_check_interval_sec_arg,
        # actions
        nav2_launch,
        perception_launch,
        load_server_process,
        start_health_check_after_server,
        start_reasoning_after_health_check,
    ])
