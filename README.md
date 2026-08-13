# Skid-Robot VLM

Autonomous skid-steer robot built on **NVIDIA Isaac ROS** (Jetson Orin Nano) that combines **Nav2 navigation**, a custom **Control Barrier Function (CBF) safety filter**, and **Vision-Language Model (VLM)-based person tracking/behavior control**.

The robot can navigate autonomously using Nav2, while a VLM (Qwen3VL-2B) interprets natural-language/vision context to drive high-level behaviors (e.g. follow a target, search for a target), with a real-time CBF safety layer guaranteeing collision-safe velocity commands regardless of which subsystem issues them.

## Hardware

| Component | Role |
|---|---|
| NVIDIA Jetson Orin Nano Dev Kit | Onboard compute — runs Isaac ROS, Nav2, YOLOv8/TensorRT, and the VLM (llama.cpp) |
| ESP32 (Micro-ROS) | Low-level motor control for the skid-steer drivetrain. Firmware lives in a separate repo: [HaIBotLab/skidsteer-firmware](https://github.com/HaIBotLab/skidsteer-firmware) |
| Velodyne VLP-16 | 3D LiDAR — obstacle detection (ground removal + RANSAC), costmap input, occupancy-grid localization |
| USB Camera (2K) | Vision input for the VLM / person detection pipeline |
| Microstrain IMU | Inertial data for EKF-based odometry |
| Wheel encoders | Wheel odometry, fused into EKF localization |

## Repository structure

```
src/
├── skid_robot/
│   ├── bringup/          # Launch files: bringup_launch.py (hardware), 
│   │                      #   nav2_launch.py, vlm_behaviors_launch.py
│   ├── description/       # URDF + STL meshes (chassis, wheels, lidar, IMU mounts)
│   ├── hardware/
│   │   └── low_control/   # Kinematic node: converts /cmd_vel → wheel commands
│   ├── localization/      # EKF config (robot_localization) + odometry
│   └── maps/               # Pre-scanned 2D map(s) for Nav2/AMCL
├── perception/            # Camera input, YOLOv8 (TensorRT10), ByteTrack,
│                            #   LiDAR ground-removal + RANSAC → 3D object position
├── reasoning/              # vlm_client_node (talks to llama-server), trigger_input_node
├── behavior_executive/     # behavior_executor_node: high-level behaviors
│                            #   (follow_target, search_target, ...) driven by VLM output
├── cbf_safety_filter/       # CBF-QP safety filter (OSQP/OsqpEigen) — final /cmd_vel gate
├── isaac_ros_common/        # NVIDIA Isaac ROS devenv scripts (run_dev.sh, etc.)
├── isaac_ros_map_localization/ # Isaac ROS occupancy-grid localizer (auto initial pose)
├── isaac_ros_nitros/         # NVIDIA Isaac ROS Nitros transport
├── isaac_ros_nvblox/          # NVIDIA Isaac ROS Nvblox (optional 3D costmap backend)
├── uros/                       # micro-ROS agent/tooling for ESP32 communication
└── llama.cpp/                   # llama.cpp — runs the local VLM inference server
```

## Perception pipeline

1. **YOLOv8** (exported to TensorRT10 engine) detects people in the 2K USB camera stream.
2. **ByteTrack** assigns/maintains a stable track ID per detected person across frames.
3. LiDAR point cloud is processed with **ground removal + RANSAC** to isolate non-ground obstacle points.
4. The 2D bounding box from YOLOv8 is projected/mapped onto the filtered LiDAR points to recover the **3D position and distance** of each tracked person.
5. Tracked object data (ID, position, distance) is made available to the reasoning/behavior layer.

## VLM & behavior pipeline

- **Model:** Qwen3VL-2B-Instruct (quantized GGUF), served locally via `llama.cpp` (`llama-server`) on the Jetson.
- **`reasoning/`**: `vlm_client_node` sends camera frames + prompts to the local VLM server and publishes the resulting decision to `/vlm/target_command`. `trigger_input_node` allows manual triggering via stdin for testing.
- **`behavior_executive/`**: `behavior_executor_node` subscribes to `/vlm/target_command` and the tracked-person stream, and drives the robot accordingly — either through Nav2 actions (`/navigate_to_pose`, `/spin`) for goal-directed motion, or direct velocity commands for close-range tracking/servoing (both routed through the smoother + CBF safety layer).

## Safety layer — CBF Safety Filter

`cbf_safety_filter/` implements a **Control Barrier Function (CBF)** based safety filter using **OSQP/OsqpEigen** to solve a real-time QP that minimally adjusts incoming velocity commands to keep the robot at a safe distance from obstacles (using the local costmap). It sits as the last node before the final `/cmd_vel` topic, so it applies uniformly to navigation, recovery, and VLM-driven commands.

Key parameters (`cbf_safety_filter/config/cbf_params.yaml`):
- `d_safe`: minimum safety distance to obstacles
- `alpha`: CBF class-K gain
- `v_max` / `v_min` / `w_max`: velocity limits enforced by the filter

## Getting started

### 1. Prerequisites

- **Jetson Orin Nano Dev Kit** with JetPack installed.
- **Docker** + **NVIDIA Container Toolkit** configured (required by Isaac ROS devenv).
- Isaac ROS development environment set up via `isaac_ros_common`'s `run_dev.sh`, per the [official Isaac ROS Docker devenv guide](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/docker_devenv/index.html).
- [ESP32 firmware](https://github.com/HaIBotLab/skidsteer-firmware) flashed separately and connected to the Jetson (USB serial, used by the Micro-ROS agent).

### 2. Clone and enter the Isaac ROS devenv container

```bash
git clone https://github.com/binh705-engineer/skid-robot-VLM.git
cd skid-robot-VLM
# Launch the Isaac ROS devenv container (per isaac_ros_common docs)
./src/isaac_ros_common/scripts/run_dev.sh
```

### 3. Install dependencies inside the container

```bash
source src/setup_my_env.sh
```
This installs required apt/pip packages, sets up `CMAKE_PREFIX_PATH`/`LD_LIBRARY_PATH` for locally-built libraries (OSQP, OsqpEigen), grants permissions to serial devices, and sources the ROS 2 workspace.

### 4. Build required C++ dependencies (one-time setup)

The CBF safety filter depends on **OSQP** and **OsqpEigen**, which are not available via apt and must be built from source. This only needs to be done **once** per devenv container:

```bash
mkdir -p /workspaces/isaac_ros-dev/.local

git clone --recursive https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/workspaces/isaac_ros-dev/.local ..
cmake --build . --target install
cd ../..

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/workspaces/isaac_ros-dev/.local -DCMAKE_PREFIX_PATH=/workspaces/isaac_ros-dev/.local ..
make -j$(nproc) && make install
```

> These libraries install to `/workspaces/isaac_ros-dev/.local`, outside the versioned `src/`/`install/` folders — if the devenv container is ever recreated from scratch, this step must be repeated. `setup_my_env.sh` already exports `CMAKE_PREFIX_PATH`/`LD_LIBRARY_PATH` pointing to this location, so once built here, every subsequent `colcon build` and `ros2 launch` picks them up automatically — no need to re-run this step on every session.

### 5. Download the VLM model

Download a quantized GGUF build of **Qwen3VL-2B-Instruct** (main model + mmproj vision projector) from Hugging Face, and place both files under `models/`:
```
models/Qwen3VL-2B-Instruct-Q4_K_M.gguf
models/mmproj-Qwen3VL-2B-Instruct-F16.gguf
```
(Model files are excluded from this repo via `.gitignore` — download separately.)

### 6. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 7. Run

**Step 1 — bring up the robot hardware** (TF, IMU, LiDAR, Micro-ROS agent ↔ ESP32, odometry, EKF):
```bash
ros2 launch bringup bringup_launch.py
```

**Step 2 — bring up navigation, perception, and the VLM behavior stack** (Nav2 + CBF safety filter, perception, behavior executive, llama-server, reasoning):
```bash
ros2 launch bringup vlm_behaviors_launch.py
```

Once both are running, the robot will localize on the pre-scanned map, and the VLM-driven behavior layer will start responding to detected/tracked people according to the configured behaviors (e.g. `follow_target`, `search_target`).

### Sending a command to the VLM

`trigger_input_node` (started as part of `vlm_behaviors_launch.py`) reads commands interactively from stdin. Alternatively, a command can be published directly to `/vlm/trigger`:

```bash
ros2 topic pub --once /vlm/trigger std_msgs/msg/String "data: 'your command here'"
```

Replace `'your command here'` with the natural-language instruction for the VLM (e.g. `'follow the person in the red shirt'`). The `reasoning` package picks this up, queries the local VLM server, and publishes the resulting decision to `/vlm/target_command`, which `behavior_executor_node` then acts on.

> If you're adapting this project for a different chassis, replace `skid_robot` (URDF, `low_control`, EKF config) with your own robot's equivalent package — the Nav2, CBF, perception, and VLM/behavior layers are chassis-agnostic.

## Project status

✅ **v1 — VLM (current):** Nav2 navigation, the CBF safety filter, and VLM-based person tracking/behavior control are integrated and working end-to-end. The VLM (Qwen3VL-2B) interprets the scene and selects from a fixed set of high-level behaviors (`follow_target`, `search_target`, ...), which the behavior executive then carries out via Nav2 actions or direct velocity commands.

🔜 **v2 — VLA (planned):** Upgrade the reasoning/behavior layer from a VLM that *selects between predefined behaviors* to a full **Vision-Language-Action (VLA)** model that *directly outputs low-level actions/trajectories* from vision + language input — removing the need for a hand-written set of discrete behaviors and enabling more general, fine-grained control. This will be developed on a separate branch and released as a new version once validated, without modifying the current working v1 pipeline.

## Author

**binh705-engineer** — [HaIBotLab](https://github.com/HaIBotLab)
