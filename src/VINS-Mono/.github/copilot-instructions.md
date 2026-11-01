# Copilot Instructions for VINS-Mono ROS 2

VINS-Mono is a ROS 2 Humble monocular visual-inertial SLAM system (sliding-window VIO + loop closure + pose graph). It uses C++14, ament_cmake and colcon.

## Build / Test / Lint

Build it from a ROS 2 workspace:

```bash
mkdir -p ~/vins_ws/src
cd ~/vins_ws/src && git clone <this-repo>
cd ~/vins_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

- Build selected packages: `colcon build --packages-up-to vins_estimator pose_graph`.
- Ubuntu 22.04's Ceres 2.x is supported. Do not reintroduce ROS 1 `LocalParameterization` assumptions without checking the current port.
- There are **no unit tests and no lint config**. Verification is manual: launch a node, play a rosbag, inspect RViz and console output.
- Launch files used by ROS 2 have the `.launch.py` suffix. Legacy ROS 1 XML launch files are retained only as reference.

## High-Level Architecture

The system is three cooperating ROS nodes plus a camera model library:

1. **`feature_tracker`** (`feature_tracker_node.cpp`) — KLT optical-flow frontend. Tracks features frame-to-frame and publishes them as `sensor_msgs::msg::PointCloud2` on `/feature_tracker/feature` (plus `/feature_tracker/feature_img` for visualization and `/feature_tracker/restart`).

2. **`vins_estimator`** (`estimator_node.cpp`, `Estimator` in `estimator.{h,cpp}`) — the core VIO backend. Runs a sliding-window nonlinear optimization (Ceres) with IMU pre-integration. Key phases: estimator initialization (`initial/`, using 5-point + SFM + visual-inertial alignment), then `solveOdometry()`/`optimization()` with marginalization (`factor/marginalization_factor.cpp`). The node runs two loops: a high-frequency `predict()` (IMU forward propagation) and the `process()` thread that consumes synchronized IMU+feature pairs.

3. **`pose_graph`** (`pose_graph_node.cpp`, `pose_graph.{h,cpp}`, `keyframe.{h,cpp}`) — loop closure and 4-DoF global pose graph optimization. Uses a vendored DBoW2 (`pose_graph/src/ThirdParty/`) for place recognition and publishes loop-closure matches to `/pose_graph/match_points`, which the estimator consumes for relocalization.

Auxiliary packages: `camera_model` (camodocal-derived camera model library used by the pose graph), `ar_demo` (AR overlay demo), `benchmark_publisher` (ground-truth visualization), `data_generator` (synthetic data).

Data flow: camera + IMU topics → `feature_tracker` → `/feature_tracker/feature` → `vins_estimator` → `/vins_estimator/odometry`, `/vins_estimator/keyframe_pose`, `/vins_estimator/keyframe_point` → `pose_graph` → `/pose_graph/match_points` → back into `vins_estimator` for relocalization.

## Key Conventions

- **Config drives everything.** YAML paths are passed as ROS 2 parameters and read with OpenCV `FileStorage`. Parameters are read via `readParameters(rclcpp::Node*)` into global `extern` variables. Adding a tunable usually means updating the YAML and the corresponding parameter reader.
- **Hardcoded constants live in `vins_estimator/src/parameters.h`** — `WINDOW_SIZE` (10), `NUM_OF_F` (1000), `NUM_OF_CAM` (1), `FOCAL_LENGTH` (460). The code is effectively mono-only despite the `NUM_OF_CAM` indirection.
- **Inter-node feature encoding.** Features are passed as `sensor_msgs::msg::PointCloud2` with eight float fields: `x`, `y`, `z`, `id`, `u`, `v`, `velocity_x`, `velocity_y`.
- **Eigen aliases** `Vector3d`, `Matrix3d`, `Quaterniond` are defined in `vins_estimator/src/utility/utility.h`; `TicToc` (timing) lives in `utility/tic_toc.h`.
- **Naming style** mirrors the upstream HKUST code: `snake_case` files, camelCase methods, globals are SCREAMING_SNAKE. Keep style consistent rather than modernizing.
- **ROS topic names are private-relative** — use explicit ROS 2 `~/...` names for node-owned outputs so they resolve under the node name (`/vins_estimator/...`, `/feature_tracker/...`, `/pose_graph/...`). Raw image and IMU subscriptions use `rclcpp::SensorDataQoS()`.
- **No `using namespace std`** at file scope in the core; `std::` is usually explicit, but `Eigen` types are often unqualified via the aliases above.
