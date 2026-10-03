# Copilot Instructions for RM_flying_robot_control

RoboMaster 2027 autonomous-flying drone upper-level control. This is a **ROS 2 Humble** (Ubuntu 22.04) colcon workspace whose job is to feed a PX4 1.14 flight controller a visual-inertial pose estimate from a monocular VINS-Mono pipeline.

## Build / Test / Lint

Environment: Ubuntu 22.04 + ROS 2 Humble. `px4_msgs` must match PX4 1.14 (`release/1.14` branch).

```bash
# 1. Pull submodules (VINS-Mono is fetched over SSH; the URL rewrite avoids the SSH key requirement)
git -c 'url.https://github.com/.insteadOf=git@github.com:' submodule update --init --recursive

# 2. Install system + ros-humble deps (see README.md for the full apt list), then:
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

# 3. Build everything
./build.sh   # = colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --symlink-install

# 4. Build a single package (or just its dependencies)
colcon build --symlink-install --packages-up-to driver_interface
```

- There are **no unit tests** and no meaningful lint config (some packages declare `ament_lint_auto` as `test_depend`, but nothing enforces it). Verification is manual: build, `source install/setup.bash`, launch a node, and inspect `ros2 topic echo`/`hz` + RViz.
- Run individual components rather than the whole system:
  - `ros2 launch bringup vins_system.launch.py` — full stack (camera + XRCE agent + driver_interface + VINS).
  - `ros2 launch driver_interface driver_interface.launch.py` — just the bridge node.
  - `ros2 launch mindvision_camera mv_launch.py` — just the camera.
  - `ros2 run driver_interface driver_interface_node --ros-args --params-file <yaml>` — run the node directly.

## High-Level Architecture

Five packages under `src/`:

1. **`px4_msgs`** — a vendored (tracked directly in this repo, **not** a submodule) copy of the PX4 message definitions, used to decode `px4_msgs/msg/SensorCombined` and friends.
2. **`mindvision_camera`** — third-party ROS 2 driver for the MindVision industrial camera. Publishes raw RGB on `/image_raw` with `frame_id: camera_optical_frame`.
3. **`driver_interface`** — the project's own bridge node (`driver_interface_node.cpp`, ~300 lines). It performs two conversions, all over `SensorDataQoS`:
   - industrial-camera RGB → mono8 grayscale published on `/camera/image_gray` (or `/image_gray`);
   - PX4 `SensorCombined` → `sensor_msgs/msg/Imu` published on `/px4/imu`, applying an FRD→FLU transform and timestamp correction.
4. **`VINS-Mono`** — git submodule (fork `Spaaaace-yyj/VINS-Mono`), a ROS 2 port of VINS-Mono: `feature_tracker` + `vins_estimator` + `pose_graph`. It has its own `.github/copilot-instructions.md` — read it before touching that submodule.
5. **`bringup`** — top-level launch composition only. `vins_system.launch.py` includes the camera, XRCE agent, driver_interface, and VINS launches; it also carries a **runtime copy** of the driver_interface params in `config/driver_interface.yaml`.

Data flow:

```
MindVision camera ── /image_raw ──► driver_interface ── /image_gray ─┐
                                                                     ├─► VINS-Mono ──► pose estimate
PX4 ── MicroXRCEAgent(serial) ── /fmu/out/sensor_combined ──► driver_interface ── /px4/imu ─┘
```

`px4_xrce_agent.launch.py` runs `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600` to bridge the serial PX4 link to DDS.

## Key Conventions

- **Coordinate frames.** PX4 uses FRD; VINS/ROS use FLU. `driver_interface` applies `(x, y, z) -> (x, -y, -z)` (`convert_frd_to_flu: true`) and tags the IMU with `frame_id: imu_link_flu`. Any VINS extrinsic calibration must be expressed in this **FLU** frame, not raw PX4 FRD.
- **IMU timestamp handling is the hard part.** `timestamp_mode` ∈ `auto` (default, recommended) / `first_arrival` / `px4` / `ros_now`. `ros_now` is diagnostics-only (reception-time jitter breaks VIO). The node preserves the camera's original `header.stamp` and, by default, drops non-monotonic IMU timestamps.
- **VINS config drives the pipeline.** The VINS YAML's `image_topic`/`imu_topic` must match the driver_interface outputs (`/image_gray`, `/px4/imu`). When first calibrating extrinsics use `estimate_extrinsic: 2`, then lock it to `0` with offline results.
- **Duplicated driver params.** `driver_interface/config/driver_interface.yaml` is the package default, but `bringup/config/driver_interface.yaml` is what the full-stack launch actually loads. Keep them in sync when changing topics/timestamps.
- **Language mix.** C++ source and ROS 2 launch/config files are the working surface; comments in launch/config are Chinese, C++ comments are English. Follow the surrounding file's language rather than normalizing.
- **C++ standard differs per package.** `driver_interface` and `mindvision_camera` build with C++17; the VINS-Mono submodule is C++14 with its own (upstream HKUST) style — do not "modernize" it.
