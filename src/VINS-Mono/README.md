# VINS-Mono ROS 2

这是 [HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
的 ROS 2 移植版本，目标平台为 Ubuntu 22.04 + ROS 2 Humble。VINS-Mono 是一个面向单目相机与
IMU 的滑动窗口视觉惯性里程计，支持在线初始化、相机—IMU 外参估计、时间偏移估计、回环检测和
4-DoF 位姿图优化。

> 本仓库仍处于移植和验证阶段。建议先完成 EuRoC 离线测试，再接入自己的 PX4 与工业相机。
> 原始算法、论文与作者信息请参阅上游仓库；本仓库的修改主要集中在 ROS 2 节点、消息、QoS、
> launch 和构建系统。

## 1. 系统结构

```text
sensor_msgs/Image ──> feature_tracker ──> /feature_tracker/feature ──┐
                                                                    ├─> vins_estimator
sensor_msgs/Imu ─────────────────────────────────────────────────────┘        │
                                                                              ├─> /vins_estimator/odometry
                                                                              └─> keyframe data
                                                                                       │
                                                                                       v
                                                                                  pose_graph
                                                                                       │
                                                                                       └─> /pose_graph/match_points
```

主要包：

| 包 | 用途 |
| --- | --- |
| `camera_model` | PINHOLE、MEI 等相机模型与标定工具 |
| `feature_tracker` | KLT 光流前端，生成带像素坐标和速度的特征消息 |
| `vins_estimator` | IMU 预积分、初始化、滑动窗口非线性优化与边缘化 |
| `pose_graph` | DBoW2 回环检测和 4-DoF 位姿图优化 |
| `benchmark_publisher` | EuRoC 真值轨迹可视化 |
| `data_generator` | 合成 IMU/特征数据，用于基础联调 |
| `ar_demo` | AR 叠加演示 |

节点内部输出使用 ROS 2 私有话题名 `~/...`，因此默认节点名下的话题为
`/feature_tracker/*`、`/vins_estimator/*` 和 `/pose_graph/*`。不要再给这三个节点额外设置同名
namespace，否则会得到重复路径，例如 `/vins_estimator/vins_estimator/odometry`。

## 2. 环境与编译

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble Desktop
- OpenCV 4、Eigen 3、Ceres 2.x
- `colcon` 与 `rosdep`

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-dev-tools \
  ros-humble-cv-bridge ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-visualization-msgs ros-humble-rviz2 \
  libopencv-dev libeigen3-dev libceres-dev

mkdir -p ~/vins_ws/src
cd ~/vins_ws/src
git clone https://github.com/Spaaaace-yyj/VINS-Mono.git

cd ~/vins_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

每次打开新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source ~/vins_ws/install/setup.bash
```

## 3. 先做基础检查

不接真实设备时可以运行合成数据测试。该测试绕过 `feature_tracker`，直接向估计器发送合成特征，
主要用于检查节点、消息格式与优化器是否能启动，不等同于真实视觉惯性测试。

```bash
ros2 launch vins_estimator simulation.launch.py
```

如果运行环境没有桌面，`data_generator` 中的 OpenCV 窗口可能无法打开，此时优先使用 EuRoC bag
测试。

## 4. EuRoC 离线测试

### 4.1 准备 rosbag2

下载 [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)。
若下载的是 ROS 1 `.bag`，可用 `rosbags` 转成 rosbag2：

```bash
python3 -m pip install rosbags
rosbags-convert MH_01_easy.bag --dst MH_01_easy_ros2
ros2 bag info MH_01_easy_ros2
```

默认 EuRoC 配置订阅：

- 图像：`/cam0/image_raw`
- IMU：`/imu0`

如果 bag 内话题名不同，复制 `config/euroc/euroc_config.yaml`，修改 `image_topic` 和
`imu_topic`，不要直接反复修改仓库内的基准配置。

### 4.2 先关闭回环和 RViz2

终端 1：

```bash
ros2 launch vins_estimator vins.launch.py \
  use_pose_graph:=false \
  use_rviz:=false \
  use_sim_time:=true
```

终端 2：

```bash
ros2 bag play MH_01_easy_ros2 --clock
```

终端 3：

```bash
ros2 topic hz /feature_tracker/feature
ros2 topic hz /vins_estimator/odometry
ros2 topic echo /vins_estimator/odometry --once
ros2 run tf2_ros tf2_echo world body
```

VIO 正常后再打开回环和 RViz2：

```bash
ros2 launch vins_estimator vins.launch.py \
  use_pose_graph:=true \
  use_rviz:=true \
  use_sim_time:=true
```

launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `config_file` | EuRoC 配置 | VINS YAML 的绝对路径 |
| `vins_folder` | 安装后的包目录 | 查找鱼眼 mask 等资源 |
| `use_pose_graph` | `true` | 是否启动回环节点 |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_sim_time` | `false` | 播放带 `--clock` 的 bag 时设为 `true` |

## 5. 接入自己的相机和 IMU

建议的数据链路：

```text
工业相机驱动 ── sensor_msgs/msg/Image ──> /camera/image_raw ──> feature_tracker

PX4 /fmu/out/sensor_combined
        └─> px4_vins_bridge ── sensor_msgs/msg/Imu ──> /px4/imu ──> vins_estimator
```

不要自己生成 `/feature_tracker/feature`。真实设备中应由 `feature_tracker` 根据原始图像生成该消息。

### 5.1 图像要求

- 消息类型为 `sensor_msgs/msg/Image`。
- 推荐 `mono8`；`bgr8`/`rgb8` 会由 `cv_bridge` 转灰度。
- `header.stamp` 必须是曝光/采样时刻，不能是回调完成时刻。
- YAML 中 `image_width`、`image_height` 必须与实际图像一致。
- 建议全局快门，首次测试 20–30 Hz。
- 相机标定从 VINS YAML 读取；当前节点不读取 `CameraInfo`。

原始相机订阅使用 `rclcpp::SensorDataQoS()`，可兼容常见相机驱动的 Best Effort 发布端。

### 5.2 PX4 `SensorCombined` 转 `sensor_msgs/Imu`

PX4 的 `gyro_rad` 单位已经是 rad/s，`accelerometer_m_s2` 单位已经是 m/s²。PX4 机体系通常为
FRD（前、右、下），而 ROS 机体系通常采用 FLU（前、左、上）。只做轴变换时：

```cpp
imu.angular_velocity.x =  msg.gyro_rad[0];
imu.angular_velocity.y = -msg.gyro_rad[1];
imu.angular_velocity.z = -msg.gyro_rad[2];

imu.linear_acceleration.x =  msg.accelerometer_m_s2[0];
imu.linear_acceleration.y = -msg.accelerometer_m_s2[1];
imu.linear_acceleration.z = -msg.accelerometer_m_s2[2];

// SensorCombined 不提供姿态；告诉消费者 orientation 无效。
imu.orientation_covariance[0] = -1.0;
```

水平静止时，转换后的加速度应大致为 `[0, 0, +9.8]` m/s²，模长约为 9.8。不要再乘重力常数，
也不要把角速度从度每秒重复转换为弧度每秒。

时间戳是接入中最重要的部分：PX4 的 `timestamp` 是飞控启动后的微秒计时，相机往往使用 ROS 系统
时间或硬件时钟，两者不能直接比较。正式使用时必须把 PX4 采样时刻映射到与相机相同的 ROS 时基：

```text
t_ros = t_px4 + clock_offset
```

其中 `clock_offset` 来自稳定的时间同步机制，并且输出时间戳必须严格单调。仅做连通性测试时可临时把
`imu.header.stamp` 设为桥接节点的 `now()`，但这会引入 DDS/USB 传输抖动，不适合作为最终 VIO
方案。`estimate_td: 1` 只能补偿近似固定的相机—IMU偏移，不能修复变化的传输延迟。

### 5.3 自定义配置模板

先复制一份最接近相机模型的 YAML：

```bash
cp ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml \
   ~/vins_ws/src/VINS-Mono/config/my_sensor.yaml
```

至少检查以下字段：

```yaml
imu_topic: "/px4/imu"
image_topic: "/camera/image_raw"
output_path: "/home/YOUR_USER/vins_output/"  # 必须是存在或可创建的绝对路径

model_type: PINHOLE
camera_name: industrial_camera
image_width: 1280       # 改为实际值
image_height: 1024      # 改为实际值

# 使用真实标定值，下面只是字段示意
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: 600.0
   fy: 600.0
   cx: 640.0
   cy: 512.0

estimate_extrinsic: 2   # 仅用于首次联调；稳定运行建议使用离线标定结果

max_cnt: 150
min_dist: 30
freq: 20
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

max_solver_time: 0.04   # 单位是秒，不是毫秒
max_num_iterations: 8
keyframe_parallax: 10.0

# 以下只是联调初值，最终应使用实际 IMU 的 Allan 方差结果
acc_n: 0.2
gyr_n: 0.02
acc_w: 0.002
gyr_w: 4.0e-5
g_norm: 9.81

loop_closure: 0
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/YOUR_USER/vins_output/pose_graph/"

estimate_td: 1
td: 0.0
rolling_shutter: 0
rolling_shutter_tr: 0.0

save_image: 0
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

YAML 不会自动展开 `$HOME` 或 `~`，请填写绝对路径。

### 5.4 外参约定

配置中的外参是“相机坐标到 IMU 坐标”的变换：

```text
p_I = R_IC * p_C + t_IC
```

如果标定工具给出的是 `T_CI`，需要先求逆：

```text
R_IC = R_CI^T
t_IC = -R_CI^T * t_CI
```

外参中的 IMU 坐标系必须对应经过 FRD→FLU 转换后的 IMU 消息坐标系。初次使用
`estimate_extrinsic: 2` 时，应在启动阶段做充分的多轴旋转；获得稳定结果后建议用 Kalibr 等工具做
离线标定，并改为 `estimate_extrinsic: 0` 或 `1`。

## 6. 推荐测试顺序

1. EuRoC：只启用 `feature_tracker + vins_estimator`，确认算法主链路。
2. 相机单测：确认频率、编码、分辨率和曝光时间戳。
3. PX4 桥接单测：确认单位、FRD→FLU、静止重力方向和时间戳。
4. 录制自己的 rosbag2，先离线回放和调参。
5. 实时 VIO 稳定后再启用回环。

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /px4/imu
ros2 topic info -v /camera/image_raw
ros2 topic info -v /px4/imu
ros2 topic echo /px4/imu --once

ros2 bag record /camera/image_raw /px4/imu
```

建议验收条件：

- 相机稳定大于 20 Hz，IMU 接近或高于 200 Hz。
- 静止时角速度接近 0，加速度模长约 9.8 m/s²。
- 水平静止且使用 FLU 时，加速度 z 约为 +9.8 m/s²。
- 两路时间戳同一时基、严格单调，相机—IMU延迟基本恒定。
- `/feature_tracker/feature` 与 `/vins_estimator/odometry` 持续输出。

## 7. 主要话题

| 节点 | 方向 | 话题 | 类型 |
| --- | --- | --- | --- |
| `feature_tracker` | 订阅 | YAML 的 `image_topic` | `sensor_msgs/msg/Image` |
| `feature_tracker` | 发布 | `/feature_tracker/feature` | `sensor_msgs/msg/PointCloud2` |
| `feature_tracker` | 发布 | `/feature_tracker/feature_img` | `sensor_msgs/msg/Image` |
| `feature_tracker` | 发布 | `/feature_tracker/restart` | `std_msgs/msg/Bool` |
| `vins_estimator` | 订阅 | YAML 的 `imu_topic` | `sensor_msgs/msg/Imu` |
| `vins_estimator` | 发布 | `/vins_estimator/odometry` | `nav_msgs/msg/Odometry` |
| `vins_estimator` | 发布 | `/vins_estimator/imu_propagate` | `nav_msgs/msg/Odometry` |
| `vins_estimator` | 发布 | `/vins_estimator/path` | `nav_msgs/msg/Path` |
| `vins_estimator` | 发布 | `/vins_estimator/point_cloud` | `sensor_msgs/msg/PointCloud2` |
| `pose_graph` | 发布 | `/pose_graph/pose_graph_path` | `nav_msgs/msg/Path` |
| `pose_graph` | 发布 | `/pose_graph/match_points` | `sensor_msgs/msg/PointCloud2` |

外部相机与 IMU 订阅使用 Sensor Data QoS（Best Effort、较小队列）。节点内部特征、里程计和回环消息
保持 Reliable。若没有数据，请首先用 `ros2 topic info -v TOPIC` 对比发布端与订阅端 QoS。

## 8. 常见问题

### 有图像但没有特征

检查 `image_topic`、图像编码、分辨率与时间戳；再查看 `/feature_tracker/feature_img`。如果相机发布
Best Effort，本仓库已使用 Sensor Data QoS 与其兼容。

### 有特征但没有里程计

检查 IMU 是否到达、时间戳是否与图像同一时基、静止重力方向是否正确。VINS 初始化需要足够的平移
和多轴转动；纯旋转或长时间静止可能无法初始化。

### 回放旧 bag 时 RViz2 没有内容

launch 使用 `use_sim_time:=true`，同时 `ros2 bag play ... --clock`。RViz2 的 Fixed Frame 应为
`world`。

### 输出文件写不出来

`output_path` 和 `pose_graph_save_path` 必须是当前用户可写的绝对路径。仓库的 EuRoC 默认路径是
`/tmp/vins_output/`；正式实验请改到持久目录。

### 只想测试 VIO，不想加载 DBoW2

```bash
ros2 launch vins_estimator vins.launch.py use_pose_graph:=false
```

## 9. 致谢与许可

原始 VINS-Mono 作者为 Tong Qin、Peiliang Li、Zhenfei Yang 和 Shaojie Shen（HKUST Aerial
Robotics Group）。项目使用 Ceres Solver、DBoW2 和 camodocal。若用于学术研究，请引用原始
[VINS-Mono 论文](https://ieeexplore.ieee.org/document/8421746)；BibTeX 位于
[`support_files/paper_bib.txt`](support_files/paper_bib.txt)。

本项目遵循 [GPLv3](LICENCE) 许可。
