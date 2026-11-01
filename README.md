# Robomaster2027自主飞行无人机控制

自主飞行无人机上层飞行控制算法

## ubuntu20.04 分支：相机输出与可选图像桥接

本分支面向 Ubuntu 20.04 + ROS 2 Foxy，功能与 `main` 的相机输出补丁保持一致。
相机输出补丁以 `ubuntu20.04` 为基准，不修改 VINS 算法或标定内外参；保留总启动
配置中的 `timestamp_mode: px4`，接口包单独启动仍默认 `auto`。

相机默认仍然是全速 `rgb8`，图像话题 `/image_raw`，`driver_interface`
默认仍转换图像并发布 `/image_gray`。改成 `mono8` 后，相机话题仍然是
`/image_raw`，不会自动变成 `/image_gray`。

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| 相机 `full_speed` | `true` | 全速采集/发布；`false` 启用限帧 |
| 相机 `target_fps` | `30` | 限帧目标，整数 Hz；全速时忽略 |
| 相机 `output_encoding` | `rgb8` | `rgb8` 彩色或 `mono8` 灰度，SDK直接输出 |
| 相机 `image_topic` | `/image_raw` | 两种编码共用的图像话题 |
| 相机 `use_sensor_data_qos` | `false` | `true` 使用 Best Effort；VINS 建议开启 |
| 相机 `qos_depth` | `1` | 传输队列深度 |
| 接口 `enable_image_bridge` | `true` | `false` 不创建任何图像订阅/发布，仅转换IMU |

这些新参数是启动参数，改变后需要重启节点，不支持运行中切换图像格式。
YAML 参数会被保留，只有显式传入的 launch 参数才覆盖 YAML。

### 编译与单独测试

在仓库根目录执行（ROS 2 Foxy 及原项目依赖已安装，不能 source Humble）：

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install \
  --packages-up-to mindvision_camera driver_interface bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

全速 RGB（原行为）：

```bash
ros2 launch mindvision_camera mv_launch.py \
  full_speed:=true output_encoding:=rgb8
```

30 FPS RGB：

```bash
ros2 launch mindvision_camera mv_launch.py \
  full_speed:=false target_fps:=30 output_encoding:=rgb8
```

30 FPS 灰度、低延迟 QoS（话题仍为 `/image_raw`）：

```bash
ros2 launch mindvision_camera mv_launch.py \
  full_speed:=false target_fps:=30 output_encoding:=mono8 \
  use_sensor_data_qos:=true qos_depth:=1
```

全速灰度只需把 `full_speed` 改成 `true`。不要同时运行多个相机实例。

另开终端，仅启动IMU接口：

```bash
source install/setup.bash
ros2 launch driver_interface driver_interface.launch.py enable_image_bridge:=false
```

此时自己的 VINS 标定配置应写成：

```yaml
image_topic: "/image_raw"
imu_topic: "/px4/imu"
```

如果保留图像桥接，VINS 仍使用 `/image_gray`。不要把桥接输入和输出设置成同一
话题，也不要让两个节点同时发布同一灰度输出话题。

### 总 launch 与单文件配置

新增的 `src/bringup/config/camera_interface.yaml` 是可选的
“30 FPS 灰度 + IMU-only”配置，不改变默认启动行为。
其中相机参数放在 `/camera_node`，接口参数放在 `/driver_interface`。

```bash
SENSOR_CONFIG="$(ros2 pkg prefix --share bringup)/config/camera_interface.yaml"
ros2 launch bringup vins_system.launch.py \
  camera_config_file:="$SENSOR_CONFIG" \
  driver_config_file:="$SENSOR_CONFIG" \
  vins_config_file:=/绝对路径/你的VINS标定配置.yaml \
  use_pose_graph:=false use_rviz:=false
```

请把上述 VINS 配置路径替换成你已验证的文件，保留标定内外参和 `td`。
VINS 的 OpenCV YAML 与 ROS 节点参数 YAML 格式不同，仍通过 `vins_config_file`
单独传入；不混入这个传感器参数文件。

也可以使用总 launch 参数覆盖各自默认配置：

```bash
ros2 launch bringup vins_system.launch.py \
  camera_full_speed:=false camera_target_fps:=30 \
  camera_output_encoding:=mono8 camera_use_sensor_data_qos:=true \
  enable_image_bridge:=false \
  vins_config_file:=/绝对路径/你的VINS标定配置.yaml
```

总 launch 还支持 `camera_image_topic` 和 `camera_qos_depth`。

### 帧率、时间戳与 QoS 验证

```bash
ros2 topic hz /image_raw
ros2 topic echo /image_raw sensor_msgs/msg/Image --no-arr --qos-reliability best_effort
ros2 topic info -v /image_raw
ros2 node info /driver_interface
ros2 topic hz /px4/imu
```

Foxy 的 `echo` 没有 `--once`、`--field`，上述图像检查会持续输出消息头、
编码与尺寸（不输出像素数组），查看后按 Ctrl+C 停止，再执行下一条命令。

- `rgb8` 时 `step = width * 3`，`mono8` 时 `step = width`；宽高和话题不变。
- 30 FPS 设置是长期平均目标/上限，不会把低于30 FPS的输入补成30 FPS。
  100 FPS经软件限流时，相邻输出可能为30/40毫秒；日志分别显示
  `capture`（取到SDK帧）与 `publish`（发布到ROS）的频率。
- SDK只保证部分相机支持 `CameraSetFrameRate`。不支持时会打印警告并在ISP
  处理前跳过多余帧，不用采集线程 `sleep()`。这降低处理和ROS传输负载，
  **不一定降低相机到主机的USB/网口带宽**；SDK灰度输出也不等于传感器原始流改变。
- 关闭图像桥接后，`ros2 node info /driver_interface` 不应出现图像订阅/发布，
  IMU链路不受影响。
- Best Effort 发布器不能连接要求 Reliable 的订阅器。检查实际运行的
  feature_tracker 图像订阅端QoS；RViz图像显示也要设置为 Best Effort。
- 保留已有硬件时间戳映射（相机tick单位0.1毫秒），不会用发布时间重打时间戳。
  这仍是首帧到达时间锚定，不等于相机与IMU硬件同步；`timestamp_mode`、标定
  `td` 与IMU坐标变换仍需使用已验证设置。更改曝光、翻转、分辨率后重新验证标定。

### 无硬件逻辑测试

```bash
g++ -std=c++14 -Wall -Wextra -Werror \
  src/mindvision_camera/test/frame_rate_limiter_test.cpp \
  -o /tmp/rm_camera_frame_rate_test
/tmp/rm_camera_frame_rate_test
python3 -m unittest discover -s tests -p 'test_sensor_launch_parameters.py' -v
```

测试覆盖全速/限帧、100/200 Hz降到30 Hz、低帧率输入、重复时间戳/时间跳变，
以及 YAML/启动参数优先级与类型转换。Python测试使用轻量替身检查配置逻辑，
不替代真实ROS launch、SDK相机和DDS集成测试。

### Foxy 兼容方式

launch 在 `OpaqueFunction` 中把布尔、整数和浮点参数转换成 Python 原生类型，
再传入节点参数字典，未显式传参时保留 YAML。无需依赖
`launch_ros.parameter_descriptions.ParameterValue` 的版本差异。
时间消息使用 Foxy 支持的 `rclcpp::Time` 转换/手动填写 `sec`、`nanosec`，
没有引入 `Time::to_msg()`。启动时格式/帧率/QoS参数设为只读；改配置后重启节点。
新增代码兼容 Python 3.8、相机 C++14 和接口 C++17。

## 编译

### 环境依赖

此分支使用 Ubuntu 20.04 + ROS 2 Foxy，Python 3.8 / GCC 9。
相机包使用 C++14，driver_interface 使用 C++17，Ubuntu 20.04 的编译器支持两者。
下面是已有 ROS 2 Foxy 环境的依赖安装步骤。

**1.Ubuntu20.04**

**2.Ros2Foxy**

参照小鱼的一键安装

```bash
#克隆
git clone --branch ubuntu20.04 https://github.com/Spaaaace-yyj/RM_flying_robot_control.git
cd RM_flying_robot_control
#确保安装了ROS2 Foxy
sudo apt update

sudo apt install -y \
  git build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-opencv \
  libopencv-dev libeigen3-dev libceres-dev \
  libboost-filesystem-dev \
  libboost-program-options-dev \
  libboost-system-dev
  
sudo apt install -y \
  ros-foxy-ament-cmake \
  ros-foxy-ament-cmake-auto \
  ros-foxy-ament-index-cpp \
  ros-foxy-ament-index-python \
  ros-foxy-rclcpp \
  ros-foxy-rclcpp-components \
  ros-foxy-builtin-interfaces \
  ros-foxy-std-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-visualization-msgs \
  ros-foxy-cv-bridge \
  ros-foxy-image-transport \
  ros-foxy-image-transport-plugins \
  ros-foxy-camera-info-manager \
  ros-foxy-camera-calibration \
  ros-foxy-tf2 \
  ros-foxy-tf2-ros \
  ros-foxy-rosidl-default-generators \
  ros-foxy-rosidl-default-runtime \
  ros-foxy-ros-environment \
  ros-foxy-launch \
  ros-foxy-launch-ros \
  ros-foxy-ros2launch \
  ros-foxy-rviz2 \
  ros-foxy-rqt-image-view \
  ros-foxy-ament-lint-auto \
  ros-foxy-ament-lint-common
  
sudo rosdep init
rosdep update

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro foxy \
  -y
```

## 源码管理、开源协议与第三方来源

### VINS-Mono 的来源与版本

`src/VINS-Mono` 现在是由本仓库最外层 Git 直接管理的普通源码目录，
不再是子模块；目录中不包含独立的 `.git`，根目录也不再需要 `.gitmodules`。
新克隆只需 `git clone`，无需 `git submodule update`。后续在该目录修改代码时，
直接在本仓库提交；切换外层分支会同时切换对应的 VINS-Mono 源码。

本次迁移分别完整导入两个外层分支原先锁定的版本，保留所有受跟踪文件及文件权限：

| 外层分支 | 原 VINS-Mono 提交（导入基线） |
| --- | --- |
| `main`（Ubuntu 22.04 / ROS 2 Humble） | [`ec59526549730b10cbab9428d79d4a211c718335`](https://github.com/Spaaaace-yyj/VINS-Mono/commit/ec59526549730b10cbab9428d79d4a211c718335) |
| `ubuntu20.04`（Ubuntu 20.04 / ROS 2 Foxy） | [`c5884d0514749d88113b191bd89b0b0b5488b491`](https://github.com/Spaaaace-yyj/VINS-Mono/commit/c5884d0514749d88113b191bd89b0b0b5488b491) |

迁移来源为 [Spaaaace-yyj/VINS-Mono](https://github.com/Spaaaace-yyj/VINS-Mono)，
原始算法来自 [HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)。
原作者为 Tong Qin、Peiliang Li、Zhenfei Yang、Shaojie Shen（HKUST Aerial Robotics Group）。
本次仅改变源码的 Git 管理方式，不合并两个版本，也不改动 VINS-Mono 算法、标定或资源文件。
外层仓库已有提交历史保留；迁移没有导入子仓库的提交历史，旧子仓库也未被删除。

### 许可与第三方声明

VINS-Mono 上游采用 **GNU GPLv3**；完整许可证保留在
[`src/VINS-Mono/LICENCE`](src/VINS-Mono/LICENCE)，原说明保留在
[`src/VINS-Mono/README.md`](src/VINS-Mono/README.md)。复制、修改或分发这部分代码时，
请遵循其许可证，保留原有版权、许可和署名声明。学术使用请引用原始 VINS-Mono 论文，
BibTeX 保留在 [`src/VINS-Mono/support_files/paper_bib.txt`](src/VINS-Mono/support_files/paper_bib.txt)。

VINS-Mono 内含 camodocal、DBoW2、DUtils、DVision 等第三方代码；各文件原有声明完整保留，
应同时查看相应文件头。例如
[`DBoW2.h`](src/VINS-Mono/pose_graph/src/ThirdParty/DBoW/DBoW2.h)
保留了 **CC BY-NC-SA 3.0** 声明，
[`DUtils.h`](src/VINS-Mono/pose_graph/src/ThirdParty/DUtils/DUtils.h) 与
[`DVision.h`](src/VINS-Mono/pose_graph/src/ThirdParty/DVision/DVision.h)
保留了 **LGPLv3 或后续版本** 声明。本次迁移不改变这些文件的原有许可。
本仓库其他包继续以各自的 `LICENSE`、`LICENCE`、`package.xml` 和文件头声明为准；
本节不为所有代码统一指定新的许可证。

### 已有子模块克隆的更新

旧工作区中的子模块可能有尚未推送的修改、分支或标定文件。更新前先备份整个旧工作区，
最稳妥的方式是在另一个目录重新克隆，再按需迁入自己的修改：

```bash
# Humble；也可使用 --branch ubuntu20.04 克隆 Foxy 版本
git clone --branch ubuntu20.04 https://github.com/Spaaaace-yyj/RM_flying_robot_control.git RM_flying_robot_control-new
```

新克隆的两个分支均直接包含各自的 VINS-Mono 源码，无需初始化子模块。


## VINS-Fusion：Ubuntu 20.04 / ROS 2 Foxy

本分支新增 `src/vins_fusion_ros2`，从 `main` 的 `d986621` 中已有的
VINS-Fusion 源码移植（导入目录基线 `7338c212abd0e8c38315d8d272bbf136ce1d6529`）。
它是普通源码目录，由外层 Git 管理。原有 Foxy 相机、接口与 VINS-Mono 版本保持不变。

### 依赖与编译

目标环境为 Ubuntu 20.04、ROS 2 Foxy、GCC 9、C++14；使用系统 OpenCV 4.2、
Ceres 1.14、Eigen 3.3.7 和 glog，不要求源码安装 Humble 使用的 Ceres 2.1 / Eigen 3.3.9。
默认使用 CPU；`use_gpu` 和 `use_gpu_acc_flow` 均应为 `0`。
原参数 `use_gpu_ceres` 只选择 `DENSE_QR` / `DENSE_SCHUR`，本代码没有启用 Ceres CUDA 后端。

```bash
source /opt/ros/foxy/setup.bash
sudo apt update
sudo apt install -y \
  build-essential cmake python3-colcon-common-extensions \
  libopencv-dev libceres-dev libeigen3-dev libgoogle-glog-dev libboost-filesystem-dev \
  ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-message-filters \
  ros-foxy-tf2-ros ros-foxy-rcpputils ros-foxy-ament-cmake-gtest
# 也可让 rosdep 按此包的 package.xml 安装依赖
rosdep install --from-paths src/vins_fusion_ros2 --ignore-src --rosdistro foxy -y
colcon build --packages-select vins_fusion_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

切换 Humble/Foxy 或使用不同 OpenCV/Ceres 后，应使用新的工作区或清理该包旧的
构建缓存再编译，不要混用之前的 `build/`、`install/`。只编译此包不会启动相机或飞控。

### CMake 3.16 / 4.4.3 兼容

该包支持 Ubuntu 20.04 自带的 CMake 3.16 和 CMake 4.4.3，不需要降级 CMake。
两层 `CMakeLists.txt` 使用 `3.16...3.22` 声明包自身的策略版本；CMake 4 下通过
`cmake/foxy_compat.cmake` 为旧的外部依赖设置最低策略版本 `3.5`，避免 Focal/Foxy
依赖中低于 3.5 的声明被拒绝。该设置只作用于本包配置目录及其依赖，不强制覆盖用户
更高的策略下限，也不修改系统安装的依赖文件。Boost 使用 Focal 的 1.71 配置包，
不再依赖新 CMake 中已弃用的 `FindBoost` 模块。

更新后清理这个包的 CMake 缓存，再单独编译：

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select vins_fusion_ros2 --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

这里的兼容修改仅针对 `vins_fusion_ros2`；工作区中其他包需要另行检查自己的构建配置。

### 启动与参数

```bash
# 实机：使用你已完成双目标定、相机-IMU标定的 OpenCV YAML
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py \
  config_file:=/绝对路径/你的配置.yaml use_sim_time:=false

# EuRoC 等 ROS 2 bag：默认配置为包内 EuRoC 双目+IMU示例
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py use_sim_time:=true
# 另一个已 source 工作区的终端
ros2 bag play /绝对路径/你的ROS2_bag --clock
```

`config_file` 为 VINS 的 OpenCV YAML，不是 ROS 节点参数 YAML；相机标定文件的相对路径
以该 YAML 所在目录为基准，也支持绝对路径。`output_path` 支持 `~/`，会创建目录并检查
写入权限；启动时会重新创建 `vio.csv`，需要保留的旧结果应先另存。
`use_sim_time` 默认为 `false`，可通过 launch 设置
`world_frame_id` / `body_frame_id` / `camera_frame_id`（默认 `world` / `body` / `camera`）。

输入由 YAML 中的 `image0_topic` / `image1_topic` / `imu_topic` 决定；图像和 IMU 均使用
Best Effort 订阅，可接收 Reliable 或 Best Effort 发布端。支持双目+IMU、单目+IMU及纯双目。
对于你的 D435 + PX4 IMU，应配置 `imu: 1`、`num_of_cam: 2`、正确的左右红外话题与
`imu_topic`，并使用你自己的时序、噪声及外参标定。上游名为
`realsense_stereo_imu_config.yaml` 的示例实际写了 `imu: 0`，不能仅根据文件名判断模式。

当前节点输出 `/odometry`、`/imu_propagate`、`/path`、`/image_track` 等话题及
world→body→camera TF。这里迁移的是 VIO 节点；PX4 接口使用时应按实际输出话题配置桥接。

### Foxy 适配与检查

- 使用原生 rclcpp 订阅与 `SimpleFilter` 转发双目图像，保持同一图像回调组，避开 Foxy
  `message_filters::Subscriber` 不支持 `SubscriptionOptions` 的接口差异。
- 修正 ROS 2 消息头文件、补齐包依赖、显式链接线程/Boost，并把共享库安装到标准 `lib/`。
- 估计器开始接收数据前初始化状态；失败恢复使用已持锁的重置函数，避免重复锁定。
- 检查配置、相机数量、标定路径和输出目录；外部特征消息缺少通道或通道过短时丢弃，避免越界。

```bash
colcon test --packages-select vins_fusion_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

`.github/workflows/vins-fusion-foxy.yml` 分别使用系统 CMake 3.16 和 CMake 4.4.3，
在真实 Focal/Foxy 容器中构建共享库，运行转换测试，
并通过 Best Effort 合成图像/IMU检查三种模式的数据接收、跟踪图像输出和正常退出；另外
通过安装目录运行 launch，检查参数解析和共享库加载。合成数据检查不等于实机精度、
EuRoC轨迹精度或无人机飞行验收。

### VINS-Fusion 来源与许可

ROS 2 移植来源为 [yangfuyuan/vins_fusion_ros2](https://github.com/yangfuyuan/vins_fusion_ros2)，
原始算法来自 [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)。
GPLv3 许可证完整保留在 [`src/vins_fusion_ros2/LICENCE`](src/vins_fusion_ros2/LICENCE)，
原作者与第三方版权声明保留在源码和包内 README；本次移植不改变其原有许可。

## D435 / D430 系列相机启动（Foxy）

`src/d435_bringup` 已从 `main` 移植到本分支，适配 ROS 2 Foxy 和 CMake 4.4.3。
包名和入口仍是 `d435_bringup` / `d435.launch.py`，依赖官方 `realsense2_camera`。

```bash
source /opt/ros/foxy/setup.bash
sudo apt install ros-foxy-realsense2-camera ros-foxy-diagnostic-updater python3-yaml
colcon build --packages-select d435_bringup --cmake-clean-cache
source install/setup.bash
ros2 launch d435_bringup d435.launch.py
# 修改源目录配置并显式加载：
ros2 launch d435_bringup d435.launch.py \
  config_file:=$PWD/src/d435_bringup/config/d435.yaml
```

默认保留主分支实际 YAML：深度/左右红外 848×480×90，RGB 848×480×60，点云及深度对齐开启。
驱动版本对应的参数接口自动识别；兼容 3.x 的旧规格参数、早期 4.x 共用 profile 和较新
驱动的分开 profile。可通过 `driver_api:=legacy/profile/split` 中的一个实际值手动指定。
数字串行号、空字符串等通过参数文件保持类型，避免 Foxy launch 的字符串推断差异。

如用于刚才的 VINS 双目标定，请自行在配置中关闭不需要的 RGB、点云、对齐等功能，并设置
适合采集的帧率。左右图像的话题随驱动版本而异，应先检查 `ros2 topic list` 再填入 VINS 配置。
详细参数、驱动接口差异和检查范围见 [d435_bringup 说明](src/d435_bringup/README.md)。
包的 MIT 许可证保留在 [LICENSE](src/d435_bringup/LICENSE)。
