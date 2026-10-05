# Robomaster2027自主飞行无人机控制

自主飞行无人机上层飞行控制算法

## ubuntu20.04 分支：相机输出与可选图像桥接

本分支面向 Ubuntu 20.04 + ROS 2 Foxy，功能与 `main` 的相机输出补丁保持一致。
本补丁以 `ubuntu20.04` 为基准，不修改 VINS 子模块或标定内外参；保留总启动
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
git -c 'url.https://github.com/.insteadOf=git@github.com:' submodule update --init --recursive
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
