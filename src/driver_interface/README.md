# driver_interface

面向 ROS 2 Humble、PX4 1.14 和 VINS-Mono ROS 2 的传感器接口节点。

节点完成两条数据转换：

```text
工业相机 RGB Image                    mono8 Image
/camera/image_raw  -> driver_interface -> /camera/image_gray

PX4 SensorCombined                       sensor_msgs/Imu
/fmu/out/sensor_combined -> driver_interface -> /px4/imu
```

## 1. 功能

- 将 `rgb8`、`bgr8`、`rgba8`、`bgra8` 等图像转换为 `mono8`；
- 保留相机消息原始 `header.stamp`，默认保留原始 `frame_id`；
- 将 PX4 `gyro_rad` 和 `accelerometer_m_s2` 转为 `sensor_msgs/msg/Imu`；
- 默认进行 PX4 FRD 到 ROS FLU 的坐标变换：`(x, y, z) -> (x, -y, -z)`；
- 使用 PX4 1.14 `SensorCombined.timestamp`，而不是每帧都使用回调到达时间；
- 相机、PX4 输入以及转换后的输出均采用 Sensor Data QoS；
- 检测并默认丢弃非单调 IMU 时间戳。

`SensorCombined` 不包含姿态，因此输出消息设置：

```text
orientation_covariance[0] = -1
```

## 2. 放入工作空间

解压后将 `driver_interface` 文件夹复制到：

```text
~/Code/HBUT2025_rm_vision/Dron/px4_ros2_ws/src/driver_interface
```

确保工作空间中已有与 PX4 1.14 匹配的 `px4_msgs` `release/1.14` 分支。

安装依赖并编译：

```bash
cd ~/Code/HBUT2025_rm_vision/Dron/px4_ros2_ws

source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-up-to driver_interface \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 3. DDS 环境

按当前 PX4 链路设置：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 4. 启动

默认话题启动：

```bash
ros2 launch driver_interface driver_interface.launch.py
```

如果工业相机实际话题为 `/mv_25001498/image_raw`：

```bash
ros2 launch driver_interface driver_interface.launch.py \
  camera_input_topic:=/mv_25001498/image_raw
```

也可以直接运行节点并加载参数文件：

```bash
ros2 run driver_interface driver_interface_node --ros-args \
  --params-file "$(ros2 pkg prefix --share driver_interface)/config/driver_interface.yaml"
```

## 5. 配置 VINS-Mono

在自己的 VINS 配置中修改：

```yaml
image_topic: "/camera/image_gray"
imu_topic: "/px4/imu"
```

第一次测试自己的相机和 IMU 外参时可以使用：

```yaml
estimate_extrinsic: 2
```

稳定后应使用离线标定结果并改为：

```yaml
estimate_extrinsic: 0
```

外参必须对应本节点输出的 `imu_link_flu` 坐标系，而不是 PX4 原始 FRD 坐标系。

## 6. 时间戳模式

参数 `timestamp_mode` 有三种值：

### `auto`（默认，推荐）

节点比较第一条 `SensorCombined.timestamp` 和当前 ROS 时间：

- 如果两者相差不超过 `auto_direct_threshold_sec`（默认 10 秒），说明 PX4/uXRCE-DDS 已经将时间映射到 ROS 时基，直接使用 PX4 时间；
- 如果相差很大，说明收到的可能是飞控启动时间，自动退化为下面的 `first_arrival` 固定偏移方式。

### `first_arrival`（快速联调）

第一条 IMU 到达时只计算一次：

```text
clock_offset = ros_now - px4_timestamp
```

后续时间戳使用：

```text
t_ros = t_px4 + clock_offset + timestamp_offset_sec
```

这样保留 PX4 IMU 的采样间隔，不会把每次 DDS/串口传输抖动写入时间戳。但第一次消息的传输延迟会成为固定误差，因此它适合快速跑通，不替代硬件同步。

### `px4`（正式同步链路）

```text
t_ros = t_px4 + timestamp_offset_sec
```

只有在 PX4 时间已经被映射到与相机相同的时基，或者你已经准确计算出 `timestamp_offset_sec` 时使用。

### `ros_now`（只用于连通测试）

使用回调到达时间。DDS、USB 和串口抖动会进入时间戳，不适合最终 VIO。

启动时覆盖参数：

```bash
ros2 launch driver_interface driver_interface.launch.py \
  timestamp_mode:=px4 \
  timestamp_offset_sec:=0.0
```

VINS 的 `estimate_td: 1` 只能补偿近似固定的相机—IMU时间偏移，不能消除随机传输抖动。

## 7. 检查输出

确认输入存在：

```bash
ros2 topic info -v /camera/image_raw
ros2 topic info -v /fmu/out/sensor_combined
```

检查灰度图像：

```bash
ros2 topic hz /camera/image_gray
ros2 topic echo /camera/image_gray --once --field encoding
ros2 run rqt_image_view rqt_image_view /camera/image_gray
```

编码应为：

```text
mono8
```

检查 IMU：

```bash
ros2 topic hz /px4/imu
ros2 topic echo /px4/imu --once
```

水平静止且采用 FLU 时，预期：

- 角速度接近 `(0, 0, 0)` rad/s；
- 加速度模长约为 `9.8 m/s^2`；
- 根据传感器安装方向，水平且 IMU `z` 轴朝上时，`z` 应接近 `+9.8 m/s^2`。

检查图像与 IMU 时间戳是否处于同一时基：

```bash
ros2 topic echo /camera/image_gray --once --field header.stamp
ros2 topic echo /px4/imu --once --field header.stamp
```

两者不应相差飞控启动时间或 Unix 纪元量级。

## 8. 参数表

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_input_topic` | `/camera/image_raw` | 工业相机 RGB 图像 |
| `gray_output_topic` | `/camera/image_gray` | VINS 灰度图像 |
| `px4_imu_input_topic` | `/fmu/out/sensor_combined` | PX4 SensorCombined |
| `imu_output_topic` | `/px4/imu` | VINS IMU |
| `image_frame_id` | 空 | 空时保留原相机 frame_id |
| `imu_frame_id` | `imu_link_flu` | 转换后的 IMU 坐标系 |
| `convert_frd_to_flu` | `true` | Y、Z 轴取反 |
| `timestamp_mode` | `auto` | IMU 时间戳策略 |
| `timestamp_offset_sec` | `0.0` | 手动附加固定偏移，单位秒 |
| `auto_direct_threshold_sec` | `10.0` | 自动模式判断同一时基的阈值 |
| `drop_non_monotonic_imu` | `true` | 丢弃时间戳倒退/重复消息 |
| `angular_velocity_variance` | `0.0` | 角速度协方差对角元素 |
| `linear_acceleration_variance` | `0.0` | 加速度协方差对角元素 |

## 9. 常见问题

### VINS 一直显示 waiting for image and imu

依次检查：

```bash
ros2 topic hz /camera/image_gray
ros2 topic hz /px4/imu
ros2 topic info -v /camera/image_gray
ros2 topic info -v /px4/imu
```

同时确认 VINS YAML 的话题名完全一致。

### 图像存在但 VINS 频繁重置

检查相机 `header.stamp` 是否单调。节点保留相机驱动给出的曝光时间戳，不会用回调时间覆盖它。

### IMU 频率正常但无法初始化

重点检查：

- 图像和 IMU 是否同一时基；
- PX4 FRD→FLU 是否与实际安装和外参标定一致；
- 陀螺仪单位是否为 rad/s；
- 加速度单位是否为 m/s²且包含重力；
- 相机—IMU外参和相机内参是否正确。
