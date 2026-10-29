# D435 YAML 启动包

适用：Ubuntu 22.04、ROS 2 Humble，以及已安装并能正常取图的官方
`realsense2_camera` 驱动。此包只负责读取配置和启动现有驱动。

默认开启深度图、左右红外图，均为 640×480、30 FPS；关闭 RGB、点云、
深度对齐和后处理。红外投射器开启，自动曝光开启。

## 安装和启动

把整个 `d435_bringup` 目录放到现有工作空间的 `src/` 下。以
`~/realsense_ws` 为例，最终文件应在
`~/realsense_ws/src/d435_bringup/package.xml`：

```bash
mkdir -p ~/realsense_ws/src
unzip ~/下载/d435_bringup.zip -d ~/realsense_ws/src
cd ~/realsense_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d435_bringup
source install/setup.bash
ros2 launch d435_bringup d435.launch.py
```

压缩包路径请按实际下载位置修改。如果文件缺失或安装依赖不全：

```bash
sudo apt install ros-humble-realsense2-camera \
  ros-humble-diagnostic-updater python3-yaml
```

也可以不编译这个启动包，直接用 launch 文件的绝对路径启动：

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/realsense_ws/src/d435_bringup/launch/d435.launch.py
```

启动前停止已经打开同一台相机的驱动或查看器。

## 修改配置

编辑 `config/d435.yaml` 后重新启动。运行时可以显式选择另一个配置：

```bash
ros2 launch d435_bringup d435.launch.py \
  config_file:=$HOME/realsense_ws/src/d435_bringup/config/d435.yaml
```

使用上面显式源文件路径的命令，可以确定编辑的就是本次读取的配置；
启动日志也会打印实际配置路径。普通安装复制了配置文件时，应重新构建，
或者直接通过 `config_file` 指向源文件。

配置使用平铺 YAML 键名，用注释分类。例如：

```yaml
camera_name: "camera"
camera_namespace: "camera"
log_level: "info"
output: "screen"
enable_color: false
enable_depth: true
enable_infra1: true
enable_infra2: true
depth_module.depth_profile: "640x480x30"
depth_module.infra_profile: "640x480x30"
```

`camera_namespace`、`log_level`、`output` 用于配置 launch 的 Node 动作；
`camera_name` 同时用作节点名和驱动参数，以保持话题与 TF 命名一致。
其余配置作为 ROS 参数直接传给 `realsense2_camera_node`。
串行号和图像规格用引号写成字符串；布尔值写 `true` / `false`；
例如 `tf_publish_rate` 这类浮点参数写 `0.0`，不要写 `0`。

这个 launch 直接启动节点，没有再包含官方 `rs_launch.py`，因此不会有
官方 launch 的默认参数覆盖本文件的情况。设备和驱动中没有显式设置的
参数仍使用它们自己的默认值。

## 导出本机完整参数

相机型号、固件和驱动版本决定可用参数。提供的 YAML 覆盖 D435 常用
设置，不声称列出了所有 SDK 自动生成的设备参数。要取得你的本机完整
运行参数，在相机节点正常运行时执行：

```bash
ros2 param dump /camera/camera > ~/realsense_ws/src/d435_bringup/config/d435_current.yaml
```

停止旧节点后，可以加载该快照：

```bash
ros2 launch d435_bringup d435.launch.py \
  config_file:=$HOME/realsense_ws/src/d435_bringup/config/d435_current.yaml
```

launch 支持单个节点的 `ros__parameters` 格式，以及 `/**` 通配节点格式。
不支持把多个节点的参数放在同一份导出文件中。普通参数快照不会保存
日志等级、输出位置等 launch 选项；加载快照时这两项默认是 info/screen。
更换固件、驱动版本或相机型号后，请重新检查快照中的设备特定字段，
不要假定旧快照的每个选项都可继续使用。建议把确认需要控制的设置
整理回有注释的 `d435.yaml` 中。

## 检查配置是否生效

```bash
ros2 node list
ros2 param get /camera/camera enable_color
ros2 param get /camera/camera depth_module.depth_profile
ros2 param get /camera/camera depth_module.infra_profile
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 param describe /camera/camera depth_module.exposure
```

不支持的分辨率和帧率可能回退到其他规格，应同时核对启动日志和实际
图像尺寸、发布频率。不要只通过参数文件判断真实采集规格。

## 用于 VINS 和避障

- D435 没有 IMU，外部 PX4 IMU 的时间同步和外参标定需要另行完成。
- `enable_sync` 只涉及相机流之间配对。
- 做 VINS 时可比较 `depth_module.emitter_enabled: 0` 的效果；
  关闭散斑可能降低缺少自然纹理区域的深度质量。
- 自动曝光开启时，手动曝光、增益配置不保证是实际采集值；
  要使用手动值，先把 `depth_module.enable_auto_exposure` 改为 false。
- 如需点云，可开启 `pointcloud.enable`；本文件已经设置为不要求 RGB 纹理。
- 深度对齐到彩色图时，需要先开启彩色流；当前配置保持关闭。
- 修改 profile 前先查询本机支持规格，不保证任意分辨率与帧率组合都可用。

## 已完成的检查与限制

交付前已检查 Python 语法、YAML 解析和参数值类型、package.xml，以及
压缩包内容。当前制作环境没有 ROS 2 和实体 D435，未在此处进行 colcon
构建或硬件取图验证；本机启动结果仍需以你的相机和已安装驱动为准。

官方参考：

- https://github.com/realsenseai/realsense-ros
- https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py
