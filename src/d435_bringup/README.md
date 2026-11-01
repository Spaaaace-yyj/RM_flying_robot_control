# D435 / D430 系列 YAML 启动包（Ubuntu 20.04 / ROS 2 Foxy）

从本仓库 `main` 分支的 `src/d435_bringup` 移植，继续使用官方
`realsense2_camera` 驱动及 `d435.launch.py` 入口。D430 模组需要配套可被 SDK
识别的处理板/USB设备；具体可用数据流以设备、固件和驱动为准。

## 安装和启动

在本仓库工作区根目录执行：

```bash
source /opt/ros/foxy/setup.bash
sudo apt update
sudo apt install ros-foxy-realsense2-camera ros-foxy-diagnostic-updater python3-yaml
colcon build --packages-select d435_bringup --cmake-clean-cache
source install/setup.bash
ros2 launch d435_bringup d435.launch.py
```

若已从源码安装驱动，source 对应工作区后直接构建/启动即可。构建配置兼容系统
CMake 3.16 与 CMake 4.4.3。启动前停止占用同一台相机的其他驱动或查看器。

## 默认设置与修改

默认保留 `main` 分支 **实际 YAML** 的设置（原先主分支 README 的默认值描述已过时）：

| 数据流/选项 | 默认 |
| --- | --- |
| 深度、左右红外 | 开启，848×480、90 FPS |
| RGB | 开启，848×480、60 FPS |
| 点云、深度对齐到 RGB | 开启 |
| 相机自带 gyro/accel | 关闭，普通 D435 无 IMU |
| 自动曝光、红外投射器 | 开启 |
| 节点名、命名空间 | `camera`、`camera` |

这些是请求值，不保证每种 D400 设备和 USB 连接都支持，应核对实际图像尺寸、频率与
驱动日志。做 VINS/标定时可在自己的配置中关闭 RGB、点云、深度对齐，降低图像帧率，
按需关闭红外投射器；本次移植未替你更改原来的采集规格。

```bash
ros2 launch d435_bringup d435.launch.py \
  config_file:=$PWD/src/d435_bringup/config/d435.yaml
```

显式指定源文件后，编辑 YAML 并重启即可；默认使用安装目录配置，普通复制安装后
修改源文件需要重新构建。日志会打印实际配置路径和使用的驱动参数接口。

## Foxy 驱动参数兼容

`driver_api` 默认 `auto`，读取已安装驱动 `rs_launch.py` 中的参数声明来选择接口，
不会执行官方 launch，也不会让其默认参数覆盖本文件。三种接口为：

| 接口 | 典型驱动 | 转换 |
| --- | --- | --- |
| `legacy` | ROS 2 3.x，例如 3.2.3 | `*_width/height/fps`、`stereo_module.*`、旧点云/filters 参数 |
| `profile` | 早期 4.x，例如 4.51.1 | `depth_module.profile`、`rgb_camera.profile` |
| `split` | 使用分开的 depth/infra profile 的较新驱动 | 直接使用主分支配置键 |

也可显式选择：

```bash
ros2 launch d435_bringup d435.launch.py driver_api:=legacy
```

早期 4.x 的深度和红外共用 `depth_module.profile`，启用流的规格必须相同，冲突时
launch 会报错。旧 3.x 的左右红外共用 `infra_qos`，不同值也会报错。旧驱动使用标准
Z16/Y8/RGB8 格式；无法转换的格式或新功能请求会报错，而非假装已经生效。
设备特定的 SDK 选项仍需以本机运行结果为准，转换器不替代驱动/固件的能力检查。

数字形式的串行号必须写成带引号的字符串，例如 `serial_no: "001234567890"`。
launch 将参数写入临时 ROS YAML 传给节点，保留数字字符串、空字符串、浮点和布尔类型，
不依赖部分 Foxy 安装中缺失的 `ParameterValue(value_type=str)`；正常退出时删除临时文件。

支持单节点 ROS 参数快照及 `/**` 格式，也支持其中的嵌套参数。加载参数快照时最好
显式选择它对应的 `driver_api`；跨驱动版本的完整快照不能保证可复用。

## 检查数据

```bash
ros2 node list
ros2 topic list
ros2 param dump /camera/camera
```

旧 3.x 常见图像话题为 `/camera/infra1/image_rect_raw`，较新驱动可能为
`/camera/camera/infra1/image_rect_raw`。以实际 `ros2 topic list` 为准，并将 VINS
配置中的左右图像话题与之对应。`enable_sync` 只处理相机内部流配对，不同步外部 PX4 IMU。

## 检查范围与许可

```bash
colcon test --packages-select d435_bringup --event-handlers console_direct+
colcon test-result --verbose
```

CI 在真实 Focal/Foxy 环境使用 CMake 3.16 / 4.4.3 构建，检查三种驱动接口的参数转换、
已安装驱动的接口识别、Foxy ROS 参数解析、launch 动作构造和退出时清理临时参数文件。
检查不打开实体相机，未验证 USB 取流、硬件支持的 profile 或设备侧曝光选项。

启动包沿用 [MIT 许可证](LICENSE)；官方驱动及其依赖保持各自许可。
参考：[官方 RealSense ROS 驱动](https://github.com/realsenseai/realsense-ros)、
[3.2.3 启动参数](https://github.com/realsenseai/realsense-ros/blob/3.2.3/realsense2_camera/launch/rs_launch.py)、
[4.51.1 启动参数](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/launch/rs_launch.py)。
