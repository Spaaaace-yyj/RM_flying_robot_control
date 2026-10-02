# Robomaster2027自主飞行无人机控制

自主飞行无人机上层飞行控制算法

## 编译

### 环境依赖

代码在Ubuntu22.04 + Ros2 Humble环境下开发和测试

**1.Ubuntu22.04**

**2.Ros2Humble**

参照小鱼的一键安装

```bash
#克隆
git clone https://github.com/Spaaaace-yyj/RM_flying_robot_control.git
git -c 'url.https://github.com/.insteadOf=git@github.com:' submodule update --init --recursive
#确保安装了Ros2Humble
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
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-auto \
  ros-humble-ament-index-cpp \
  ros-humble-ament-index-python \
  ros-humble-rclcpp \
  ros-humble-rclcpp-components \
  ros-humble-builtin-interfaces \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager \
  ros-humble-camera-calibration \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-ros-environment \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-ros2launch \
  ros-humble-rviz2 \
  ros-humble-rqt-image-view \
  ros-humble-ament-lint-auto \
  ros-humble-ament-lint-common
  
sudo rosdep init
rosdep update

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y
```

