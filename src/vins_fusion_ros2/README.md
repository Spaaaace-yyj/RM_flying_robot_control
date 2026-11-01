# vins_fusion_ros2

This repository is a ROS 2 adaptation of the original [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) project. It supports visual-inertial odometry for both monocular and stereo camera setups, using the IMU when enabled. This package contains the VIO estimator, not the upstream global-fusion or loop-closure nodes.

## Features

- Ported to ROS 2 Foxy on this ubuntu20.04 branch (the main branch targets Humble)
- Compatible with stereo or monocular cameras
- Integration with IMU
- Supports tightly coupled sensor fusion
- Output of odometry and pose 

## Prerequisites
- **System**
  - Ubuntu 20.04
  - ROS 2 Foxy
- **Libraries**
  - OpenCV 4.2 (system CPU build; use_gpu and use_gpu_acc_flow must be 0)
  - Ceres Solver 1.14 and Eigen 3.3.7 from Ubuntu 20.04 packages
  - glog

### Install Dependencies
```bash
sudo apt-get install libgoogle-glog-dev libeigen3-dev libceres-dev libopencv-dev libboost-filesystem-dev 
```
## Build Instructions

This directory is already included in the outer repository; do not clone another copy into it.
See the [outer README](../../README.md) for full Foxy dependency and validation instructions.

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select vins_fusion_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
# Real sensors: replace this path with your calibrated OpenCV YAML
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py config_file:=/absolute/path/config.yaml use_sim_time:=false
# EuRoC example and ROS 2 bag replay
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py use_sim_time:=true
ros2 bag play /path/to/ros2_bag --clock
```

The launch defaults to wall time. The EuRoC example is not a calibration for your own cameras.
This Foxy adaptation preserves the original GPLv3 license and source attributions.

## Converting ROS 1 Bag Files to ROS 2 Format

### ROS 2 cannot play ROS 1 `.bag` files directly

Unfortunately, you **cannot directly play** a `.bag` file recorded in **ROS 1** using **ROS 2** tools like `ros2 bag play`.

This is because:

- ROS 1 `.bag` files are stored as a **single binary file**.
- ROS 2 `.bag` files use a **folder-based structure** containing:
  - a `.db3` SQLite3 database,
  - metadata YAML files.

---

### Convert ROS 1 `.bag` to ROS 2 `.db3`

You can convert ROS 1 `.bag` files to ROS 2 `.db3` format using [`rosbags`](https://pypi.org/project/rosbags/).

#### 1. Install `rosbags`

```bash
pip install rosbags
```

If you're using `~/.local/bin` (default for user-level installs), you may need to add it to your shell PATH:

```bash
export PATH=$PATH:~/.local/bin
```

You may also consider putting the above line in your `.bashrc` or `.zshrc`.

---

#### 2. Convert the `.bag` file

```bash
rosbags-convert --src foo.bag --dst /path/to/output_folder
```

- `foo.bag`: your original ROS 1 bag file
- `/path/to/output_folder`: target folder where converted ROS 2 `.db3` bag will be saved

---

#### 3. Play the converted bag file

Once conversion is complete, you can play it back using ROS 2 tools:

```bash
ros2 bag play /path/to/output_folder
```

---


## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.