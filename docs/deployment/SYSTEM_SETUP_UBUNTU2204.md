# Ubuntu 22.04 x86_64 公共部署指令

适用于两套工程的目标机。下列命令是部署说明，本轮没有在当前主机执行安装或升级。已有对应环境可以跳过相应章节；不要为了部署而覆盖一个正常工作的驱动。其他 Ubuntu/ROS/ARM 平台需单独适配，不能直接套用。

## 1. 基础系统与驱动

```bash
uname -m
lsb_release -ds
sudo apt-get update
sudo apt-get install -y ca-certificates curl wget gnupg lsb-release \
  software-properties-common locales git git-lfs rsync unzip \
  build-essential cmake python3-dev python3-pip python3-venv pkg-config
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
nvidia-smi
```

新系统先完成 Ubuntu 系统更新并重启，再安装 ROS；对应注意事项见 [ROS Humble 官方安装文档源码](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/Ubuntu-Install-Debs.rst)。

如果 `nvidia-smi` 已正常且驱动兼容现有 CUDA，可以保留。新机器尚未安装驱动时，先确认 Ubuntu 为该显卡提供的推荐项：

```bash
sudo apt-get install -y ubuntu-drivers-common
ubuntu-drivers devices
```

然后通过 Ubuntu“软件和更新→附加驱动”安装推荐且满足所选 CUDA 要求的 NVIDIA 驱动；也可在确认推荐版本后运行 `sudo ubuntu-drivers install`，重启并再次检查 `nvidia-smi`。本机参考为 580.178.04，不要求其他机器具有完全相同补丁号。驱动涉及内核/Secure Boot，不能提供一个适合所有机器的硬编码包名。Isaac ROS 3.2 的[官方硬件说明](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html)给出了其驱动要求。

## 2. CUDA Toolkit 12.6

以下为已安装兼容驱动的 Ubuntu 22.04 amd64 目标机安装开发工具。使用 Toolkit 元包，不选包含驱动升级的 `cuda` 元包。来源：[CUDA 12.6 Linux 安装说明](https://docs.nvidia.com/cuda/archive/12.6.0/cuda-installation-guide-linux/index.html)。

```bash
wget -O /tmp/ur7e-cuda-keyring.deb \
  https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i /tmp/ur7e-cuda-keyring.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-6
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export PATH=/usr/local/cuda-12.6/bin:$PATH
/usr/local/cuda-12.6/bin/nvcc --version
```

每个构建终端保留 `CUDACXX`。`env.sh` 会清理 `LD_LIBRARY_PATH`，因此不能依赖 source 之前手工设置的此变量修复库路径。nvcc、Torch CUDA runtime 与驱动分别核查。

## 3. ROS 2 Humble

按 ROS 官方当前的 [ros2-apt-source 安装方式](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/_Apt-Repositories.rst)添加仓库；如已配置 ROS 仓库，无需重复：

```bash
UR7E_ROS_APT_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ur7e-ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${UR7E_ROS_APT_VERSION}/ros2-apt-source_${UR7E_ROS_APT_VERSION}.jammy_all.deb"
sudo dpkg -i /tmp/ur7e-ros2-apt-source.deb
sudo apt-get update
sudo apt-get install -y ros-humble-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

仅在 `/etc/ros/rosdep/sources.list.d/20-default.list` 不存在时初始化：

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

## 4. 两工程通用的场景、控制器和显示依赖

```bash
sudo apt-get install -y \
  libeigen3-dev libgoogle-glog-dev libgflags-dev libfcl-dev \
  libglib2.0-0 libgl1 python3-numpy python3-scipy python3-yaml \
  python3-opencv python3-trimesh \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 \
  ros-humble-tf2-ros-py ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-point-cloud-transport \
  ros-humble-camera-info-manager \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime
```

Gazebo 指本工程使用的 **Gazebo Classic 11**，不能只安装 Gazebo Harmonic/Fortress 的新 `gz sim` 来替代。仅实机运行可不启动 Gazebo，但当前包清单包含场景包，按现有完整构建流程仍会构建其插件。

性能测试截图工具还需要：

```bash
sudo apt-get install -y xdotool wmctrl imagemagick x11-utils libxcomposite1 python3-pil
```

桌面窗口捕获要求可用的 X11/XWayland DISPLAY；它是截图条件，不是 nvBlox 计算依赖。

## 5. 可选：真实 D435

只做 Gazebo 不安装本节。采用 ROS 仓库的 SDK 与 wrapper 为一条安装路线；不要同时再往 `/usr/local` 编译安装另一套 librealsense。来源：[RealSense 官方 ROS 安装说明](https://github.com/realsenseai/realsense-ros/tree/ros2-master#installation-on-ubuntu)。

```bash
sudo apt-get install -y 'ros-humble-librealsense2*' \
  ros-humble-realsense2-camera ros-humble-realsense2-description
source /opt/ros/humble/setup.bash
ros2 pkg prefix realsense2_camera
```

实际 wrapper/SDK 版本记录下来；根据目标设备 firmware 与所选发行版复核支持表。`pyrealsense2` 不是当前 ROS 图像订阅路径的必需品。机械臂驱动和手眼 TF 也需另行部署，不能用 Gazebo 控制命令操作实机。

## 6. 转到对应工程

- [C++ ROS 部署](../../nvblox_in_hand/docs/MIGRATION_DEPLOYMENT_20261109.md)：继续安装 Isaac/NITROS/GXF/VPI 并构建依赖 underlay。
- [Python 部署](../../ur7e_nvblox_in_hand/docs/MIGRATION_DEPLOYMENT_20261109.md)：继续安装 Torch/wheel 并构建本地桥接。

这两条路线选择其一即可，不需要为 Python 路线额外安装完整 Isaac ROS 栈。
