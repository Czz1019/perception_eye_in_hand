# nvblox_in_hand：原 ROS C++ 工程迁移与部署

核对日期：2026-11-09。本文件对应原 C++ 工程，不是同级 Python 接口工程。

## 1. 当前到底依赖什么

当前运行链为：D435 图像/CameraInfo → 图像时刻的机械臂相机 TF → 官方 ROS `nvblox::NvbloxNode` → nvBlox C++/CUDA Mapper → TSDF / Color / ESDF / Mesh；本工程的 diagnostic mapper 继承官方节点，增加原生 observed ESDF 点云与统计。

源码依据：`src/nvblox_in_hand/CMakeLists.txt`、`src/nvblox_in_hand/src/mapper_node.cpp`、`upstream/nvblox_ros/{CMakeLists.txt,package.xml}`、`env.sh`、`build.sh`。核心链接是 `nvblox_ros::nvblox_ros_lib` 和 `CUDA::cudart`，不是 nvblox_torch。

本机 ldd 已核实：

| 库 / 包 | 本机来源 | 用途及迁移处理 |
| --- | --- | --- |
| libnvblox_ros_lib.so | workspaces/isaac_ros-dev/install/nvblox_ros/lib | ROS 节点、订阅、调度、转换；需目标机提供 |
| libnvblox_lib.so / libnvblox_datasets.so | 同上 | TSDF/Color/ESDF/Mesh 核心与数据支持；随该源码构建 |
| nvblox_msgs、nvblox_ros_common、nvblox_ros_python_utils | 同一 underlay | 消息、CUDA/ROS 工具；一起构建 |
| nvblox_rviz_plugin | 同一 underlay | 官方分块 Mesh / voxel 消息显示 |
| isaac_ros_common 3.2.5 | 本机 workspaces 的 install | 公共 ROS、QoS、VPI 工具；可在新依赖工作区重建 |
| managed_nitros / nitros / image_type / camera_info_type 3.2.5 | /opt/ros/humble | 图像、CameraInfo 加速类型和节点桥接；当前源码需要 |
| GXF / gxf_isaac_* | /opt/ros/humble/lib 和 share 下 | NITROS 的间接运行依赖，apt 解析安装 |
| libnegotiated.so 与 interfaces | **本机 ros_pack/install** | NITROS 协商；必须处理，不能遗漏这一额外 underlay |
| libnvvpi.so.3 | /opt/nvidia/vpi3 | 当前 isaac_ros_common 实际链接；本机 3.1.5 |
| libcudart.so.12 / libnpp*.so.12 | /usr/local/cuda，当前指向 12.6 | CUDA 运行时/图像计算；安装 Toolkit |
| Eigen / glog / gflags / SQLite / stdgpu | 系统包或核心 CMake 下载 | 编译、日志、存储、GPU 哈希依赖 |

完整原始链接与版本见[本机审计记录](../../docs/deployment/DEPENDENCY_AUDIT_20261109.json)。本机还装了识别人像等其他 Isaac 包，它们不是本工程的必选依赖。

因此，“用了 workspaces 的库”是当前部署事实，但“必须搬整个 workspaces”不是必要条件。目标机可以建立 `~/nvblox_ros_deps`，只放必需源码，生成自己的 install，再设置 `NVBLOX_ROS_WS` 指向它。

## 2. 哪些不能直接替换

官方 ROS 3.2 的[安装入口](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)提供 Debian 和源码两种方式；源码使用 `release-3.2`。当前本机相关包主要是 3.2.5，核心快照的 CMake 版本为 0.0.7。**不要直接下载最新 nvBlox core 或最新 Isaac ROS 替换这一组合。** 官方最新分支与当前 Humble 移植、诊断节点访问接口、RViz 兼容处理不保证一致。

优先采用本工程已保留的 `upstream/` 源码快照。它包含当前 ROS 实现、配套 core、消息与插件。迁移时一同保留 provenance 与许可证。源码来源参考 release-3.2，但用重新 clone 的分支直接覆盖本地快照，不能复现本地修改和已验证行为。

`build.sh --from-source` **不能独立完成首次部署**：脚本进入编译分支之前已经 source env.sh，后者要求 `${NVBLOX_ROS_WS}/install/setup.bash` 和 underlay 的 nvblox_node 存在。必须先按下文构建依赖 underlay，再运行工程 build.sh。

## 3. 原机器准备交付物

建议交付完整 `nvblox_in_hand` 源码目录和一个小型额外源码目录，不复制旧 install/build。以下只打包必需的 common、negotiated 源码，不打包识别模型、SLAM、MoveIt 或其他传感器示例：

```bash
cd /home/czz/perception_eye_in_hand
mkdir -p /tmp/nvblox_migration_sources/extra_src
rsync -a --exclude=.git \
  /home/czz/workspaces/isaac_ros-dev/src/isaac_ros_common/isaac_ros_common \
  /tmp/nvblox_migration_sources/extra_src/
rsync -a --exclude=.git \
  /home/czz/ros_pack/src/negotiated/negotiated \
  /home/czz/ros_pack/src/negotiated/negotiated_interfaces \
  /tmp/nvblox_migration_sources/extra_src/
tar -czf /tmp/nvblox_extra_sources.tar.gz -C /tmp/nvblox_migration_sources extra_src

tar -czf /tmp/nvblox_in_hand_source.tar.gz \
  --exclude='nvblox_in_hand/build*' --exclude='nvblox_in_hand/install*' \
  --exclude='nvblox_in_hand/log' --exclude='nvblox_in_hand/.ros' \
  --exclude='nvblox_in_hand/artifacts' --exclude='*/__pycache__' \
  nvblox_in_hand docs/deployment
sha256sum /tmp/nvblox_in_hand_source.tar.gz /tmp/nvblox_extra_sources.tar.gz
```

这段是供操作者执行的打包指令，本轮没有替你生成这些归档。交付 common/negotiated 时一并保留其包内版权声明及仓库根 LICENSE/NOTICE（若包目录外存在，另行加入归档）。若仅内部部署，可从原机器复制这些源码；向外分发还应遵守各组件许可，不能把整套组件都声称为本工程原创。

如果不便使用该额外源码包，可按官方 release-3.2 获取 isaac_ros_common、通过 apt 获取 negotiated；但这属于替代部署，需重新核对版本和测试，不应称为精确复现本机路径。

## 4. 目标机安装系统、CUDA 与 ROS

先完成[公共安装步骤](../../docs/deployment/SYSTEM_SETUP_UBUNTU2204.md)。以下继续针对 Ubuntu 22.04 amd64 / ROS Humble / Toolkit 12.6。

### 4.1 添加 Isaac ROS 3 系列 apt 源

依据 [Isaac ROS 3.2 官方 apt 源说明](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/isaac_apt_repository.html)，此处使用 signed-by keyring 写法。已有相同源时检查并复用，不重复创建不同签名配置：

```bash
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | gpg --dearmor > /tmp/ur7e-isaac-ros-keyring.gpg
sudo install -m 644 /tmp/ur7e-isaac-ros-keyring.gpg \
  /usr/share/keyrings/ur7e-isaac-ros.gpg
printf '%s\n' 'deb [signed-by=/usr/share/keyrings/ur7e-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0' \
  | sudo tee /etc/apt/sources.list.d/ur7e-isaac-ros.list
sudo apt-get update
apt-cache policy ros-humble-isaac-ros-gxf ros-humble-isaac-ros-nitros
```

以下版本是本机实际安装值。先确认仓库可提供；不存在时停止此步骤，使用存档包或配套 release-3.2 环境，不要静默升级所有 Isaac 包：

```bash
sudo apt-get install -y \
  ros-humble-isaac-ros-common=3.2.5-0jammy \
  ros-humble-isaac-ros-gxf=3.2.5-0jammy \
  ros-humble-isaac-ros-nitros=3.2.5-0jammy \
  ros-humble-isaac-ros-managed-nitros=3.2.5-0jammy \
  ros-humble-isaac-ros-nitros-image-type=3.2.5-0jammy \
  ros-humble-isaac-ros-nitros-camera-info-type=3.2.5-0jammy \
  ros-humble-negotiated=0.1.0-0jammy \
  ros-humble-negotiated-interfaces=0.1.0-0jammy
sudo apt-get install -y libgtest-dev libbenchmark-dev \
  ros-humble-ament-cmake-auto ros-humble-rosbag2-cpp \
  ros-humble-tf2-eigen ros-humble-message-filters
```

其余 GXF 组件由依赖关系安装；用后续 rosdep/ldd 验证。安装系统 negotiated 后再在 underlay 编译所交付源码，会由新 underlay 覆盖系统包；保留其相同版本，避免偷偷引用原机器 ros_pack。

### 4.2 安装 VPI 3 开发库

本机 apt 源实际是 NVIDIA 的 x86_64/jammy r36.3，安装了 libnvvpi3 / vpi3-dev 3.1.5。官方对应仓库和包名可参见 [JetPack 6 的 x86 主机安装说明](https://docs.nvidia.com/jetson/jetpack/6.0/install-setup/index.html)。此处只安装 VPI 包，不安装 Jetson 交叉编译整套工具。

```bash
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
  | gpg --dearmor > /tmp/ur7e-vpi-keyring.gpg
sudo install -m 644 /tmp/ur7e-vpi-keyring.gpg /usr/share/keyrings/ur7e-vpi.gpg
printf '%s\n' 'deb [signed-by=/usr/share/keyrings/ur7e-vpi.gpg] https://repo.download.nvidia.com/jetson/x86_64/jammy r36.3 main' \
  | sudo tee /etc/apt/sources.list.d/ur7e-vpi.list
sudo apt-get update
apt-cache policy libnvvpi3 vpi3-dev
sudo apt-get install -y libnvvpi3=3.1.5 vpi3-dev=3.1.5
```

仅拿到头文件不够；isaac_ros_common 的 CMake 有 `find_package(vpi REQUIRED)`，运行时也要解析 libnvvpi.so.3。

## 5. 构建新的依赖工作区，脱离原路径

将两个归档放入目标机 `/tmp`。下列路径以目标用户 `$HOME` 为准：

```bash
mkdir -p "$HOME/perception_eye_in_hand" "$HOME/nvblox_ros_deps/src"
tar -xzf /tmp/nvblox_in_hand_source.tar.gz -C "$HOME/perception_eye_in_hand"
tar -xzf /tmp/nvblox_extra_sources.tar.gz -C /tmp
cp -a /tmp/extra_src/. "$HOME/nvblox_ros_deps/src/"
for pkg in nvblox_ros nvblox_ros_common nvblox_msgs nvblox_rviz_plugin nvblox_ros_python_utils; do
  cp -a "$HOME/perception_eye_in_hand/nvblox_in_hand/upstream/$pkg" \
    "$HOME/nvblox_ros_deps/src/"
done
```

只复制五个包目录，不复制 upstream 根的 COLCON_IGNORE。保留 nvblox_ros/nvblox_core 整体结构。

在新终端打开干净 shell，避免 `.bashrc` 自动 source 原工作区；该命令只创建子 shell，后续命令在该子 shell 执行：

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM="${TERM:-xterm}" LANG=en_US.UTF-8 bash --noprofile --norc
source /opt/ros/humble/setup.bash
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd "$HOME/nvblox_ros_deps"
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon list --base-paths src
colcon build --base-paths src --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_CUDA_ARCHITECTURES=89
```

RTX 4060 的 89 需按目标卡修改。内存不足时先执行 `export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2`。rosdep 主要处理 ROS 包清单，不会替代 CUDA/VPI 安装；若有无法解析的 key，应定位对应库并安装，再重试，不能把 rosdep 报错当作安装成功。

核心构建还可能联网：stdgpu commit `71a5aef26626eda47d15e5f577ca3b1538ff996a`，SQLite amalgamation 3500400；Eigen 3.4.0 取决于 USE_SYSTEM_EIGEN。glog/gflags/gtest/benchmark 当前 ROS CMake 选择系统包。详见 `upstream/nvblox_ros/nvblox_core/nvblox/thirdparty/` 的 cmake。stdgpu 有项目携带的 CUDA12.6/哈希补丁，不能随意用无补丁系统版替换。

此步骤为首次 underlay 构建流程，本轮未在全新机器执行。完成后先核查，再编译应用：

```bash
source "$HOME/nvblox_ros_deps/install/setup.bash"
ros2 pkg prefix nvblox_ros
ros2 pkg prefix isaac_ros_common
ros2 pkg prefix negotiated
ldd "$HOME/nvblox_ros_deps/install/nvblox_ros/lib/nvblox_ros/nvblox_node"
```

必须没有 `not found`，也不能出现旧机器 `/home/czz/workspaces` 或 `/home/czz/ros_pack` 路径。GXF 若在 share 子目录解析失败，需要核对其环境 hook/同版本安装路径，不能从原机器复制几个 .so 来掩盖依赖错误。

## 6. 编译本工程与运行

```bash
export NVBLOX_ROS_WS="$HOME/nvblox_ros_deps"
export IN_HAND_CUDA_ARCH=89
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
cd "$HOME/perception_eye_in_hand/nvblox_in_hand"
./build.sh
source ./env.sh
ros2 pkg prefix nvblox_in_hand
./run.sh --show-args
```

默认 `build.sh` 即可，底层已经由新 underlay 提供。需要再次覆盖 upstream 源码时才用 `./build.sh --from-source`，并在运行时设置 `IN_HAND_SOURCE_BUILD=1`；这不是本次迁移必做的第二次编译。

快速检查 Gazebo 和 3D ESDF（不自动动机械臂）：

```bash
export NVBLOX_ROS_WS="$HOME/nvblox_ros_deps"
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
cd "$HOME/perception_eye_in_hand/nvblox_in_hand"
./run_esdf.sh gazebo_gui:=true camera_width:=640 camera_height:=480 voxel_size:=0.03
```

另一终端：

```bash
export NVBLOX_ROS_WS="$HOME/nvblox_ros_deps"
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
cd "$HOME/perception_eye_in_hand/nvblox_in_hand"
source ./env.sh
ros2 topic list -t
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

Ctrl+C 后再查询统计，实际话题名称以本工程 `PARAMETERS_AND_TOPICS.md` 和 topic list 为准。高位静止、轻微左右扫描、Mesh/3D ESDF 截图完整操作继续参考[已有仿真说明](BUILD_MESH_ESDF_SIMULATION_TEST_20261109.md)与[日常操作说明](OPERATIONS.md)。这些命令不用于实机机械臂运动。

## 7. 部署验收与常见错误

| 现象 | 原因 / 处理 |
| --- | --- |
| Missing official dependency environment | NVBLOX_ROS_WS 未设置，或 underlay 没编完 |
| --from-source 仍找 workspaces | env.sh 先检查 underlay；按第5节 bootstrap，再 build |
| libnegotiated.so 找不到或指向 ros_pack | 新 underlay 没提供相应包，或旧环境/RPATH 污染 |
| VPI CMake / libnvvpi.so.3 失败 | 同时核对开发包、运行库及版本，不只安装 GXF |
| RViz 缺 box.geom / scripts150 后退出 | 当前快照插件支持 fallback；env.sh 自动检测，或设置 IN_HAND_RVIZ_MATERIAL_MODE=fallback；不混用未经验证的新插件 |
| undefined reference 中有 thrust/CUDA_ARCH_LIST | 头文件、Toolkit、库 ABI/编译方式不一致；目标机整套重编，不能由此直接断定驱动错误 |
| Gazebo master already in use | 查看 `ss -ltnp 'sport = :11457'`，确认进程身份；不要重复启动场景 |
| 只有 /rosout 与 /parameter_events | Mapper/场景未启动或 DDS 域/RMW不一致；先查启动日志，不能靠 topic hz 修复 |
| 数据正常但性能不同 | 核对分辨率、voxel_size、工作范围、输入频率、观察路径、订阅者负载、独显状态和电源模式 |

验收必须分别检查输入频率、成功融合计数、ESDF 更新计数、点云发布与新输入关联、图像到输出年龄、Mesh 和实际 RViz 图像。配置写 30/33.4/40Hz 不等于目标机器实测一定达到该值。

官方建议的 Dev Docker 是另一条部署路线，见前述 release-3.2 quickstart；它可提供较一致依赖环境，但仍需把本地源码快照和场景编译进去。本文没有把“最新官方 Docker 一键启动”当作本工程已验证的可迁移镜像。
