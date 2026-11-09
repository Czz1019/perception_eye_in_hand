# nvBlox库来源、与workspaces的关系及空白ROS2环境部署

核对日期：2026-11-09。本轮读取当前安装包元数据、源码及构建配置；没有重新安装依赖或运行仿真。

## 1. nvblox_torch从哪里来，是什么版本

本机实际读取结果：

| 项目 | 实际值 |
| --- | --- |
| 上游仓库 | https://github.com/nvidia-isaac/nvblox |
| 发布入口 | GitHub release v0.0.10 |
| 安装文件 | nvblox_torch-0.0.10+cu12ubuntu22-py3-none-linux_x86_64.whl |
| 已安装包内部版本 | 0.0.10.dev1+cu12ubuntu22 |
| 源码提交 | e4a2b00f45ecf989682d72a33a2c29defe39fd2d |
| 本机模块目录 | /home/czz/.local/lib/python3.10/site-packages/nvblox_torch |
| 配套Torch | 本机已验证2.9.1+cu128 |

安装来源由该包的direct_url.json记录确认，不是从原workspaces的install里导入Python模块。文件名和内部版本不同是当前发布物的元数据事实，不能把它解释成安装了两个不同的nvBlox包。

[官方安装页面](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/installation.html)给出了Ubuntu22/CUDA12的Python wheel入口。当前使用的正是该核心仓库内置的Python/Torch接口，不是另一个同名第三方项目。

## 2. 它还是ROS版本吗

**nvblox_torch本身不是ROS功能包；ur7e_nvblox_in_hand整体是ROS2应用。** 两个层次需要区分：

```text
原nvblox_in_hand：
ROS输入 → 官方nvblox_ros/NvbloxNode → 对应C++/CUDA核心

新ur7e_nvblox_in_hand：
ROS输入 → 本工程rclpy节点 → nvblox_torch → wheel内C++/CUDA核心
```

两条路线都使用nvBlox技术和CUDA计算，但ROS适配层、核心版本与二进制来源不同。新工程使用ROS发布订阅，不代表又调用了旧nvblox_ros节点。Torch是张量与扩展接口框架；这里没有因此启用神经网络识别模型。

## 3. 借用了官方哪一部分，本工程新增了什么

| 内容 | 提供方 |
| --- | --- |
| 稀疏体素地图、TSDF深度融合、颜色融合、ESDF更新、Mesh生成 | 官方nvBlox C++/CUDA核心，来自wheel内libnvblox_lib.so |
| Mapper、MapperParams、Sensor、深度/颜色帧接口、Mesh/查询接口 | 官方nvblox_torch与libpy_nvblox.so |
| 图像与CameraInfo订阅、图像时刻机械臂TF、有限队列、调度、ROS话题与服务 | 本工程node.py、backend.py、messages.py |
| 原生observed ESDF与地图层导出、2D调用、半径清理接入 | 本工程native桥接，访问同一个官方Mapper |
| UR7e/D435仿真场景、挂载和模型 | 从之前工程复制后保留在本工程，不运行时引用兄弟目录 |
| 匹配的nvBlox/Torch绑定头文件 | 官方固定提交，随vendor保留 |
| stdgpu编译头文件 | 原机器workspaces安装中的头文件副本，现已放进vendor；不是运行库路径依赖 |

没有重新实现一套TSDF/ESDF算法；本工程主要实现ROS接入、时序与输出适配。原workspaces仍是场景、组织方式及历史对比的参考来源，这不等于当前建图任务由它的节点完成。

## 4. 建图现在到底由谁完成

现在由**本工程进程内加载的官方wheel C++/CUDA Mapper**完成，而非workspaces中的ROS节点。

源码证据：

- `backend.py`定位已安装的nvblox_torch，加载其 `lib/nvblox/libnvblox_lib.so`，创建 `Mapper(...)`，调用 `add_depth_frame`、`add_color_frame`、`update_esdf`。
- `native/CMakeLists.txt`链接wheel内 `libnvblox_lib.so`、`libpy_nvblox.so`和Torch库，不链接workspaces中的libnvblox_ros_lib.so。
- `env.sh`清理旧overlay，只加载系统ROS和本工程install，不source原workspaces。
- 运行时 `/ur7e_nvblox/metrics`的 `nvblox_core_library`、`loaded_libraries`可检查真实加载路径。

所以目标机器无需创建 `/home/czz/workspaces` 或安装完整isaac_ros-dev。它需要重新下载官方wheel及配套依赖。这里的“独立”指不依赖原机器的工作区，而不是不依赖第三方库。

## 5. 新机器只有ROS2时怎样配置

以下前提为Ubuntu22.04 x86_64、ROS2 Humble、系统Python3.10。其他ROS发行版或ARM平台不能照搬当前wheel和脚本。

| 顺序 | 安装或准备 | 为什么需要 |
| --- | --- | --- |
| 1 | 兼容NVIDIA驱动，确认nvidia-smi可用 | GPU设备访问 |
| 2 | CUDA Toolkit12.6，确认nvcc | 编译本地桥接；只有Torch运行库不够 |
| 3 | ROS扩展依赖：CycloneDDS、消息/接口生成、TF、RViz、Gazebo11、控制器等 | 现有Humble基础安装未必包含这些 |
| 4 | Eigen、glog、FCL、编译工具等 | 本地C++/CUDA及场景构建 |
| 5 | 固定Torch/torchvision、NumPy等Python依赖 | 张量、图像、位姿及wheel依赖 |
| 6 | 下载Ubuntu22/CUDA12的官方nvBlox wheel并检查SHA/hash | 保证与vendor头文件兼容 |
| 7 | 复制本工程完整目录，在目标机build | 产生本机ABI、路径和GPU架构对应产物 |
| 8 | GPU契约检查，再启动场景/Mapper/RViz | 验证实际计算和输入输出 |

完整可复制的安装命令按以下两个文档顺序执行，不只运行build.sh：

1. [系统、驱动、CUDA、ROS扩展和可选D435安装](../../docs/deployment/SYSTEM_SETUP_UBUNTU2204.md)。ROS Humble已具备时跳过其中ROS基础仓库/安装步骤，继续其余依赖。
2. [本工程Torch/wheel固定版本安装、hash校验、编译及完整仿真](MIGRATION_DEPLOYMENT_20261109.md)。

准备完依赖后，目标机编译验收入口为：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export UR7E_CUDA_ARCH=89  # 本机4060；其他GPU按实际计算能力修改
./build.sh
source ./env.sh
/usr/bin/python3 tools/check_wheel.py
/usr/bin/python3 tools/check_backend.py
./run.sh gazebo_gui:=true rviz_mode:=diagnostic
```

若真正是一台没有任何ROS依赖的空白Ubuntu机器，先完整执行公共文档中的ROS Humble安装步骤，再继续；安装ROS基础环境不等于安装了本项目所有依赖。

## 6. 迁移时最容易混淆的地方

- 不需要安装旧nvblox_ros/NITROS/GXF/VPI/cuVSLAM；需要的是普通ROS2环境加Torch/wheel与本地桥接。
- 不要拷贝旧build/install/install_native代替构建，它们含原路径和ABI信息。
- 当前脚本显式使用 `/usr/bin/python3`；只激活venv或Conda不能切换这些调用。
- native桥接固定wheel源码SHA。升级wheel时要同时核对头文件/API并重新验证，不能跳过版本检查。
- 新机做Gazebo仿真不需要真实D435 SDK；实机才需RealSense ROS驱动/SDK，以及真实机械臂驱动与手眼TF。
- 即使实机不启动Gazebo，当前完整build仍包含场景包，需要其构建依赖；本轮没有拆成纯实机最小构建方案。
- 默认本机DDS配置与ROS_LOCALHOST_ONLY=1不适合直接跨主机通信，需要核对DDS网卡、域与话题QoS。
- 本机仿真性能已有记录，但干净机器迁移尚未实测，不能保证仅安装成功便达到相同频率或重建质量。

更详细的模块与算法解释见[功能包、依赖与核心算法文档](PACKAGES_DEPENDENCIES_CORE_ALGORITHMS_20261109.md)。
