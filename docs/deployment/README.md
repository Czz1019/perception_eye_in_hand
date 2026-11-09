# 感知工程迁移与部署说明

核对日期：2026-11-09。本文中的“原工程”指 `nvblox_in_hand`；“Python 工程”指同级 `ur7e_nvblox_in_hand`。两者都是独立 colcon 工作区，不能把两套同名 `ur7e_d435_gazebo` 包一起放进同一次递归编译。

## 1. 先明确迁移对象

| 项目 | nvblox_in_hand：ROS C++ 版 | ur7e_nvblox_in_hand：Python 接口版 |
| --- | --- | --- |
| ROS 接入、调度 | 官方 `nvblox::NvbloxNode`，本工程继承适配 | 本工程 rclpy 节点、GPU 单所有者工作线程 |
| 核心计算 | ROS underlay 中的 nvBlox C++/CUDA | 官方 wheel 内的 nvBlox C++/CUDA |
| 本工程需编译 | ESDF CUDA 导出、mapper、相机插件、场景 | Torch C++/CUDA 桥接、ROS 接口、相机插件、场景 |
| NITROS / GXF / negotiated | 当前实现需要，包含间接依赖 | 当前实现不需要 |
| VPI | 当前 isaac_ros_common 链接需要 | 当前实现不需要 |
| PyTorch / nvblox_torch | 不需要 | 需要，且有版本检查 |
| ROS nvBlox RViz 插件 | 需要，配合 nvblox_msgs 分块 Mesh | 不需要，使用标准 MarkerArray/PointCloud2 |
| 现有 workspaces | 默认 env.sh 仍引用；可重建为别处的依赖工作区 | 不加载、不链接其二进制 |
| Gazebo 仿真 | 两者都需要 ROS Humble、Gazebo Classic、控制器及场景资产 | 同左 |
| 真 D435 | 额外安装 RealSense ROS 驱动/SDK | 同左 |
| 相机 pose | 机械臂 TF + 手眼外参；不需要 cuVSLAM | 同左 |

**不必把 workspaces 的所有功能部署一遍，但 C++ 版必须提供完整且版本一致的 ROS nvBlox 依赖环境。Python 版减少的是 ROS 加速框架依赖，不是用 Python 替代 CUDA 计算。** 官方 Python 安装入口本身也是 wheel；对应安装文档见[官方 nvBlox v0.0.10 安装页](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/installation.html)。

完整步骤分为：

1. [公共系统、驱动、CUDA 与 ROS 安装](SYSTEM_SETUP_UBUNTU2204.md)。
2. [原 C++ 工程迁移：依赖清单、源代码打包、独立依赖工作区、运行和排错](../../nvblox_in_hand/docs/MIGRATION_DEPLOYMENT_20261109.md)。
3. [Python 工程迁移：wheel、Torch、桥接、仿真与实机入口](../../ur7e_nvblox_in_hand/docs/MIGRATION_DEPLOYMENT_20261109.md)。
4. [本机依赖审计原始记录](DEPENDENCY_AUDIT_20261109.json)：包含实际 ldd 路径、已安装版本和 wheel 来源/hash。记录中出现的 DetectNet/人物模型等只是本机其他任务安装过，**不代表本功能需要安装**。

## 2. 本机已验证组合与迁移边界

| 层次 | 本机记录 | 迁移时处理 |
| --- | --- | --- |
| OS / CPU | Ubuntu 22.04.5，x86_64 | 本文命令仅针对 Ubuntu 22.04 amd64 |
| ROS / Python | Humble / 系统 Python 3.10 | 使用系统 `/usr/bin/python3`，不要混 Conda |
| GPU | RTX 4060 Laptop，8GB，CC 8.9 | 本地 CUDA 桥接/导出默认架构 89，换卡须修改构建变量 |
| NVIDIA driver | 580.178.04 | 驱动由目标机器安装，不复制本机驱动文件 |
| CUDA Toolkit | nvcc 12.6.20，`/usr/local/cuda-12.6` | 安装 12.6 系列开发工具并在目标机重编 |
| C++ ROS 栈 | nvblox_ros / Isaac 相关主要包 3.2.5，core CMake 0.0.7 | 保留现有 upstream 快照，勿换最新分支当成等价 |
| VPI | libnvvpi3 / vpi3-dev 3.1.5 | C++ 版需要此依赖链；Python 版不用 |
| Python Torch | 2.9.1+cu128 / torchvision 0.24.1 | 与系统 Toolkit 12.6 是两层不同依赖 |
| Python nvBlox | 元数据 `0.0.10.dev1+cu12ubuntu22` | 发布下载文件名写 0.0.10；以 hash/内部源码 SHA 检查为准 |

当前驱动能够运行现有 CUDA 12 系程序。`nvidia-smi` 的 “CUDA Version 13.0” 不表示 nvcc 是 13，也不表示需要把项目改成 CUDA 13；兼容关系参考 [NVIDIA 驱动兼容说明](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)。Isaac ROS 3.2 的[官方 x86 配置建议](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html)还包含 Ampere 或更新 GPU、CUDA 12.6+、驱动 560+ 与显存要求，不能只取 CUDA 12 系通用最低驱动数字来选择整套 Isaac 环境。

本轮做了源文件、包清单、构建脚本、wheel 元数据和动态链接的只读核查，新增部署文档和审计记录；**没有新装系统依赖、修改感知算法、启动仿真或完成干净机器安装验证**。此前本机仿真结果见两工程已有验证文档；本文的全新目标机命令属于按当前实现整理的部署流程，执行后必须进行相应验收。

## 3. 应交付哪些文件

交付源代码、config/launch/rviz、机器人 URDF/mesh/world、相机插件源码、vendor/upstream、许可证、provenance 与操作文档；Python 版务必包含 native 与 vendor，C++ 版务必包含 upstream。每套目录应完整交付。

不交付旧 build/install/log/.ros 代替构建；其中可能有原用户目录、旧 Python 包路径、CMake 缓存、RPATH、GPU 架构和另一工作区的 setup 链。更换机器、用户名、工程路径或 GPU 后重新编译。

实机额外交付机械臂驱动版本、关节名称和 URDF、手眼标定、深度/颜色内参与外参、TF frame 约定、图像话题映射、QoS 和时间同步配置。Gazebo 里的准确 pose 并不能代替实机标定。
