# ur7e_nvblox_in_hand：Python 接口版完整迁移与安装

核对日期：2026-11-09。本工程与 `nvblox_in_hand` 同级。当前代码无需原 workspaces 的运行库，但必须安装 ROS、驱动、CUDA Toolkit、Torch 和匹配的 nvBlox wheel，并编译本地桥接。

## 1. 需要下载哪些东西

| 必需内容 | 获取来源 | 实际作用 |
| --- | --- | --- |
| 本工程完整源码 | 交付 ur7e_nvblox_in_hand 整个目录 | 包含 ROS 节点、接口、相机插件、机械臂场景、配置与工具 |
| NVIDIA 驱动 | Ubuntu / NVIDIA 官方渠道 | GPU 设备驱动，由目标主机安装 |
| CUDA Toolkit 12.6 | NVIDIA CUDA apt 仓库 | nvcc、头文件与开发库；编译 native CUDA 桥接和插件 |
| ROS 2 Humble 和 Gazebo Classic 11 | ROS / Ubuntu apt | 图像、TF、控制器、场景、RViz、服务 |
| Torch 2.9.1 cu128 / torchvision 0.24.1 | 官方 PyTorch cu128 wheel 索引 | CUDA 张量、C++ 扩展算子加载及 wheel 运行依赖 |
| nvBlox v0.0.10 Ubuntu22 CUDA12 wheel | NVIDIA nvBlox GitHub release | 自带 libnvblox_lib.so、libpy_nvblox.so 与 Python 接口 |
| NumPy/SciPy/PyYAML 等 | pip / Ubuntu | 图像数组、位姿、配置及 wheel 导入依赖 |
| Eigen、glog、FCL 等开发库 | Ubuntu apt | 桥接、场景碰撞检查及插件构建 |
| RealSense ROS wrapper / SDK | 仅真 D435，ROS apt | 相机驱动；Gazebo 不需要 |

不需要下载整个 Isaac ROS、NITROS、GXF、cuVSLAM、人物识别模型和其他相机/雷达示例。`vendor/` 已保留匹配的 nvblox/stdgpu 头文件；它们来自上游/原安装的编译声明，不是运行时读取 workspaces。

官方推荐 Python 使用 pip wheel，并提供 Ubuntu22/CUDA12 下载项，见[官方安装说明](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/installation.html)。**不要把另一个仓库 `NVlabs/nvblox_torch` 的安装步骤与本工程使用的 NVIDIA core 仓库内置接口混用。**

## 2. 与代码对应的依赖约束

- `env.sh` 只 source `/opt/ros/humble/setup.bash` 与本工程 `install/local_setup.bash`，同时清理其他工作区的环境变量。
- `build.sh`、`native/CMakeLists.txt` 显式调用 `/usr/bin/python3`，因此当前脚本按系统 Python3.10 和当前用户的 site-packages 工作。单纯激活 venv 不会切换这些硬编码解释器；本文不把“创建一个 venv”冒充当前脚本支持的部署方式。建议新机器使用专门用户，避免影响其他 Python 工程。
- `native/CMakeLists.txt` 从该解释器实际安装位置读取 Torch 与 wheel，再链接 wheel 的 `lib/nvblox/libnvblox_lib.so`、`lib/nvblox_torch/cpp/libpy_nvblox.so`、Torch 库及 CUDART；运行 RPATH 含安装路径，故换路径/用户后重新编译。
- `tools/check_wheel.py` 检查 `_version.py` 中源码 SHA 必须是 `e4a2b00f45ecf989682d72a33a2c29defe39fd2d`，再检查 Torch CUDA 可用。文件名的 0.0.10 与已装包元数据的 `0.0.10.dev1+cu12ubuntu22` 不完全相同，不能只凭版本字符串判断。
- 本机该 wheel 的安装来源记录 SHA256 为 `9ffb308c2d94a01321d8c2558183118903cfcd4b13fe881fae197d9e5b5c96a8`。下文据此校验重新下载内容。
- 桥接按 `torch.compiled_with_cxx11_abi()` 设置 C++ ABI；不能拿其他版本 Torch 的头文件和本机旧 .so 混用。

本机 Torch 为 2.9.1+cu128，系统 nvcc 为12.6，两者不是同一项版本：前者包含 Python 运行环境的 CUDA 依赖，后者负责本地编译。此组合已完成此前本机 GPU 和仿真验证，但不代表任意版本组合都可工作。驱动兼容关系见[官方 CUDA 兼容说明](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)。

## 3. 源码打包与目标机目录

原机器：

```bash
cd /home/czz/perception_eye_in_hand
tar -czf /tmp/ur7e_nvblox_in_hand_source.tar.gz \
  --exclude='ur7e_nvblox_in_hand/build*' \
  --exclude='ur7e_nvblox_in_hand/install*' \
  --exclude='ur7e_nvblox_in_hand/log' --exclude='ur7e_nvblox_in_hand/.ros' \
  --exclude='ur7e_nvblox_in_hand/artifacts' --exclude='*/__pycache__' \
  ur7e_nvblox_in_hand docs/deployment
sha256sum /tmp/ur7e_nvblox_in_hand_source.tar.gz
```

源码必须保留 `native/`、`vendor/`、`src/`、`tools/`、配置和许可证。只复制 `src/ur7e_nvblox_in_hand` 不够。归档指令由操作者执行，本轮未生成归档。

目标机：

```bash
mkdir -p "$HOME/perception_eye_in_hand"
tar -xzf /tmp/ur7e_nvblox_in_hand_source.tar.gz -C "$HOME/perception_eye_in_hand"
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
```

## 4. 安装系统依赖

先逐项完成[公共系统、驱动、CUDA 和 ROS 安装](../../docs/deployment/SYSTEM_SETUP_UBUNTU2204.md)。尤其需要 `/usr/local/cuda-12.6/bin/nvcc`，只有 pip 安装 Torch 并不提供本工程所需的完整编译环境。

安装脚本 `./install_dependencies.sh --system` 可安装其列出的 apt 包并继续安装 Python 依赖，但它**不负责**配置 ROS apt 源、安装 ROS 基础环境、驱动或 CUDA。单独执行该脚本不等于完成空机器部署。

以下第5节提供固定版本、校验 wheel 的手工路线；选择这一条后不要再重复运行安装脚本覆盖版本。脚本的 OpenCV 范围为 >=4.8,<4.12，而原验证环境记录为4.5.5.62，因此脚本是部署配方，不能声称其所有依赖与旧测试环境逐项相同。

## 5. 安装 Python 库与校验 wheel

### 5.1 固定本机已记录的主要版本

在专门部署用户下执行，使用系统解释器，不使用 sudo pip：

```bash
/usr/bin/python3 -m pip install --user \
  setuptools==79.0.1 numpy==1.26.4 scipy==1.15.3 PyYAML==6.0.3
/usr/bin/python3 -m pip install --user \
  --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.9.1 torchvision==0.24.1
/usr/bin/python3 -m pip install --user \
  open3d==0.19.0 opencv-python==4.5.5.62 einops==0.8.2 \
  imageio==2.37.2 matplotlib==3.10.9 nvtx==0.2.15 \
  timm==1.0.25 transforms3d==0.4.2 trimesh==4.10.1
```

这是主要直接依赖的本机版本记录，**不是所有传递依赖的完整锁文件**。某个历史版本在目标下载源不可用时，应停下来核对/存档，不应随意升级后仍声称复现。`opencv-python==4.5.5.62` 是原环境已有的旧版本，仅用于复现；换成更新版本属于需要重新验证的部署变更。

wheel 的包元数据包含 open3d、timm、torchvision 等依赖，即使本工程不运行神经网络任务，导入官方封装仍可能需要安装它们。安装 timm 不表示启用了识别或下载了模型权重。

### 5.2 下载正确的官方 wheel

```bash
mkdir -p "$HOME/nvblox_downloads"
curl -fL -o "$HOME/nvblox_downloads/nvblox_torch-0.0.10+cu12ubuntu22-py3-none-linux_x86_64.whl" \
  'https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.10/nvblox_torch-0.0.10%2Bcu12ubuntu22-py3-none-linux_x86_64.whl'
printf '%s  %s\n' \
  9ffb308c2d94a01321d8c2558183118903cfcd4b13fe881fae197d9e5b5c96a8 \
  "$HOME/nvblox_downloads/nvblox_torch-0.0.10+cu12ubuntu22-py3-none-linux_x86_64.whl" \
  | sha256sum --check
```

校验成功后再安装；失败时不要执行下一条：

```bash
/usr/bin/python3 -m pip install --user --no-deps \
  "$HOME/nvblox_downloads/nvblox_torch-0.0.10+cu12ubuntu22-py3-none-linux_x86_64.whl"
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
/usr/bin/python3 tools/check_wheel.py
/usr/bin/python3 -m pip check
```

`--no-deps` 用于保留前面指定的 Torch/NumPy 等组合，不是忽略缺失依赖；pip check 报错需逐项处理。当前 SHA256 来自已安装包的 direct_url 记录，本轮未重新下载大型 wheel 做第二次下载校验。

新机系统也装有 Python apt 包时，用户 site-packages 中的 pip 版本可能覆盖它们；因此务必保持 NumPy1.x，避免直接升级到 NumPy2 破坏 Humble cv_bridge 等二进制模块。

## 6. 编译与底层检查

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export UR7E_CUDA_ARCH=89
export UR7E_BUILD_JOBS=1
./build.sh
source ./env.sh
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 tools/check_backend.py
./run.sh --show-args
```

89 对应本机 RTX4060。换卡可先查询：

```bash
/usr/bin/python3 -c 'import torch; print(torch.cuda.get_device_name()); print(torch.cuda.get_device_capability())'
```

例如返回(8,6)对应86；具体架构还需在当前 Toolkit 与 wheel 的支持范围内。原工程验证的是 amd64，不能把这个 wheel 移到 ARM Jetson 使用。

`build.sh` 先编译 native→install_native，再 colcon 编译 src 中的三个包。`check_backend.py` 使用合成平面验证原生 ESDF、inside/observed、查询、Mesh、地图保存加载清图和2D切片，不运行机械臂。若出现 ABI/符号错误，应重新核对 wheel SHA、Torch、CUDA 和重新构建，不删除版本检查。

检查实际库路径：

```bash
ldd install_native/lib/libur7e_nvblox_bridge.so
ros2 pkg prefix ur7e_nvblox_in_hand
ros2 pkg prefix ur7e_d435_gazebo
```

nvBlox 库应来自 Python wheel，ROS 包应来自本工程 install，不能指向原 workspaces/ros_pack 或兄弟工程。最终导入是否成功以 check_backend 为准；单独 ldd 无法替代 Torch 实际加载测试。

## 7. 完整仿真运行指令

各终端使用本工程 env.sh，保持 DDS域135 / Gazebo端口11565。先场景与观察姿态，再启动建图，避免把机械臂接近过程写进静态地图。

终端1：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
./scene.sh gazebo_gui:=true
```

终端2：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
source ./env.sh
/usr/bin/python3 tools/view_sweep.py --check-only
/usr/bin/python3 tools/view_sweep.py --seconds 0
```

终端3：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
./run.sh start_scene:=false rviz_mode:=diagnostic \
  camera_width:=640 camera_height:=480 voxel_size:=0.03 \
  depth_hz:=40 color_hz:=40 esdf_hz:=33.4 esdf_view_hz:=33.4 \
  mesh_hz:=5 layer_hz:=5
```

40/33.4 是调度上限，不是图像实际输入频率。场景原始 RGB/Depth 输入是30Hz；40Hz融合上限用于避免30Hz时间戳量化造成周期性丢帧，不会凭空生成40张图像。3D ESDF 默认只在地图变脏后更新/输出，亦不会靠重复发同一帧填满显示Hz。

静止15秒后，需要左右轻扫时在终端2运行：

```bash
/usr/bin/python3 tools/view_sweep.py --seconds 20 --amplitude 0.08
```

它仅针对本工程 Gazebo，不环绕、不用于实机。只需静态观察则不执行该条。

终端4：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
source ./env.sh
ros2 topic list -t
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

Ctrl+C 后按需分别运行：

```bash
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/color/image_raw --wall-time
ros2 topic hz /ur7e_nvblox/mesh --wall-time
ros2 topic info /ur7e_nvblox/esdf_points --verbose
```

| 输出 | 类型与解释 |
| --- | --- |
| /ur7e_nvblox/esdf_points | PointCloud2，原生 observed ESDF 体素中心，距离及inside/site标记 |
| /ur7e_nvblox/mesh | 标准 MarkerArray 完整彩色Mesh；与官方分块nvblox_msgs协议不同 |
| /ur7e_nvblox/tsdf_layer、color_layer | PointCloud2，原生地图层 |
| /ur7e_nvblox/camera_pose、camera_path | 机械臂TF对应的相机位姿/路径，不是SLAM估计 |
| /ur7e_nvblox/metrics | JSON字符串计数、频率、计时与库信息 |
| /ur7e_nvblox/static_esdf_pointcloud | 仅2D模式发布，3D模式没有该数据属于正常 |

ESDF内部点、自由空间点与表面Mesh是不同对象；unknown不会凭空成为已知自由空间。全体 observed ESDF 并不只贴着障碍物表面。

## 8. 自动验收、截图及结果解释

停止上面的手动实例，确认端口11565没有本工程残留后：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
source ./env.sh
printf '%s\n' "$DISPLAY"
/usr/bin/python3 tools/sim_test.py
```

在可用图形桌面终端执行，DISPLAY应为实际会话，不盲目写死:1。脚本保存 `artifacts/simulation_时间/`，包含静止/左右扫描、点云与Mesh、服务/开关、2D和层输出验证及RViz截图。只有 status=completed 的结果可作为完整成功样本；失败目录保留日志用于排错。额外打开多个RViz可能增加负载并影响截图匹配。

此前本机验收（不是本轮迁移重测）：

| 场景 | 深度融合 | 颜色融合 | ESDF更新调用 | 3D点云发布/新输入关联 | 输出年龄P50/P95 |
| --- | --- | --- | --- | --- | --- |
| 静止 | 29.19Hz | 29.86Hz | 29.19Hz | 26.65Hz / 26.65Hz | 40 / 45ms |
| 左右移动 | 29.54Hz | 29.84Hz | 29.54Hz | 25.95Hz / 25.95Hz | 39 / 44ms |

历史条件为640×480、0.03m、Gazebo GUI关闭、RViz开启，并订阅地图层；本轮未重新启动测试。截图、输出点数及完整口径见[原验证文档](VALIDATION_20261109.md)。GPU kernel耗时、ROS仿真时间年龄、墙钟频率和RViz FPS不能混为一个指标。

## 9. 真 D435 部署入口

先安装公共文档中可选 RealSense 依赖，提供真实机械臂 joint_states/robot_state_publisher 与正确手眼 TF，再执行：

```bash
cd "$HOME/perception_eye_in_hand/ur7e_nvblox_in_hand"
./run.sh start_scene:=false launch_camera:=true use_sim_time:=false \
  camera_width:=640 camera_height:=480 rviz_mode:=diagnostic
```

当前 launch 使用 `depth_module.profile`、`rgb_camera.profile`，应匹配所装 wrapper 参数。核对真实发布话题是否与配置的 `/camera/depth/image_rect_raw`、`/camera/color/image_raw` 和 CameraInfo 一致；不同 wrapper 的双层 camera 命名空间不能靠假设解决，应通过 mapper_config YAML覆盖实际话题。

本工程不会自动安装机械臂驱动，也没有提供适用于任意实机的控制命令。仿真性能不等于实机时间同步/标定精度已验收。

## 10. 故障与离线交付

| 错误 | 优先检查 |
| --- | --- |
| Wheel source commit differs | 下载包与vendor不匹配；恢复正确SHA，不绕过检查 |
| PyTorch CUDA unavailable | 使用的是CPU Torch、驱动未加载、权限/容器GPU未映射 |
| undefined symbol / libtorch / nvBlox符号错误 | 是否混用Torch/wheel/头文件，是否复制旧install_native，是否加载兄弟工程库 |
| NumPy2/cv_bridge异常 | 恢复NumPy1.26.4并核对系统Python和pip包覆盖关系 |
| Gazebo无画面/Mapper没图 | 先检查输入topic、CameraInfo、TF、/clock和启动日志 |
| 只有少量话题 | 同域/RMW、场景/Mapper进程是否存活；不要仅反复运行topic hz |
| 3D有点但2D话题无输出 | 检查esdf_mode；两模式不是同时运行 |
| Mesh不显示 | 本工程应使用MarkerArray，不能套官方nvblox_msgs插件类型 |
| 频率低于本机历史值 | 检查输入与成功融合、导出/打包耗时、地图规模、订阅负载和硬件 |

离线迁移应在相同Ubuntu22 amd64/Python3.10的联网准备机下载本节全部wheel及传递依赖，保存hash，并缓存apt/deb及其传递依赖；只下载 nvBlox 一个wheel不够。可用 `pip download --dest wheelhouse ...` 按本节固定版本和Torch索引准备，再用 `--no-index --find-links wheelhouse` 安装，但需实际验证目录中包含所有依赖。本文没有交付完整离线apt镜像或宣称已验证离线一键安装。

本轮新增文档、审计与README入口，没有修改感知代码、系统驱动或依赖版本。干净机器/其他GPU部署仍需执行上述编译与验收。
