# 运行入口与当前代码现状

[返回文档索引](README.md)

## 根目录脚本对应关系

以下列出本次发现的全部 28 个根目录 Shell 脚本。此表解释入口用途，不表示当前机器已满足驱动、标定、模型、ROS overlay 或 GPU 条件。本次未执行任何启动/控制脚本。

| 构建入口 | 启动入口 | 目标 |
| --- | --- | --- |
| `build_ur7e_d435_scene.sh` | `run_ur7e_d435_scene.sh` | `ur7e_d435_gazebo scene.launch.py`，相机仿真场景 |
| `build_ur7e_esdf.sh` | `run_ur7e_esdf.sh` | `ur7e_nvblox mapping.launch.py`，深度/TF ESDF |
| `build_ur7e_reconstruction.sh` | `run_ur7e_reconstruction.sh` | `ur7e_reconstruction reconstruction.launch.py`，彩色表面重建 |
| 同上，并需场景包已构建 | `run_ur7e_reconstruction_scene.sh` | `ur7e_reconstruction scan_scene.launch.py`，扫描初始姿态场景 |
| `build_ur7e_sage_planner.sh` | `run_ur7e_sage_planner.sh` | `ur7e_sage_planner planning.launch.py`，基线规划与伺服 |
| `build_ur7e_perception_planning.sh` | `run_ur7e_perception_planning.sh` | `ur7e_perception_planning bringup.launch.py`，默认 demo 配置 |
| `build_ur7e_real_static.sh` | `run_ur7e_real_static.sh` | `ur7e_real_static bringup.launch.py`，静态地图/预览/显式执行应用 |
| `build_ur7e_real_reconstruction.sh` | `run_ur7e_real_reconstruction.sh` | `ur7e_real_reconstruction bringup.launch.py`，只读 RGB-D 重建 |
| `build_ur7e_auto_reconstruction.sh` | `run_ur7e_auto_reconstruction.sh` | `ur7e_auto_reconstruction bringup.launch.py`，自动彩色扫描 |
| `build_ur7e_auto_esdf.sh` | `run_ur7e_auto_esdf.sh` | `ur7e_auto_esdf bringup.launch.py`，自动深度 ESDF 扫描 |
| `build_ur7e_gazebo_esdf_sage.sh` | `run_ur7e_gazebo_esdf_sage.sh` | 原分段闭环，默认 ROS 域 87、Gazebo 端口 11387 |
| `build_ur7e_gazebo_esdf_sage_continuous.sh` | `run_ur7e_gazebo_esdf_sage_continuous.sh` | 连续闭环，默认 ROS 域 88、Gazebo 端口 11388 |
| 无独立配对脚本 | `run_gazebo_mapping.sh` | 基础 `realtime_nvblox gazebo_mapping.launch.py`；当前含冲突标记 |

另有四个辅助入口：

| 脚本 | 作用 |
| --- | --- |
| `source_realtime_nvblox.sh` | 定位 Torch/cuVSLAM/nvblox 动态库、加载 ROS 和历史工作区 overlay |
| `ur7e_auto_scan_control.sh` | 对 `/ur7e_auto_scan/` 调用 prepare/start/cancel，或查看 status |
| `ur7e_auto_esdf_control.sh` | 对 `/ur7e_auto_esdf_scan/` 调用 prepare/start/cancel，或查看 status |
| `ur7e_gazebo_esdf_sage_control.sh` | 在默认仿真域 87 中运行该包 control 客户端，支持目标、状态和服务操作 |

两种闭环启动脚本拒绝域 0；默认域/端口可通过 `UR7E_SIM_DOMAIN_ID` 与 `UR7E_SIM_GAZEBO_PORT` 覆盖。上述 control 包装器默认指向旧版域 87，不能不改环境就当作域 88 的控制终端。

## 环境和构建方式

根目录脚本通常清理 ROS/Python/动态库变量，再加载 `/opt/ros/humble/setup.bash` 和当前工作区 `install/local_setup.bash`。部分 `build_*.sh` 也要求已有 install overlay，因此它们不全是面向空环境的一键初次构建脚本。

建图或 SAGE 的可执行包装器多选择 `whole_control` Conda Python，常用覆盖项为 `SAGE_CONDA_PREFIX`；`run_ur7e_esdf.sh` 和 `run_ur7e_reconstruction.sh` 使用各自的 Conda 根目录变量并显式激活环境。Gazebo 场景采用系统 ROS 环境，避免 GPU wheel 动态库污染仿真进程。

主要依赖为 ROS 2 Humble、Gazebo 11/ros2_control、RealSense SDK 或 ROS 驱动、Torch/CUDA、nvblox_torch、cuVSLAM、NumPy/SciPy/YAML，以及外部 `storm_kit`。具体路径只按所选模式使用：ROS 深度 mapper 不需要直接接管 RealSense 或运行 VIO。

构建时应根据所选链路显式限定源码目录并确保依赖已安装。尤其当前同时存在同名 ROS 包副本，不能把全工作区无选择构建视为已验证流程。本次不调整包组织或构建脚本。

## 已确认的问题与行为差异

### 1. 基础 Gazebo 启动脚本残留合并冲突

[run_gazebo_mapping.sh](../run_gazebo_mapping.sh) 第 6–11 行包含 `<<<<<<< HEAD`、`=======`、`>>>>>>> upstream/main`。一支加载系统 ROS 和当前 overlay，另一支引用 `source_gazebo_env.sh`；当前根目录没有该文件。

本次对 28 个根目录 Shell 脚本运行 `bash -n`，仅此脚本语法检查失败。它当前无法正常作为 Bash 启动入口。后续修复需要明确采用的环境分支；本文档没有替用户选择或修改业务代码。

### 2. 两个目录声明同一个 ROS/Python 包

[第一份 package.xml](../src/realtime_nvblox/package.xml) 与[第二份 package.xml](../src/realtime_nvblox_2/package.xml) 都声明 `realtime_nvblox`；其 Python 包目录也均为 `realtime_nvblox/`。全工作区包发现/构建会遇到同名包问题，实际导入哪份还取决于 overlay 和 PYTHONPATH。

比较两份 Python 源码时，排除 `__pycache__`，差异集中在 `gazebo_ros_node.py` 和 `nvblox_backend.py`。第二份 Gazebo mapper 主要发布稀疏已观测体素 ESDF，dense 查询作为可选 debug 输出；launch 和配置资源也有差别。目录 `_2` 不是第二个 ROS 包名，不能直接写 `ros2 launch realtime_nvblox_2 ...`。

### 3. 真机默认配置不能直接移植

[真机重建配置](../src/ur7e_real_reconstruction/config/real.yaml)、[自动彩色 mapping](../src/ur7e_auto_reconstruction/config/mapping.yaml)、[自动 ESDF mapping](../src/ur7e_auto_esdf/config/mapping.yaml) 等仍含 `/home/wqj/...` 模型路径。相机序列号、机器人网络及外参也保存在具体部署脚本/配置中。

[real_static 默认配置](../src/ur7e_real_static/config/real.yaml) 的模型路径为空，`execution_enabled=false`、`commissioned=false`；其加载器会要求提供有效模型。`source_realtime_nvblox.sh` 仍使用 `$HOME/realtime_nvblox_ws`，而当前源码目录是 `perception_eye_in_hand`。这些都是移植时应检查的实际配置差异，不代表本次已部署或已连接设备。

自动路线还绑定模型文件 SHA256 与 waypoint 变换。换路径但保持文件内容可以保留模型身份；修改模型内容需要重新选择/检查观察点并更新相关产物，不能只更改文件名规避一致性检查。

### 4. 连续闭环 README 示例与 launch 默认模式不同

[连续版 bringup.launch.py](../src/ur7e_gazebo_esdf_sage_continuous/launch/bringup.launch.py) 默认 `planning_policy=strict`、`online_engine=v1`、`online_obstacles=false`、`hold_after_scan=false`。README 中介绍的 bounded_unknown/v2 在线实验需显式指定对应参数。该 launch 的 `auto_start` 默认是 true，启动仿真流程不能理解为只显示静态预览。

### 5. 自动扫描成功与目标避障就绪是两项结论

[自动扫描 trajectory.py](../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/trajectory.py) 和 ESDF 对应实现会校验关节范围、模型、固定路线、速度/加速度和计划身份，但生成计划明确记录 `collision_checked=false` 和 `self_collision_checked=false`。

自动 ESDF 的已有说明也区分 DONE/冻结成功与 `planning_ready=false`。后者涉及整臂观测覆盖、自身模型间距和目标任务条件，不能由“完成八点扫描”推导出任意目标可以执行。

### 6. 历史报告有不同日期与不同终态

自动彩色重建 README 保留早期末点采集失败及其离线修复说明；自动 ESDF README 记录另一日期/版本的八点完成。这些记录属于不同任务与控制路径。阅读时应检查 `result.json`、完整会话、失败原因和修复后是否重新实机验收。

## 配置参数如何影响代码

| 参数组 | 常见字段 | 作用与约束 |
| --- | --- | --- |
| 输入/几何 | `depth_topic`、`camera_info_topic`、`joint_topic`、`camera_frame`、`robot_urdf`、`collision_spheres` | 决定输入与统一运动学/碰撞模型；TF、图像 frame 和模型须一致 |
| 融合 | `depth_stride`、`min_depth`/`min_depth_m`、`max_depth`/`max_depth_m`、`voxel_size`/`voxel_size_m` | 控制采样、有效量程和 TSDF 空间分辨率；不同包键名不同 |
| 规划网格 | `grid_center`、`grid_size`、`grid_resolution` 及 `_m` 变体 | 决定规划/显示查询网格，和内部 TSDF 体素大小是两组参数 |
| 时间/停稳 | `frame_timeout`、`joint_timeout`、`settle_speed`、`settle_seconds` 等 | 决定输入时效和图像时刻停稳标准；不能直接用最新状态替代 |
| 调度 | `capture_hz`、`visualization_hz`、`esdf_hz` | 处理或显示目标上限，不保证实测完成率 |
| 地图冻结 | `min_frames`、`min_views`、`view_separation_rad` | 冻结前有效帧、不同观察点等门槛 |
| 真机执行 | `execution_enabled`、`commissioned`、`trajectory_action` | 控制 Action 执行能力；还需场景确认与 live 状态检查 |
| 碰撞/运动 | `collision_margin`、`self_margin`、`max_joint_speed`、`max_joint_acceleration` | 路线、整臂连接与时间参数化限制 |
| RTDE 控制 | `frequency_hz`、`cycle_timeout_sec`、`lookahead_time`、`gain`、反馈/跟踪/采集超时 | 直接控制周期、servoJ 和监督约束；由自动扫描配置校验 |
| 仿真策略 | `planning_policy`、`online_engine`、`online_obstacles` | 选择严格/有限未知准入和在线层版本，改变地图语义及进程结构 |

每个包文档列出配置文件与实际键分区；准确默认值请查看对应 YAML。相似字段不能跨包直接互换。

## 本次核查范围

已核对包清单、Python 静态语法、根目录 Bash 语法、文档相对链接和说明覆盖；发现的已有业务问题只记录，没有自动修复。没有运行 pytest、colcon build、集成测试、仿真或硬件控制，也没有验证本机 GPU/驱动兼容性。
