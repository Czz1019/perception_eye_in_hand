# 原生 C++ ESDF 完整操作说明

> **sparse-v2 当前状态：** 已修复测试工具的 Thrust 链接错误及构建过期检查，生产目标和可选工具均已编译、链接、安装成功；**未执行工具测试，未启动新版感知或仿真**。下一步按 [构建修复与仿真操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 启动应用，再查看 Topic。算法/接口说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧性能和截图不能作为 v2 验收结果。

> 2026-10-27 统一入口更新：推荐使用本项目 `run_ur7e_perception.sh` 联合启动场景、输入、感知与显示，详见 [项目内统一感知结构与启动说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_INTEGRATED_DESIGN_AND_STARTUP.md)。下文分组件方式保留用于调试；不要与统一入口同时启动。

适用本机 `/home/czz/perception_eye_in_hand`，ROS 2 Humble，参考库 `/home/czz/workspaces/isaac_ros-dev`。日期：2026-10-27。实测数据、修改理由和边界见 [接入报告](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md)。本手册默认使用仿真，手动移动机械臂即可持续采集；不依赖 auto 扫描包。

## 1. 首次检查和构建

打开普通 Bash 终端，不必激活 Conda。原生节点直接链接官方 C++ 库，Python/Torch 仅用于旧版对照和验收工具。

```bash
cd /home/czz/perception_eye_in_hand
ls /opt/ros/humble/setup.bash
ls /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ls /home/czz/workspaces/isaac_ros-dev/install/nvblox_ros/lib/libnvblox_ros_lib.so
/usr/local/cuda/bin/nvcc --version
nvidia-smi
./build_ur7e_d435_scene.sh
./build_ur7e_esdf.sh
./build_ur7e_native.sh
```

第三条库文件路径以本机安装实际路径为准；若该路径不存在可用 `find /home/czz/workspaces/isaac_ros-dev/install/nvblox_ros -name '*nvblox_ros_lib*'` 核对。构建脚本通过 CMake 的 nvblox_ros 导出目标寻找库。

本轮现有依赖已满足，未安装新系统包。需要的组件：ROS Humble、Gazebo Classic、Cyclone DDS、CUDA 编译器及匹配驱动、workspaces 中已编译的 nvblox_ros/NITROS/GXF、glog/gflags、nlohmann-json、rosbag2_cpp，以及验收用 NumPy/pytest。遇到缺失时按构建日志补装同一发行版依赖；不要用另一套不匹配的 nvblox wheel 替代 C++ ROS 库。可先检查：

```bash
source /opt/ros/humble/setup.bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ros2 pkg prefix nvblox_ros
ros2 pkg prefix nvblox_msgs
ros2 pkg prefix isaac_ros_nitros
/usr/bin/python3 -c 'import numpy, rclpy; print("ROS Python ready")'
```

不同机器可以设置 `NVBLOX_ROS_WS` 指向对应官方工作区；本机默认 CUDA 架构是 RTX4060 的 89，换 GPU 时按实际设置 `ESDF_CUDA_ARCHITECTURES`。这两个变量需要在相应构建/运行终端中设置。

## 2. 每个运行终端先执行的公共环境

下面所有终端都先执行这一段，确保处于同一个隔离仿真域：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
```

终端 A：启动 Gazebo 场景。

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

做性能验收时使用 `gazebo_gui:=false`，与报告中的测试条件一致。等待关节控制器、相机和 TF 正常启动。只启动一份相同 domain/master 的场景；已有本轮场景仍在运行时无需重复启动。

终端 B：启动公共深度转换。两组都使用这一输入，内参同步缩放。

```bash
./run_ur7e_common_depth.sh --stride 2 --ros-args -p use_sim_time:=true
```

本机场景原始深度 1280×720，这一步输出 640×360、32FC1、单位米、有效范围 0.2～4 m。保留原图时间戳和 optical frame，输出 `/esdf_input/depth` 和 `/esdf_input/camera_info`。不要再给 mapper 加 stride=2。

终端 C：启动原生 C++ mapper。

```bash
./run_ur7e_native.sh
```

默认读取源码配置 `/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo.yaml`。必须只运行一个 `/ur7e_nvblox` mapper。切换后端时先在这个终端 Ctrl+C。

终端 D：查看实时效果。

```bash
./run_ur7e_native_rviz.sh
```

默认 Fixed Frame=`base_link`、Observed ESDF=`/ur7e_nvblox/esdf_observed`、Best Effort，显示的是已观测的 ±0.15 m ESDF 距离带，不会显示全部未知灰点。它是距离体素，不是彩色表面 Mesh。默认红色 Pose 箭头显示光学相机位姿；觉得遮挡可以取消 Depth Camera Pose。

本机官方 Mesh 插件要求的 GLSL150 资源缺失，所以默认配置完全不加载它。`native_with_mesh.rviz` 仅供资源完整的环境选择，不能在本机默认当作已验证的可用选项。Mesh 的显示、地图层流式发布及完整网格订阅会增加负载，性能测试时关闭。

终端 E：看实时更新频率。

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox:${PYTHONPATH:-}
/usr/bin/python3 -B -m ur7e_nvblox.monitor
```

监控程序自身订阅 observed 点云，持续运行；Ctrl+C 退出。限制时长可加 `--seconds 30`。输出例如：

```text
queued= 18.0  fusion= 18.0  ESDF= 18.0  export= 18.0  received= 18.0  unique_image_stamps= 18.0 Hz  points=9129  map_version=7609  error=None
```

`queued` 是 native 处理侧首次看到队列项，`fusion` 是实际积分，`ESDF` 是新输入驱动更新，`export` 是实际导出，`received` 是监控端接收频率。`unique_image_stamps` 是不同源图像时间戳数，说明没有靠重复旧 stamp 冒充更新；不代表场景每帧一定发生几何变化。RViz 右下角 30/31 FPS 只是 UI 帧率。

## 3. 单独检查话题、参数和内部延迟

以下命令在执行过公共环境的诊断终端使用：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /esdf_input/depth --wall-time
```

本机 Humble 的 `topic hz` 没有 `--qos-reliability` 选项，使用其默认传感器订阅策略；`--wall-time` 明确按墙钟计 Hz。Ctrl+C 后分别运行：

```bash
ros2 topic hz /ur7e_nvblox/esdf_observed --wall-time
ros2 topic echo /ur7e_nvblox/stats --once
ros2 topic echo /ur7e_nvblox/camera_pose --once --qos-reliability best_effort
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox update_esdf_rate_hz
ros2 param get /ur7e_nvblox export_hz
```

`stats` 的 `latency` 包含最近 512 次的 mean/P50/P95/P99/max：`depth_process_complete`、`esdf_complete`、`block_export_readback`、`queue_seen_to_esdf`、`queue_seen_to_publish`，单位 ms。C++ 完成阶段包含同一 CUDA stream 同步；它们不包含相机曝光之前的开销。共同观察器的输入到输出延迟请用下一节验收工具测量，不把这几个不同起点的数值混成一种延迟。

`map_latest` 的 `map_version`、`exported_map_version`、`input_stamp_ns`、`map_epoch`、`integration_size`、`voxel_size_m` 应同时检查。没有 ESDF 点云订阅者时，地图仍持续更新，导出应停止。`topic hz` 或此监控会成为订阅者，因此会重新触发按需导出。

## 4. 复现实测：相同条件下 C++ 与 Python 分开运行

先关闭 RViz、其他 ESDF 消费者和监控，只保留 A/B/C。验收脚本 `--exercise-simulation` 会将**仿真**机械臂移动到测试观察姿态，再小幅摆动 wrist_1；它仅允许 domain 127。手动观察不需要运行此脚本。一次完整采样约两分钟，加预热和生命周期检查。

新终端先执行公共环境，再清理可能的 Python/ROS overlay：

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox:/home/czz/perception_eye_in_hand/src/realtime_nvblox:${PYTHONPATH:-}
export OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
mkdir -p /home/czz/perception_eye_in_hand/artifacts/my_native_test
/usr/bin/python3 -B /home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py \
  --external --common-input --mode native --voxel-size 0.03 \
  --seconds 60 --esdf-hz 30 --publish-hz 30 \
  --exercise-simulation --lifecycle \
  --output /home/czz/perception_eye_in_hand/artifacts/my_native_test/native_003.json
```

等待打印 `PASS`。输出保留 JSON、静止/运动快照 NPZ、最近 24 帧的 NPZ；JSON 报告实际更新/导出 Hz、唯一 stamp、观察器延迟百分位、配对数量和 TF 误差。`--external` 下 voxel-size/hz 参数用于预期检查/报告，**不会替你修改外部 mapper 参数**；两组启动配置必须预先一致。若不想控制运动，去掉 `--exercise-simulation`；此时不能拿结果冒充同轨迹运动对照。

然后在终端 C 停止 C++，保持场景和公共输入继续运行，在同一公共环境启动 Python：

```bash
ESDF_PYTHON=/usr/bin/python3 ./run_ur7e_esdf.sh \
  config:=/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/python_comparison.yaml \
  run_rviz:=false
```

另一个诊断终端用相同脚本、相同参数，把输出改为：

```bash
/usr/bin/python3 -B /home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py \
  --external --common-input --mode native --voxel-size 0.03 \
  --seconds 60 --esdf-hz 30 --publish-hz 30 \
  --exercise-simulation --lifecycle \
  --output /home/czz/perception_eye_in_hand/artifacts/my_native_test/python_003.json
```

Python 对照需要本机用户 site 中可用的 Torch/nvblox_torch；该环境已通过本次验证。不要同时运行两个 mapper，不要用观察器和 Python mapper 同进程的结果与 C++ 独立进程混比。顺序实时流的到达频率仍会波动，严格相同帧序列和时间线的实验还需要另做 rosbag 回放。

本轮正式数据位于 `/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/`，不会被上述自测覆盖。重新生成本轮归档图：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/esdf_mpl /usr/bin/python3 -B \
  /home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/render_results.py
```

这个绘图脚本读取本轮目录内的 JSON/NPZ；要画自己的数据，复制脚本到自己的结果目录并保持对应文件名。

## 5. 手动移动采集和修改精度

正常仿真使用 A/B/C/D/E 即可。利用现有仿真控制工具手动移动机械臂，mapper 通过图像时刻 TF 获得 pose；无需启动 auto_reconstruction、auto_esdf 或预定义扫描任务。默认对累积静态地图建图，移动机械臂会增加已观测区域；静止物体地图稳定是正常现象。

C++ 参数与 Python 参数名不同：

| 含义 | C++ | Python 同条件配置 |
| --- | --- | --- |
| 体素 | `voxel_size` | `voxel_size_m` |
| ESDF / 导出目标 | `update_esdf_rate_hz` / `export_hz` | `esdf_hz` / `publish_hz` |
| 导出范围 | `export_center_m` / `export_size_m` | `grid_center_m` / `grid_size_m` |
| 显示距离带 | `export_band_m` | `max_visualized_distance_m` |
| 坐标 | `global_frame` | `world_frame` |
| 图像坐标约束 | `expected_depth_frame` | `depth_frame` |

修改精度后重启 mapper，不要运行中只改查询网格。例如单独查看 2 cm：

```bash
./run_ur7e_native.sh -p voxel_size:=0.02
```

对照时 Python 也要改为相同精度，公共输入 stride 和 ROI 保持不变。Python 的 `esdf_output_mode: native` 下 `grid_resolution_m: 0.05` 不决定采样间距；其存在是兼容 dense 模式，不代表本次用了 5 cm 查询。C++ 原生中心点数随 voxel_size 改变，但点数还受场景、观测和距离带影响。导出设有 200 万候选单元上限；超出应合理设置工作范围或显式评估预算，不能静默截断。

目标 30 Hz 并不保证实际 30 Hz。`integrate_depth_rate_hz=1000` 是为绕开图像时间量化节流的内部上限；当前输入和 ESDF 目标仍是 30。实测本场景上游约 17～19 Hz，不能靠发布旧图把它显示成 30 Hz。

## 6. 清图、保存与规划查询接口

运行 native 后，普通诊断终端（公共环境 + `/opt/ros/humble/setup.bash`）：

```bash
ros2 service call /ur7e_nvblox/reset_map std_srvs/srv/Trigger '{}'
ros2 service call /ur7e_nvblox/reset_stats std_srvs/srv/Trigger '{}'
```

`reset_map` 清除工作区地图与输入队列，epoch 增长，等待新图像重新融合。`reset_stats` 清空滚动延迟样本，不清图；累计计数按前后差计算。

原生 save_map 使用官方 **FilePath** 服务，不是旧 Python Trigger。需要 source 官方消息包：

```bash
source /opt/ros/humble/setup.bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
mkdir -p /home/czz/perception_eye_in_hand/artifacts/manual_map
ros2 service call /ur7e_nvblox/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '/home/czz/perception_eye_in_hand/artifacts/manual_map/map.nvblx'}"
```

扩展名为 `.nvblx`；库会对不匹配的后缀追加此后缀。此文件保存地图层，MeshBlock 不属于此序列化器支持的层，不能把它当成表面模型文件。

有界三维距离查询示例（原生接口输出 origin、voxel_size、x/y/z layout）：

```bash
ros2 service call /ur7e_nvblox/get_esdf_and_gradient nvblox_msgs/srv/EsdfAndGradients \
  "{update_esdf: false, visualize_esdf: false, use_aabb: true, frame_id: 'base_link', aabb_min_m: {x: -0.1, y: -0.1, z: 0.0}, aabb_size_m: {x: 0.2, y: 0.2, z: 0.2}}"
```

该安装实现输出三维距离，**没有额外梯度通道**。服务默认未知值 -1000；完整 `/ur7e_nvblox/esdf_3d` 使用 1e6，包含整个 ROI，可达数 MB/帧，使用 Reliable QoS；`/esdf_observed` 已过滤未知。需要规划时先明确分辨率、origin、layout、未知处理和距离符号，不能直接套用旧 dense 消费者。保存或查询完整网格会增加处理延迟。

## 7. 验证构建与基础正确性

真实 CUDA 平面/官方导出器比较测试：

```bash
ESDF_NATIVE_EXECUTABLE=native_geometry_test ./run_ur7e_native.sh
```

预期包含 `PASS official_converter_max_error=0`。它是独立几何测试，不启动第二个 mapper；5 cm 测试平面不影响实际 3 cm 场景配置。

已有 Python 回归，在已 source ROS 且设置了两个源码 PYTHONPATH 的终端执行：

```bash
cd /home/czz/perception_eye_in_hand/src/ur7e_nvblox
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -B -m pytest -q \
  test/test_efficient_esdf.py test/test_native_sampling.py test/test_publish_versions.py
```

本轮 13 项通过。仿真、TF、按需导出和 reset 的集成正确性由第 4 节工具验证，不能用单元测试通过替代。

## 8. 常见问题

| 现象 | 检查和处理 |
| --- | --- |
| 没有点云 | 是否同 domain/RMW；公共图像是否发布；CameraInfo 尺寸/frame 是否匹配；图像时刻 base_link→camera_depth_optical_frame TF 是否存在；有无点云订阅者 |
| ESDF 只有约 18 Hz | 先看公共输入到达率；本轮实际场景即如此。不能由配置 30 推导实际输入 30 |
| 仍约 10 Hz | 是否运行本轮 native_mapper；是否仍使用早期 ROS now 周期调度；检查 `/clock`，本场景约 9.99 Hz |
| 画面有很多灰点 | 订阅了完整 esdf_3d 或旧 dense 点云；改看 esdf_observed。自由空间与未知空间语义不同 |
| 修改 voxel_size 点数不变 | 核对实际运行配置、是否旧 mapper、是否仍使用旧 dense 模式；native 点数还随可见范围和距离带变化 |
| 找不到 GXF/共享库 | 使用根目录 run_ur7e_native.sh，确保 NVBLOX_ROS_WS 对应本机完整 install；勿混入 Conda/Torch 的 LD_LIBRARY_PATH |
| RViz box.geom 异常 | 当前系统缺 GLSL150 资源；使用默认 native.rviz，勿加载可选 Mesh 配置 |
| RViz 崩溃但 mapper 正常 | 本轮另见一次无日志 GUI 段错误；默认点云配置后续已成功显示。保留日志检查图形驱动/插件，不能据此认定融合失败 |
| 生命周期断言 queries 不为 0 | 仍有 RViz、topic hz、监控、规划器或其他点云订阅者；关闭后重测 |
| TF / frame 校验失败 | 不要改 header 假装一致，修复真实外参、光学轴定义和图像时刻 TF |

## 9. 实机接入准备

物理运行性能尚未验收。接入时沿用同精度、同输入/内参和同应用 ROI，机械臂驱动提供关节状态，正确手眼外参接入 TF；深度必须为对应 optical frame 的深度图。不要把某一相机的图像配另一相机的内参。

在真实机器人所在 domain 设置环境，按实际相机话题运行公共转换，例如：

```bash
./run_ur7e_common_depth.sh --stride 2 --ros-args -p use_sim_time:=false \
  -p depth_topic:=/camera/depth/image_rect_raw \
  -p camera_info_topic:=/camera/depth/camera_info
./run_ur7e_native.sh -p use_sim_time:=false \
  -p expected_depth_frame:=camera_depth_optical_frame
```

以上是两个独立终端命令，话题/frame 需按现场核对。如果物理原图已经 640×360，公共 stride 应改为 1，确保没有意外降低输入分辨率。RViz 用 `ESDF_USE_SIM_TIME=false ./run_ur7e_native_rviz.sh`。跨机器时需相同 DDS/域及网络配置，并按实际需要取消 ROS_LOCALHOST_ONLY=1。

原生 mapper 不发送关节运动指令；实机不要运行 `--exercise-simulation` 测试。自体过滤、动态残影、时钟同步和规划消费者接入仍需专门验收。

## 10. 结束运行

依次在监控/RViz、mapper、公共输入、Gazebo 终端 Ctrl+C。不要用全局 pkill 清理其他工作区的 ROS/Gazebo。切回 Python 或 native 时，只停止当前 mapper，公共输入与场景可保持。

本轮效果和原始证据：[结果目录](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027)、[实际 RViz 截图](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/rviz_live.png)、[实时采样动图](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/native_live.gif)。
