# UR7e 眼在手上 TF / ESDF 感知

本机测试结果和性能边界见 [TEST_RESULTS.md](TEST_RESULTS.md)。

复用 `realtime_nvblox.NvbloxBackend`、点云打包和统计工具；原包源码保持不变。
不导入其 RealSenseSource、Runtime 或 cuVSLAM，不打开真实相机。

数据流：仿真深度图 + CameraInfo + 图像时间戳的 TF → TSDF → 三维 ESDF → ROS 点云。
世界坐标固定为 `base_link`（默认配置），网格不会跟随相机移动。
机器人关节经已有 robot_state_publisher 计算正运动学，新节点直接查询
`base_link <- camera_depth_optical_frame`；不订阅控制命令，不重复发布机器人 TF。
深度未对齐 RGB，因此使用深度内参和深度光学坐标系。

## 构建、运行

在工作空间根目录：

```bash
./build_ur7e_esdf.sh
# 终端 1：已有仿真正在运行时无需重复启动
./run_ur7e_d435_scene.sh gazebo_gui:=false
# 终端 2：脚本自动激活 whole_control
./run_ur7e_esdf.sh run_rviz:=true
```

`ESDF_CONDA_ROOT` 可指定 conda 安装位置，默认 `$HOME/miniconda3`。
启动脚本不会安装或升级 conda 依赖。Gazebo 与感知使用独立进程和环境。
ROS launch 默认也指定 whole_control 的 Python，避免 colcon 生成的系统 Python shebang
误用非 CUDA 环境。可通过 `python_executable:=/path/to/python` 覆盖。

输入和参数见 `config/gazebo.yaml`，可以复制为新文件后传入 `config:=/absolute/file.yaml`。
输入话题和 `depth_frame` 可修改；`CameraInfo` 必须与深度图分辨率、光学坐标系一致。
支持 `16UC1` 毫米和 `32FC1` 米，处理大小端与行填充；默认隔点采样到 640×360，
同步缩放相机内参。`depth_stride: 1` 使用原始 1280×720。

TF 严格按图像时间戳查询，缺失时等待至超时并丢帧，不使用最新 TF 替代。
相机内参变化会重新建图；仿真 `/clock` 回退时也会重建地图。
输入队列只保留最新帧，慢处理时丢弃积压帧；统计区分 TF 缺失、过期与覆盖丢帧。
融合与点云发布各用一个独立工作线程，ROS executor 持续接收深度、时钟和 TF。
发布线程也只取最新结果，DDS 传输较慢时跳过积压结果。
输入时间必须接近当前 ROS 时钟，防止仿真重启后旧 DDS 图像污染新地图。
`ros_clock_minus_image_sec` 受 `/clock` 回调到达顺序影响，可略为负数；
`receive_to_result_ms` 则是节点收到图像至生成结果的墙钟耗时，不包含相机传输。

## 输出

| 接口 | 内容 |
|---|---|
| `/ur7e_nvblox/esdf_3d` | 完整规则查询网格，PointCloud2：x/y/z/rgb/distance |
| `/ur7e_nvblox/esdf_observed` | 已观测且位于配置距离带内的样本，适合 RViz |
| `/ur7e_nvblox/camera_pose` | 本次 ESDF 使用的深度相机位姿，PoseStamped |
| `/ur7e_nvblox/stats` | JSON：处理频率、丢帧、已知单元数、图像时间戳、运行 Python |
| `/ur7e_nvblox/reset_map` | std_srvs/Trigger，下一个有效输入帧开始重建 |
| `/ur7e_nvblox/save_map` | std_srvs/Trigger，保存 base_link_map.nvblox |

图像、ESDF、相机位姿均使用仿真时间；输出 frame_id 为 `base_link`。
完整 ESDF 默认使用 RELIABLE、KEEP_LAST(1)，降低大点云分片丢失；
`esdf_observed` 和位姿使用 BEST_EFFORT、KEEP_LAST(1)。
设置 `full_grid_reliable: false` 可切换完整点云，验证器对应加 `--best-effort`。
未知单元保留后端 sentinel（统计中提供 unknown_distance_m），显示灰色；
下游必须排除未知值，不能当作超大安全距离。

默认体素 3 cm，输出采样 5 cm，查询范围 2×2×1.5 m，中心 `[0,0,0.5]`。
ESDF 的 10 Hz 是更新频率上限；实际速度取决于 Gazebo 实时率、GPU 和发布开销。
相机单视角只能建立已观测区域；遮挡区域保持未知。
此包不提供机器人自身过滤、动态物体分离或地图衰减；机器人进入相机视野也会被融合。

## 验证

在仿真和 mapper 已运行时：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate whole_control
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
python -m ur7e_nvblox.verify --output /tmp/ur7e_esdf_check.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_GPU_TESTS=1 python -m pytest /home/wqj/realtime_nvblox_ws/src/ur7e_nvblox/test -q
```

验证器检查连续的真实输出、base_link 坐标系、非空有效距离、图像时间戳 TF
与输出位姿一致、whole_control 运行路径，并记录墙钟/仿真时间频率。
CUDA 单元测试融合已知深度平面，检查变换后的距离数值和未知空间。

清空地图建议调用 `ros2 service call /ur7e_nvblox/reset_map std_srvs/srv/Trigger '{}'`。
本机测试中 Gazebo `/reset_simulation` 后曾出现上游关节 TF 不再正常更新；
此时 mapper 会停止融合并等待有效 TF，需要重启场景恢复上游。
mapper 可以在场景重启、仿真时钟回退后恢复建图。
