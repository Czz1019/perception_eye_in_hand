# UR7e 手动眼在手感知：原生三维 ESDF

> **sparse-v2 当前状态：** 已修复测试工具的 Thrust 链接错误及构建过期检查，生产目标和可选工具均已编译、链接、安装成功；**未执行工具测试，未启动新版感知或仿真**。下一步按 [构建修复与仿真操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 启动应用，再查看 Topic。算法/接口说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧性能和截图不能作为 v2 验收结果。

> 2026-10-27：本包是已优化并保留的 Python/Torch 感知路径；本仓库新增原生 C++ 感知位于 `/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native`，使用 `run_ur7e_native.sh` 启动。本文下方 `run_ur7e_esdf.sh` 不会自动切换到 C++。详见 [已完成修改说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_REPOSITORY_COMPLETED_CHANGES.md) 和 [完整操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)。

本包从机械臂关节状态生成的 TF 获取相机位姿，将深度融合为原生 nvBlox TSDF/ESDF。
不发送真实机械臂运动指令，不依赖 cuVSLAM。默认采用 `base_link <- camera_depth_optical_frame`
的**图像时刻**变换，保留已有手眼标定；未对齐 RGB 的深度图使用对应深度内参。

2026-10-26 修改与实测见 [修改说明](../../docs/ESDF_ROS_ALIGNMENT_IMPLEMENTATION.md)。
早期测试记录见 [TEST_RESULTS.md](TEST_RESULTS.md)、[PERFORMANCE.md](PERFORMANCE.md)，其默认参数与环境不代表当前版本。

## 构建与运行

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_esdf.sh
# 终端 1；已有同域场景运行时不重复启动
./run_ur7e_d435_scene.sh gazebo_gui:=false
# 终端 2
./run_ur7e_esdf.sh run_rviz:=true
# 终端 3
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run ur7e_nvblox esdf_rates
```

启动脚本优先使用 `ESDF_PYTHON`，其次是已存在的 whole_control 环境，最后选择系统
`/usr/bin/python3`。选定环境需要已有 ROS、PyTorch CUDA 和 nvblox_torch；脚本不安装依赖。
本机实际使用系统 Python 和用户目录中的 CUDA wheel。可用 `config:=/absolute/file.yaml` 指定参数。
构建限定两个功能包路径，避免 `realtime_nvblox_2` 同名包冲突；自动清理这两个包的失效生成链接，
并移出旧的非法 `realtime_nvblox~upstream_main` 安装索引。

需要与现有 ROS/Gazebo 隔离时，两个启动终端必须使用相同域和 master URI，例如：

```bash
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
```

如果选择 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 做传输对照，场景、mapper、RViz 和监测终端
都要一致设置，不能只切换其中一个进程。测试报告分别记录不同 RMW 的结果。

## 更新与导出

融合、导出和 ROS 发布各用一个 latest-only 工作线程。融合线程只维护原生地图；导出线程
按订阅需求查询最新 ESDF，并受独立 `publish_hz` 上限控制。GPU 地图访问仍由同一锁保护，
并不是无锁并行；查询仍会占用 GPU/地图锁。

无点云订阅者时不查询、不读回。停止输入后不重复发布旧地图；同一话题同一地图版本最多发布一次。
输出时间戳仍为最近一次 ESDF 更新所对应的融合图像时刻，不能用当前时间伪装新鲜度。

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `voxel_size_m` | 0.05 m | 原生地图分辨率；native 模式同时决定查询间距 |
| `esdf_hz` | 30 Hz | 有新深度时的原生 ESDF 更新目标上限 |
| `publish_hz` | 30 Hz | 点云导出目标上限，不反复发布同一张图 |
| `esdf_output_mode` | native | 原生体素中心对齐的、有界 ROI 三维采样 |
| `grid_size_m` | [2,2,1.5] | 世界坐标系中的查询范围 |
| `grid_center_m` | [0,0,0.5] | 固定查询中心，不跟随相机漂移 |
| `grid_resolution_m` | 0.05 m | 仅 legacy dense 模式使用 |
| `max_visualized_distance_m` | 0.15 m | 已观测显示距离带；不删除内部自由空间 |
| `depth_stride` | 2 | 深度降采样并同步缩放内参 |

默认 native ROI 在 5 cm 下有 48,000 个查询位置，10 cm 下为 6,000；实际显示仅保留已观测距离带。
`native` 使用现有 Torch 原生 ESDF 查询 API 读取原生体素中心，**不是**移植 C++ 的块级 exporter，
也不遍历整张无界地图。已安装 Torch API 没有公开 ESDF layer view。
只有 observed 消费者时在 GPU 筛掉未知和距离带之外的值，读回紧凑结果；完整点云有人订阅时
才读回完整数组。小查询的 GPU 筛选也有固定开销，不能保证它比完整读回更快。

`dense` 保留旧的独立采样间距、对称网格顺序与未知值；旧固定网格消费者应显式选择它。
`sparse` 保留历史正 TSDF 权重候选导出，缺失位置不能据此认定自由，不保证比有界采样更快。
模式/分辨率通过启动参数或 YAML 设置，需要重启节点生效，不支持在线动态改图。

## 输出与实际频率

| 接口 | 内容 |
| --- | --- |
| `/ur7e_nvblox/esdf_observed` | 已观测距离带 PointCloud2；默认 RViz 显示，无未知灰色盒 |
| `/ur7e_nvblox/esdf_3d` | 完整采样（含未知），仅有订阅者才导出；调试/算法接口 |
| `/ur7e_nvblox/camera_pose` | 最近一次 ESDF 更新使用的图像时刻相机位姿 |
| `/ur7e_nvblox/stats` | JSON：实际融合、更新、导出、两路发布次数与耗时 |
| `/ur7e_nvblox/reset_map` | Trigger；下次有效输入重建原生地图 |
| `/ur7e_nvblox/save_map` | Trigger；保存 base_link_map.nvblox |

点云字段保持 `x/y/z/rgb/distance`。完整点云默认 RELIABLE，observed 和 pose 默认 BEST_EFFORT，
KEEP_LAST(1)。未知 sentinel 仍用于完整接口；下游必须剔除未知，不能把它当成大安全距离。

`esdf_rates` 同时显示 `depth/fusion/ESDF/export/received/unique_image_stamps`，单位为墙钟 Hz。
`unique_image_stamps` 表示不同融合图像的时间戳，不代表几何一定发生变化。
`topic hz` 只测消息接收：旧 ROS 查询节点可 30 Hz 重复读取 5 Hz 更新的场；当前实现不采用该策略。
当前即使目标写为 30 Hz，输入不足、TF 等待、GPU 或传输竞争仍可使实测明显低于目标。

`stats.map_latest` 是最新 ESDF 版本及图像时间；`stats.latest` 是最近导出发布结果的摘要。
`query_cells` 是候选采样数，`total_cells` 是读回数，`known_cells` 是读回中的有效数。
只有 observed 消费者时，`total_cells` 已经是 GPU 距离带过滤后的点数。
`esdf_updated` 记录实际调用更新的次数，`esdf_exported` 记录查询读回次数；
`full_cloud_published`、`observed_cloud_published` 只在实际调用对应 publish 后增加。
`esdf_published` 记录至少发布一路点云的结果批次，双路订阅不计两次。

`depth_submit`、`esdf_update_submit` 是 CPU 调用耗时；`query_readback` 还可能包含此前 CUDA 任务的等待，
不能当成纯 GPU kernel 时间。`image_age_at_publish` 使用 ROS 时钟与图像时间差，仿真时钟回调不同步时
可短暂为负；它不是可靠的墙钟端到端延迟。测试另记录接收后的墙钟耗时。

## 仿真与 CUDA 验证

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/ur7e_nvblox:$PWD/src/realtime_nvblox:$PYTHONPATH"
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_GPU_TESTS=1 \
  /usr/bin/python3 -B -m pytest src/ur7e_nvblox/test -q

# 场景已在域 127 启动，先关闭同域其他 mapper；该脚本自行运行一个 mapper。
# --exercise-simulation 只发送隔离 Gazebo 的位置控制命令，限制域 127。
ROS_DOMAIN_ID=127 /usr/bin/python3 -B src/ur7e_nvblox/test/run_gazebo_validation.py \
  --mode native --voxel-size .05 --seconds 15 --exercise-simulation --lifecycle \
  --output artifacts/esdf_ros_alignment_20261026/recheck_005.json
```

图像时刻 TF 缺失时最多等待 `tf_timeout_sec`，然后丢帧，禁止用 latest TF 替代。
内参变化、reset_map 和 `/clock` 回退会重建地图。Gazebo 全局 reset 可能使控制插件停止关节状态，
建议仅 reset_map；该上游限制不由感知节点修复。

本包尚未加入机械臂自身过滤、动态物体分离和地图衰减。机器人进入深度视野仍会参与融合。
