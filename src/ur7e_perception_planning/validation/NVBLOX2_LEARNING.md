# 阅读 realtime_nvblox_2 后的借鉴、修改和对照测试

2026-09-23；ROS Humble、whole_control、RTX 5060 Laptop GPU、真实 nvBlox/CUDA 与 SAGE。

**结论：固定查询、距离变换和地图吞吐改善；完整绕墙仍有长尾停顿，到达总时间没有改善。**
没有改动原 `realtime_nvblox` 和参考 `realtime_nvblox_2`；分别对全部 58 / 110 个现存文件及文件集合做 SHA256 校验，通过。

## 1. 阅读结论

参考启动脚本对应独立 `gazebo_ros_node.py`，数据流为：

`Image + CameraInfo → 最新帧槽 → 图像时刻 base_link/深度光学 TF → CUDA TSDF → 原生 ESDF → 已观测体素导出 → 独立发布线程`。

- Gazebo 前端不打开 RealSense SDK，不启动 SLAM；`runtime.py/ros_node.py/robot_pose.py` 则另有 SDK、SLAM/机器人位姿分支，不应混入这个仿真入口。
- 图像时刻 TF 已包含手眼固定链，不再重复乘外参。时钟回退通过 epoch 丢弃旧结果；初始化后再次拒绝过期帧。
- `observed_esdf_numpy()` 在 GPU 遍历 TSDF 块，按正权重选原生体素中心，再查询真正的 ESDF，过滤无效/unknown，最后读回。它不是直接发布 TSDF 距离。
- 默认关闭长方体稠密查询；debug 网格仅在启用且有订阅者时生成。完整稀疏点云按订阅门控，发布与融合分离。
- 稀疏输出只覆盖“TSDF 已观测候选体素且 ESDF 查询有效”的支持域，不保证导出整个 ESDF 层的所有有效体素，也不携带整张规则地图的未知空间约束。

原实现已经具备图像时刻 TF、最新帧队列、独立融合/发布线程和感知优先 GPU 锁。这些不是本轮新增功能。
参考默认 5 cm 体素、稀疏输出、30 Hz 更新配置；本规划 demo 为 2.5 cm 原生体素、2 cm 规则地图，还执行整臂自过滤、自由射线和未知边界距离变换。
因此不能将参考场景约 15 Hz 直接视为原规划闭环也应达到的频率。

当前参考 `run_gazebo_mapping.sh` 依赖的 `source_gazebo_env.sh` 缺失；并且两个感知目录的 ROS 包名都叫 `realtime_nvblox`。
本次通过显式 PYTHONPATH、配置和参考 Python 前端运行其代码，未修改参考包或补写其启动依赖。构建只发现两个待修改包，避免同名包冲突。

## 2. 实际改动

### ur7e_nvblox

- 本地 `esdf.py/EfficientBackend` 继承原后端；固定查询坐标/输出留在 GPU，避免每帧重新上传、拼接和 GPU clone，CPU 读回数组独立持有数据。
- 借鉴参考包增加 `esdf_output_mode: sparse`。默认 `dense` 保留原规则网格的位置顺序、unknown、字段及 QoS；不强迫原消费者改变接口。
- 无完整点云订阅者时跳过打包发送；修正计算完成后再额外等待整个周期的问题，改为开始到开始调度。
- `gazebo.yaml` 的目标更新率改为 15 Hz；不修改体素尺寸、相机外参或过期阈值。

### ur7e_perception_planning

- 356,136 个 demo 规划查询位置复用 GPU 缓冲，仍然发布完整规则 `PlanningMap`，不把稀疏点云直接当可行空间。
- 借鉴参考实现把候选筛选留在 GPU、减少 CPU 工作的思路，新增 CUDA **精确欧氏距离变换**。逐轴计算 `min_i(f(i)+(i-j)^2)`，在受支持网格上平方距离为可精确表示的整数，CPU float64 开方；与 SciPy 逐点相等。临时矩阵分块，上限约 32 MB；轴长超过 256 时回退 SciPy。
- 可配置 `free_space_backend: cuda|cpu`。CUDA 计算纳入已有感知优先 GPU 锁；没有新装依赖或修改 storm_kit。
- `free_space_update_hz: 5.0` 只合并**自由区增加**时的距离变换。旧自由区是新自由区的子集，其距离为保守下界；新障碍或旧自由证据丢失立即重算，不受该频率限制。
- 当前 native ESDF 距离始终参与取小值，旧保守距离不会隐藏新障碍。地图时间戳仍来自实际处理的深度图；空图、缺 TF、旧 epoch 不冒充新感知。
- 默认更新目标由 5 Hz（online 配置原为 4 Hz）改为 10 Hz；仿真输入仍是原有 10 Hz，没有通过降低分辨率、修改场景或提速控制器获得结果。
- 增加自由投影、距离变换、是否延后重算等分段统计。

碰撞净空、地图过期、关节反馈时效、运动速度参数均未放宽。

## 3. 相同输入和算子验证

### ROS 单独感知

30 Hz 合成平面、移动相机 TF、160×90 输入、3 cm 体素、15 Hz 更新配置；每种模式独立运行、稳态采样 15 s。

| 实现 | 实测更新 Hz | 延迟 P50 / P95 ms | 点数 |
|---|---:|---:|---:|
| 原 standalone dense | 10.09 | 61.44 / 64.71 | 52,111 |
| 新 standalone dense | 13.99 | 60.45 / 72.17 | 52,111 |
| 新 standalone sparse | 13.75 | 24.29 / 26.58 | 19,714 |
| 未改参考 sparse，同参数 | 11.43 | 42.99 / 46.93 | 19,714 |

这是统一参数测试，不是用户原参考 Gazebo 场景 15 Hz 的复现；不要据此宣称参考默认配置只有 11 Hz。
新 dense 吞吐增加，但其 P95 延迟没有改善。稀疏传输更少，采样语义也不同。

### 同数据算子

| 算子 | 修改前 P50 | 修改后 P50 |
|---|---:|---:|
| 固定 356,136 点 ESDF 查询/读回 | 1.139 ms | 0.376 ms |
| 移动视角自由场更新 | 6.752 ms | 1.923 ms |

查询随机交替顺序、重复 100 轮，丢弃前 10 轮；距离值逐点一致。
自由场用 90 帧、15 Hz 的确定性采集时序，参考为每帧精确 SciPy，新实现为 CUDA 和 5 Hz 扩展合并；其中 65 帧合并，所有输出均不大于同帧精确场。
另对不同大小、随机占用、边界与空/满内部体素逐点验证 CUDA EDT 与 SciPy 相等。
新 sparse 与参考后端输出的位置和距离逐点一致；同场景 sparse 导出 P50 0.908 ms，反而比小范围 prepared query 慢，因此没有盲目把规划默认改为稀疏。

## 4. Gazebo 完整首次到达与绕墙

每轮冷启动、headless、关闭 RViz、相同相机/体素/网格、原 SAGE 和速度设置。
首次目标 `[0.5,0,0.45]`，到达后通过 world/PoseStamped 改为 `[0.45,0.4,0.55]`。

| 实现/轮次 | 首次到达 s | 首段地图 Hz | 绕墙 s | 绕墙地图 Hz | 结果 |
|---|---:|---:|---:|---:|---|
| 修改前，5 Hz 配置 | 34.92 | 4.79 | 84.17 | 2.75 | 两段通过 |
| 仅缓存/调度，5 Hz，仍 CPU EDT | 34.41 | 4.80 | 99.16 | 2.55 | 两段通过 |
| 最终 CUDA，10 Hz，第 1 轮 | 33.94 | 9.15 | 89.64 | 4.66 | 两段通过 |
| 最终 CUDA，10 Hz，正式安装入口复测 | 37.09 | 9.09 | 90.64 | 4.69 | 两段通过 |

最终两轮的加密反馈轨迹/半采样间隔余量检查：环境球模型净空约 5.50 / 5.54 cm，自碰撞约 2.05 cm。
起终点直线穿过 cube1，实际末端从约 0.988 m 高处绕过，没有末端进入墙体；最终位置误差均小于 2 cm。
这仍是几何球模型验收，不是实体网格碰撞或 Gazebo 接触传感器证明。

### 没有改善的部分

- 绕墙总时间没有变快；原版 84.17 s，最终两轮 89.64 / 90.64 s。
- `STALE_MAP` 采样累计原版约 29.51 s，最终两轮约 34.47 / 33.14 s，仍会停动等待感知恢复。
- 长尾中自过滤、投影、GPU 提交/读回、Python/ROS 调度和 SAGE 都可能拖慢；某些帧没有重算 EDT 也超过 1 s，不能归因于 EDT 一个算子。
- 观察到约 400 MHz 的 CPU 频率中位数样本与长尾同时出现。但全核频率中位数还受空闲核心影响；本次没有证明温控、功耗、活跃核心调频或调度中哪项是根因，也没有更改系统功耗设置。
- 不同轮次资源状态不完全一致。算子对照支持提速结论，整段实测只支持吞吐改善，不能承诺硬实时、15 Hz 规划闭环或无停顿。

## 5. 回归、文件与复现

**54 项测试通过**：两个改动包 39 项，加上共享网格接口的 `ur7e_real_static` 15 项回归。3 条上游弃用警告；无失败。
覆盖 native 查询、稀疏语义、未知输出清理、CPU 输出独立性、CUDA/CPU EDT 相等、保守延期、新阻挡立即重算、TF 时间回退、地图版本、碰撞和 SAGE。
构建成功；最终正式入口 Gazebo/规划进程均正常退出。

- 精简记录：[nvblox2_learning](nvblox2_learning/)，含测试、构建、完整性校验、算子/ROS 对照和各轮 summary。
- 完整输入配置、修改前源码副本、逐帧状态及日志：工作区 `log/perception_planning_learning/20260923/`。
- 正式入口复测原始日志：`log/perception_planning_fast_demo/learning_cuda_10hz_installed/`。

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/setup.bash
cd /home/wqj/realtime_nvblox_ws
# 限定包发现范围，避免参考目录中的同名 realtime_nvblox：
colcon build --symlink-install \
  --base-paths src/ur7e_nvblox src/ur7e_perception_planning \
  --packages-select ur7e_nvblox ur7e_perception_planning

PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 RUN_GPU_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/wqj/miniconda3/envs/whole_control/bin/python -B -m pytest -q -p no:cacheprovider \
  src/ur7e_nvblox/test src/ur7e_perception_planning/test src/ur7e_real_static/test

./run_ur7e_perception_planning.sh
```

单独闭环自动验收沿用 `test/run_benchmark.py --no-rviz --duration 180 --tag <未使用标签>`。
同数据算子脚本是 `test/benchmark_esdf_pipeline.py`；可用 `--reference src/realtime_nvblox_2 --output /tmp/esdf_micro.json`。
源码隔离对照用 `test/run_learning_comparison.py`，`--source-root` 指向包含修改前两个包的目录。
这些自动仿真测试仅在空闲、隔离 ROS 域/仿真端口运行，不接入真实机器人。
