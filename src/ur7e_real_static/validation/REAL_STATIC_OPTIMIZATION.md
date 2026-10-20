# 真机静态流程感知优化

日期：2026-09-24。环境：本机 ROS Humble、whole_control、CUDA/nvBlox、storm_kit SAGE。
没有连接真实 UR7e/D435i；所有动作测试使用 `/mock/` 话题、模拟 Action 控制器和隔离 ROS_DOMAIN_ID。

## 阅读后的取舍

`realtime_nvblox_2/realtime_nvblox/nvblox_backend.py` 的固定网格查询缓存、输出缓冲复用及 `ros_node.py` 的融合/输出分离，适合减少重复分配与显示开销。
此前已在 `ur7e_nvblox.esdf.EfficientBackend` 实现可复用后端；本轮让真机静态包直接使用它，不复制另一套原生绑定代码。

| 本次采用 | 在真机静态包中的实现 |
|---|---|
| GPU 查询点与输出缓冲复用 | 每次扫描新建后端时准备一次固定查询网格，每帧只查询并取回拥有独立存储的结果 |
| 处理与可视化分离 | 独立显示线程、latest-only 队列、订阅检查、1 Hz 默认上限；采集和服务回调不再打包/发布大点云 |
| 减少无效体积的计算 | 自由区域包围盒加未知边界后执行精确 EDT；复用此前实现的 CUDA EDT，可选择 SciPy CPU |
| 有界输入与统计 | 15 Hz 处理上限、小型待匹配队列、较短轮询等待；图像时间 TF/关节匹配；拒绝重复、逆序、过期和扫描前图像 |
| 便于定位停顿 | 完成率、输入覆盖计数、TF 等待、过滤、GPU 等待、原生查询、保守场、显示和总耗时 |

本包冻结后直接用完整规则网格做整臂路径检查，因此保留这个网格。显示复用已经查询的原生网格结果，避免为了画图再次遍历全部 TSDF 块。
每个提交帧均计算完整保守场；未采用在线包的自由空间扩张延期策略。未知空间不允许通行，没有加入 Gazebo 地板或扫描走廊先验。
冻结再次检查 CameraInfo，文件写入成功后才提交冻结状态。线程初始化失败会报告 CAPTURE_ERROR，并保留重新 start_scan 的能力。

流程仍为 **操作员带动扫描并停稳采集 → 冻结静态地图 → SAGE 离线规划 → 显式低速执行**。
默认执行关闭，速度/加速度维持 0.05 rad/s、0.10 rad/s²；没有自动预扫描运动，也没有动态避障能力。

## 对照测试：同一个原生地图、相同输入

使用修改前的 `FieldBuilder` 快照与新实现，对 373,977 个网格点、65 个平面观察帧逐帧比较，前 5 帧作为预热；每帧交替新旧调用顺序。
原生 ESDF 旧查询与缓存查询逐点相等；新旧最终距离、来源标记在全部 65 帧中逐点相等。

| 阶段 | 修改前 P50 / P95 | 修改后 P50 / P95 |
|---|---:|---:|
| 原生网格 ESDF 查询 | 1.267 / 2.387 ms | 0.283 / 0.334 ms |
| 保守自由空间场更新 | 32.204 / 35.980 ms | 10.870 / 13.428 ms |

两个阶段的中位耗时分别下降约 78% 和 66%。它们不包含原生融合、ROS 传输、TF 等待、机器人自身过滤或动作执行，不能换算成整条流程的 Hz。
原始结果：[benchmark.json](upgrade_20260924/benchmark.json)。

## 软件及 ROS 闭环结果

- **21 项单元/GPU 测试通过**，包含实际 SAGE、CPU/GPU FK、未知空间、冻结保存、轨迹/取消以及本轮新增检查。3 条上游弃用警告，无测试失败。
- 两次合成 ROS 闭环通过，实际调用深度解码、图像时间 TF、自身过滤、原生 nvBlox、冻结、SAGE 和 FollowJointTrajectory。
- 验证空深度与错误相机 TF 不增加有效帧数；冻结后停止采集；无点云订阅时不打包，冻结后加入的订阅能收到表面。
- 验证无自动运动、无地图拒绝、未经场景确认拒绝执行、A→B 完成、关节反馈断流取消、失效清图、重载重新确认。
- 实际根目录启动脚本在默认预览模式启动成功，SIGINT 后干净退出。

| ROS 合成输入设置 | 有效场完成率（2 秒窗口） | 冻结帧数 / 视角数 | 闭环 |
|---|---:|---:|---|
| 160×90、32FC1、定时器 10 Hz | 9.99 Hz | 27 / 2 | 通过 |
| 640×480、16UC1 毫米、定时器 15 Hz | 5.49 Hz | 25 / 2 | 通过 |

上述输入 Hz 是模拟发布定时器设置；实际传输与调度可能更慢。高分辨率版本在同一 Python 进程生成平面深度、发布关节/TF、建图并运行模拟控制器，存在资源竞争和过期帧。
该次样本的原生融合/查询约 1.64 ms，但从收到图像到完成结果约 200 ms，保守场阶段约 128 ms，即便该帧不需要重算 EDT。说明原生 ESDF 查询提速没有消除整个链路的长尾；当前测量不能进一步区分 Python 调度、数组运算和系统负载各自占比。
**未证明真实 D435i 达到 15 Hz，也未证明已解决严重降频或硬实时问题。** 后续需要真实驱动分进程输入下的长时间测量；按状态细分指标定位，保留过期拒绝而非放宽门限。

合成场景另有显式测试自由区，用于补足测试机器人近场；两次测试均记录观测/先验体素数量。这证明软件闭环可运行，不证明相机独立覆盖整个真实场景。
模拟控制器理想跟踪，末端数值误差不作为真机精度指标。真实覆盖、标定、工具碰撞包络、取消和停止距离仍按 [现场测试说明](../REAL_TEST_GUIDE.md) 验证。

## 文件与复现

本轮修改仅限 `ur7e_real_static` 与根目录 `build_ur7e_real_static.sh`；构建限定本包目录，避开原包和 realtime_nvblox_2 的 ROS 包名重复。
四个关联包逐文件 SHA256 验证记录见 [protected_integrity.json](upgrade_20260924/protected_integrity.json)。
结果与日志位于 [upgrade_20260924](upgrade_20260924)，旧包快照在工作区 `log/real_static_upgrade/20260924/before`。

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_real_static.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/wqj/miniconda3/envs/whole_control/bin/python -B \
  -m pytest -q -p no:cacheprovider src/ur7e_real_static/test
# 每个输出目录必须不存在；下例全部使用合成输入，不连接设备。
ROS_DOMAIN_ID=110 /home/wqj/miniconda3/envs/whole_control/bin/python -B \
  src/ur7e_real_static/test/run_mock_integration.py --output /tmp/real_static_mock_low
ROS_DOMAIN_ID=111 /home/wqj/miniconda3/envs/whole_control/bin/python -B \
  src/ur7e_real_static/test/run_mock_integration.py --depth-size 640 480 \
  --depth-encoding 16UC1 --depth-hz 15 --output /tmp/real_static_mock_d435i
/home/wqj/miniconda3/envs/whole_control/bin/python -B \
  src/ur7e_real_static/test/benchmark_capture.py \
  --baseline-field log/real_static_upgrade/20260924/before/ur7e_real_static/field.py \
  --output /tmp/real_static_same_map_benchmark
ROS_DOMAIN_ID=112 /home/wqj/miniconda3/envs/whole_control/bin/python -B \
  src/ur7e_real_static/test/run_launch_smoke.py --output /tmp/real_static_launch
```
