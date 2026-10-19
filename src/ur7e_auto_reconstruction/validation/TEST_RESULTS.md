# 自动扫描功能包验证记录

日期：2026-09-29。运行环境：ROS Humble + `whole_control`。

## 最新离线修复验证

针对 S08 采集超时，已将高频运动控制与静止采集分开：每点释放控制、再次确认静止后采集。19 项控制测试、12 类协调器故障测试及完整模拟 8 点 + 真实 CUDA 闭环均通过；S08 注入约 80 ms 主进程停顿后正常完成，8 点均至少 8 帧，共融合 67 帧。**本次没有进行真机复测**。详见 [修复说明](capture_fix/REPAIR_NOTES.md)。下文保留此前分阶段结果。

## 结论

软件层已打通：RTDE 风格反馈/命令 → 8 个观察点 → 连续停稳 → RGB-D 融合 → 彩色地图导出 → 停止。自动包与手动包独立。

**更新：已进行两轮真机运动测试。** 第二轮依次到达 8 点，保存 58 帧与彩色地图；在 S08 停稳采集期间因控制周期超时终止，完整扫描未通过。用户反馈“无明显抖动”。详见 [真机测试报告](../report/2026-09-29_real_auto_scan_test.md)。下文 CUDA 平面等数值属于此前模拟 RTDE 测试，不能替代真机结果。

## 验证结果

| 验证 | 结果与证据 |
|---|---|
| 构建 | colcon 构建成功；[build.log](build.log) |
| 控制层 | 最新 18 项通过；[最新日志](../report/evidence/final_unit_tests.log) |
| 8 点闭环 | ROS_DOMAIN_ID=84，模拟 RTDE、合成 RGB-D、真实 CUDA/nvblox TSDF；[results.json](integration/results.json) |
| 取消执行 | 第二轮在进入段取消，控制端停止，命令数不再增加，不自动恢复 |
| 协调器故障 | 无有效采集帧、感知心跳丢失、反馈丢失均 ERROR 并停止；连接期间取消为 CANCELLED，发送命令数为 0；新增延迟冻结状态用例验证部分地图记录，共 5 项通过；[结果](faults/results.json) |
| 安装启动入口 | ROS_DOMAIN_ID=85，实际 launch、实际 robot_state_publisher、已冻结地图回看；[结果](launch_results.json) |
| 真实只读检查 | 默认 PREVIEW_ONLY、allow_motion=false；1668 对实时 RGB-D 已接收，关节缓存 500 个样本；地图融合帧数为 0、采集门关闭；[结果](live_receive_only_results.json) |
| 原实现完整性 | 核对 `ur7e_real_reconstruction`、`ur7e_real_static`、`realtime_nvblox`、`realtime_nvblox_2` 共 275 个文件，无变化；[结果](source_integrity.json) |

控制层覆盖：选定各段速度/加速度界限、起停连续性、关节限位、非法目标、模型/FK一致性、过期或篡改预览、反馈重复时间戳、反馈超时、保护状态、跟踪偏差、控制周期超时、协调器心跳丢失、速度缩放、取消。软件 watchdog 不能代替机器人安全硬件。

## CUDA 闭环数值

合成场景为 `base_link` 中 z=0.15 m 的红色平面。测试从 S01 起步，按完整顺序完成 8 个观察姿态，使用生产默认 0.12 rad/s、0.18 rad/s²；不是提前跳到目标的假控制。

- 融合 **65 帧**，保存 **8 组**独立观察数据，各点至少 8 帧。
- 导出 **9,140 个顶点、14,015 个三角面**。
- 顶点到合成平面高度误差 P95：**3.25 mm**。
- 明显红色顶点比例：**94.90%**。
- 测试包装器将 ESDF 更新 API 替换为会报错的函数，整个闭环未触发它。
- 最大单周期关节命令变化 **0.000240 rad**，符合 0.12 rad/s ÷ 500 Hz 的采样步长界限。
- 模拟控制线程周期 P95 **5.04 ms**。500 Hz 是设定目标，普通 Python/Linux 在该并行测试负载下并未做到所有周期 2 ms。实现按有界采样推进，因此变慢时不会直接跳过命令；超过 40 ms 则停止。此数字不代表实际 UR 控制频率。
- 整个测试约 **148.2 秒**，包含准备、CUDA 初始化、全部点位采集、导出和第二轮取消测试，不能直接等同于真机扫描耗时。

地图路径及完整观测记录见 `integration/results.json` 的 `final_surface`。轨迹命令、实际关节角、速度及时间戳在对应 `plans/run_*/servo_trace.npz`。

## 启动与 TF

实际安装的启动入口在不运行控制节点的模式下加载冻结地图，订阅者获取到全部 9,140 个顶点。`robot_state_publisher` 输出的 `base_link → camera_depth_optical_frame` 与校准 FK 的矩阵最大元素差约 **2.74×10⁻⁸**。所有测试启动的子进程正常退出。

测试端曾在继承旧库路径的环境下出现 DDS 发现不完整；使用与启动器一致的环境清理后，启动/TF/回看测试通过。未将该现象归因为某个确定 DDS 缺陷。新增 `ur7e_auto_scan_control.sh` 与主启动器采用相同的干净 ROS 环境，便于稳定调用服务。

完整闭环还发现并修复了 `DONE` 先于 `result.json` 写入的时序问题：现在只有地图和运行记录保存后才报告完成。服务发现也使用限时等待，不依赖刚收到状态消息时服务就一定可调用。

## 真实起点预览

[实际起点预览说明](live_preview_final/PREVIEW.md)包含相机路径和各段机械臂起点/中点/终点形态。该文件由运行中的只读 `prepare` 服务生成。

该次快照的相机高度约 0.342 m，到 S01 的进入段标称约 14.0 s，完整标称运动约 94.1 s；另加停稳、采集、初始化/导出和速度缩放。预览只有在当前关节姿态仍处于 0.015 rad 容差内、模型和参数未改变时才可使用。姿态变化后必须重新 prepare。

上述只读阶段未接管控制。后续真机阶段已退出旧 UR ROS 控制驱动并交接给独立 RTDE，原手动预览保留；运行结果及最终停止状态见真机报告。

## 复现软件测试

以下命令不运行真实控制器；集成脚本还检查隔离的 ROS_DOMAIN_ID。

```bash
cd /home/wqj/realtime_nvblox_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
AUTO_PY=/home/wqj/miniconda3/envs/whole_control/bin/python
$AUTO_PY -B -m pytest -q src/ur7e_auto_reconstruction/test/test_control.py
ROS_DOMAIN_ID=84 $AUTO_PY -B src/ur7e_auto_reconstruction/test/run_integration.py
ROS_DOMAIN_ID=86 $AUTO_PY -B src/ur7e_auto_reconstruction/test/run_faults.py
ROS_DOMAIN_ID=85 $AUTO_PY -B src/ur7e_auto_reconstruction/test/run_launch_smoke.py
```

真实首次执行流程见 [REAL_TEST_GUIDE.md](../REAL_TEST_GUIDE.md)。

## 本次真机发现后的回归

CUDA 预热测试完成单点 8 帧（模拟控制接口、真实 CUDA），确认预热完成且真实融合帧仍为 0 后才建立控制接口。软件回归为 18 项控制测试和 5 项协调器故障测试，均通过。新增延迟冻结测试验证地图保存记录时序；控制周期超时的具体根因仍未解决。
