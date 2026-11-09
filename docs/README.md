# 项目代码说明文档

2026-10-28 操作补充：[扫描启动失败排查与完整仿真测试步骤](/home/czz/perception_eye_in_hand/docs/ORBIT_SCAN_TEST_GUIDE.md)，包含统一通信环境、分终端启动顺序、`--check-only`、实际频率/延迟与实时画面检查。

最新操作：[RViz 鼠标交互与障碍物组多视角扫描](/home/czz/perception_eye_in_hand/docs/RVIZ_INTERACTION_AND_ORBIT_SCAN_GUIDE.md)，含新增一条指令、已验证的可达范围及完整 360° 的限制。

新增分析：[重建效果、体素大小与表面平整度差异及优化方向](/home/czz/perception_eye_in_hand/docs/ESDF_SURFACE_QUALITY_ANALYSIS_AND_OPTIMIZATION.md)。对照 workspaces 的实际源码配置，区分显示对象、空间精度、TSDF/ESDF 参数和运动输入；本轮仅文档分析，未修改程序或运行仿真。

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

> 2026-10-27 统一入口更新：推荐使用本项目 `run_ur7e_perception.sh` 联合启动场景、输入、感知与显示，详见 [项目内统一感知结构与启动说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_INTEGRATED_DESIGN_AND_STARTUP.md)。下文分组件方式保留用于调试；不要与统一入口同时启动。

已完成修改的范围与文件清单见 [perception_eye_in_hand 感知改造完成说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_REPOSITORY_COMPLETED_CHANGES.md)。修改位于本仓库；workspaces 提供参考和第三方库依赖。旧 Python 入口与新 C++ 入口目前分别保留。

本文档面向第一次阅读本仓库的开发者，按“总体架构 → 目录 → 关键执行流程 → 功能包源码”组织。分析对象是早期静态分析的 13 个源码目录；2026-10-27 新增原生包后为 14 个源码目录；其中两个目录声明同一个 ROS 包名，详见运行现状说明。

当前 ESDF 专题入口（2026-10-27，原生模块已接入）：

- [原生 C++ 接入、修改说明与实测](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md)：直接复用 workspaces 官方模块、同 3 cm/640×360/ROI 对照、延迟、频率和效果图。
- [完整用户操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)：从构建、手动采集、RViz 到监控、对照、服务和实机接入准备。
- [早期设计](/home/czz/perception_eye_in_hand/docs/ESDF_NATIVE_REALTIME_DESIGN.md) 与 [早期操作手册](/home/czz/perception_eye_in_hand/docs/ESDF_OPERATION_MANUAL.md)：保留设计历史，其“尚未接入”状态已由上述实测报告更新。

专题分析：[ESDF 生成效果与实时性问题分析](ESDF_REALTIME_ANALYSIS.md)，对比用户提供的压缩包、当前仿真/实机代码和固定提交的 NVIDIA 官方源码，附[版本与证据索引](esdf_analysis/EVIDENCE.md)。该专题只做分析，不修改代码。

针对用户 RViz 灰色点阵现象的设计见[手动采集 ESDF 感知修改方案](ESDF_PERCEPTION_MODIFICATION_PLAN.md)。用户后续授权实施后的改动、兼容方式和真实仿真结果见[ROS 逻辑对齐修改与实测](ESDF_ROS_ALIGNMENT_IMPLEMENTATION.md)，区分已实现部分与仍保留的边界。

用户确认的 workspaces 官方 ROS 移植版对比见[查询逻辑、voxel_size 与实时性差异说明](ROS_NVBLOX_QUERY_VOXEL_COMPARISON.md)：解释固定查询网格与原生体素导出的区别，核对三维服务、实际 launch 参数及“查询占主要耗时”的证据边界。

## 阅读顺序

1. [目录说明](DIRECTORY_GUIDE.md)：先判断哪些是源码、配置和实验产物。
2. [架构与数据流](ARCHITECTURE.md)：理解共享底层、各运行分支和地图语义。
3. [关键流程说明](CODE_WALKTHROUGH.md)：沿函数入口追踪采集、规划和控制。
4. [运行入口与现状](ENTRYPOINTS_AND_FINDINGS.md)：识别实际启动默认值和当前已知阻碍。
5. 下表各功能包文档：按文件查看职责、符号位置、接口和参数。

## 功能包文档

| 源码目录 | 主要职责 | 说明 |
| --- | --- | --- |
| `realtime_nvblox` | RealSense/cuVSLAM/nvBlox 实时运行库 | [代码说明](modules/realtime_nvblox.md) |
| `realtime_nvblox_2` | 同包名基础库副本及 ESDF/Gazebo 扩展 | [代码说明](modules/realtime_nvblox_2.md) |
| `ur7e_d435_gazebo` | UR7e 相机仿真、场景、标定及 C++ 插件 | [代码说明](modules/ur7e_d435_gazebo.md) |
| `ur7e_nvblox_native` | 官方 ROS 原生 C++ 感知、块级 ESDF 导出、版本和统计 | [模块说明](/home/czz/perception_eye_in_hand/docs/modules/ur7e_nvblox_native.md) |
| `ur7e_nvblox` | 图像时刻 TF 深度融合及原生 ESDF | [代码说明](modules/ur7e_nvblox.md) |
| `ur7e_reconstruction` | 仿真彩色 TSDF 表面重建和扫描 | [代码说明](modules/ur7e_reconstruction.md) |
| `ur7e_sage_planner` | 固定场景 SAGE 与关节伺服基线 | [代码说明](modules/ur7e_sage_planner.md) |
| `ur7e_perception_planning` | 在线感知地图、SAGE 和独立执行检查 | [代码说明](modules/ur7e_perception_planning.md) |
| `ur7e_real_static` | 真机静态 ESDF、离线规划、显式 Action 执行 | [代码说明](modules/ur7e_real_static.md) |
| `ur7e_real_reconstruction` | 只读真机 RGB-D 重建 | [代码说明](modules/ur7e_real_reconstruction.md) |
| `ur7e_auto_reconstruction` | 独立 RTDE 自动八点彩色扫描 | [代码说明](modules/ur7e_auto_reconstruction.md) |
| `ur7e_auto_esdf` | 独立 RTDE 自动八点 ESDF 扫描 | [代码说明](modules/ur7e_auto_esdf.md) |
| `ur7e_gazebo_esdf_sage` | 预扫描冻结底图与分段 SAGE 闭环 | [代码说明](modules/ur7e_gazebo_esdf_sage.md) |
| `ur7e_gazebo_esdf_sage_continuous` | 连续最终目标优化与异步短轨迹替换 | [代码说明](modules/ur7e_gazebo_esdf_sage_continuous.md) |

## 覆盖范围和证据口径

分包文档包含运行源码、launch、可执行包装器、测试/示例源码、主要配置和构建清单。Python 类和函数位置、ROS 接口与命令行参数从源码静态提取；文件职责和执行流程结合实现整理。符号行号对应生成文档时的源码，后续修改后应重新核对。

`validation/`、`report/`、`artifacts/` 中的数据、截图、日志、历史报告和报告生成脚本按用途归档，不逐条复述实验观测。`build/`、`install/`、`log/` 是派生产物，不作为实现依据。第三方算法依赖只说明仓库内调用边界，不能据此还原 wheel 或外部 `storm_kit` 的全部内部实现。

[分析清单](analysis_manifest.json) 记录分包文件数与行数。这里的代码文件数按本次文档范围统计，包括测试和脚本，不等于 Git 跟踪文件总数。

早期基础模块文档完成的是静态代码分析和文档核查；2026-10-27 原生专题已执行 CUDA、Gazebo 和集成测试，详见接入报告，尚无物理机器人测试。历史 README 中“已通过”“已完成”属于原报告结论，不代表本次验证。ESDF 专题的独立来源、压缩包比较和校验记录见 [esdf_analysis/](esdf_analysis/EVIDENCE.md)，不混入较早的模块文档分析清单。
