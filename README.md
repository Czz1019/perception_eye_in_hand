# perception_eye_in_hand

[扫描启动失败排查与完整仿真测试步骤](/home/czz/perception_eye_in_hand/docs/ORBIT_SCAN_TEST_GUIDE.md)：从构建、启动感知、无运动预检查到扫描、RViz、频率与延迟查询；新增 `--check-only` 和逐项缺失诊断。

[RViz 鼠标交互修复与障碍物组多视角扫描](/home/czz/perception_eye_in_hand/docs/RVIZ_INTERACTION_AND_ORBIT_SCAN_GUIDE.md)：Move Camera、全景/俯视预设，以及 `run_ur7e_orbit_scan.sh` 的离线设计和仿真采集指令。

[重建效果、体素大小与表面不平整分析及优化方向](/home/czz/perception_eye_in_hand/docs/ESDF_SURFACE_QUALITY_ANALYSIS_AND_OPTIMIZATION.md)：当前项目与 workspaces 的配置/显示/采集差异、可能原因及同精度验证方案。

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

> 2026-10-27 统一入口更新：推荐使用本项目 `run_ur7e_perception.sh` 联合启动场景、输入、感知与显示，详见 [项目内统一感知结构与启动说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_INTEGRATED_DESIGN_AND_STARTUP.md)。下文分组件方式保留用于调试；不要与统一入口同时启动。

已完成修改的范围与文件清单见 [perception_eye_in_hand 感知改造完成说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_REPOSITORY_COMPLETED_CHANGES.md)。修改位于本仓库；workspaces 提供参考和第三方库依赖。旧 Python 入口与新 C++ 入口目前分别保留。

UR7e 机械臂眼在手上感知与规划工作区，包含 D435/D435i 仿真、RGB-D 表面重建、nvBlox ESDF、SAGE 规划，以及真机停稳扫描和显式轨迹执行应用。

历史 0.1 版曾完成官方 ROS 原生模块接入及同精度仿真对照（结果不能代替当前版本验收）：

- [原生 C++ 接入和实测报告](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md)：修改、几何验证、实际频率和延迟、原始证据。
- [完整操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)：构建、仿真、手动采集、RViz、监控、对照测试、地图服务和实机准备。

完整中文代码说明从 **[项目文档索引](docs/README.md)** 开始。

- [目录说明](docs/DIRECTORY_GUIDE.md)：根目录、14 个源码目录、配置、测试和历史产物的用途。
- [架构与数据流](docs/ARCHITECTURE.md)：各条运行链路、共享依赖、坐标和距离场契约。
- [关键流程说明](docs/CODE_WALKTHROUGH.md)：采集、融合、冻结、规划和执行的函数级阅读指南。
- [运行入口与现状](docs/ENTRYPOINTS_AND_FINDINGS.md)：脚本入口、环境要求、重复包名、合并冲突及默认配置差异。
- [分包代码说明](docs/README.md#功能包文档)：逐文件职责、类/函数索引、ROS 接口表达式、参数和配置资源。

基础模块文档基于 2026-10-26 静态分析；2026-10-27 原生 ESDF 专题另有实际 CUDA/Gazebo 运动测试，未进行物理机器人测试。历史结果按各报告日期和证据区分。
