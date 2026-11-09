# nvblox_in_hand

[迁移部署：完整依赖、库版本、安装、编译和仿真指令](docs/MIGRATION_DEPLOYMENT_20261109.md)。

[2026-11-09：从编译开始的Mesh/3D ESDF仿真、实际截图、融合策略和质量问题说明](docs/BUILD_MESH_ESDF_SIMULATION_TEST_20261109.md)。

[最新：voxel_size=0.02/0.03/0.05m的ESDF点云频率、点数与九轮测试](docs/VOXEL_SIZE_ESDF_POINTS_FREQUENCY_TEST_20261105.md)。

[问题说明：为什么ESDF显示约30Hz，新深度关联输出仍约20Hz](docs/ESDF_REALTIME_PROBLEM_ANALYSIS_20261105.md)。

[最新：三维ESDF输出实时性九轮对比——发布率、新输入关联率与年龄P50/P95](docs/ESDF_REALTIME_OUTPUT_COMPARISON_20261105.md)。

[最新九轮复测：18Hz融合原因、融合门限30/33.4/40实测与修改说明](docs/FUSION_18HZ_MULTIRUN_RETEST_20261105.md)。

[汇总：重构差异、约30Hz的准确含义、话题、融合问题及全部近期测试对照表](docs/PERCEPTION_REFACTOR_FREQUENCY_FUSION_TEST_SUMMARY_20261105.md)。

[表面模糊、串色与物体融合：源码分析及改善优先级（未运行测试）](docs/RECONSTRUCTION_BLUR_COLOR_BLEED_ANALYSIS_20261105.md)。

[最新：抬高到0.82m、左右16cm扫描，真实轨迹、重建与截图](docs/RAISED_LATERAL_RECONSTRUCTION_TEST_20261105.md)。

[扩大范围：相机上方±9.2°扫描重建、三轮对比与实际截图](docs/OVERHEAD_WIDER_SCAN_V3_20261105.md)。

[最新：相机上方±4.6°转动重建测试、前后网格与真实截图](docs/OVERHEAD_ROTATION_RECONSTRUCTION_V2_20261105.md)。

[障碍物上方轻微转动重建测试：实测数据、前后对比与窗口截图](docs/OVERHEAD_RECONSTRUCTION_TEST_20261105.md)。

逐终端执行请看 [三维ESDF仿真命令测试操作文档](docs/ESDF_COMMAND_TEST_GUIDE_20261105.md)，包含观察姿态恢复、显示、频率统计、地图/截图保存及排错。

当前三维点云显示可直接运行 `./run_esdf.sh gazebo_gui:=true`（与 `./run.sh` 互选，不要同时启动）。它显式开启诊断后端和原生三维导出；默认官方组件不发布 `/nvblox_node/esdf_points`。三维距离色带现为 −0.09～0.24m，突出近障碍距离差异，二维仍保持0～1m。新测试及说明见 [三维发布与密集色带测试](docs/ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)。

2026-11-05最新：ESDF门限与诊断显示默认33.4Hz，配色对齐workspaces二维0–1m距离色带。实测官方ESDF调用32.78Hz，诊断点云29.44Hz，但关联新深度时间戳推进约19.75Hz；不能当作30Hz全新地图。[修改、截图、数据与运行命令](docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)。

2026-11-05第一阶段融合限频优化：默认realtime档的深度/颜色限频改为40Hz，传感器仍30Hz。实测深度18.23→29.71Hz、颜色17.96→29.85Hz；ESDF仍约24.58Hz。[修改、对照测试与完整运行说明](docs/FUSION_RATE_MARGIN_OPTIMIZATION_20261105.md)。

基于 `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples` 的单D435眼在手上感知工程。保留perception原机器人、手眼、相机和障碍物场景，以机械臂图像时刻TF提供pose，不启动人物/任务识别、语义分割、VSLAM、多相机或LiDAR工作流。

2026-11-04重构：默认直接加载官方 **nvblox::NvbloxNode组件**，遵循示例的入口、感知、相机、配置和可视化分层。官方核心继续执行TSDF/Color/ESDF/Mesh及地图服务；项目自定义三维导出已降为显式可选诊断，不再把inside-only点集当作完整ESDF。

- [重构依据、调用链、模块分析及尚未完成的优化](docs/WORKSPACES_EXAMPLE_REFACTOR_20261104.md)
- [最新：33.4Hz门限、距离配色及四组仿真对照](docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)
- [第一阶段：融合限频优化及同场景3D ESDF对照测试](docs/FUSION_RATE_MARGIN_OPTIMIZATION_20261105.md)
- [最新：640×480输入下的3D ESDF复测与运行指令](docs/INPUT640_3D_ESDF_TEST_20261104.md)
- [重构后1280×720仿真复测：实际更新率、ESDF内外侧、质量与窗口截图](docs/REFACTORED_EXAMPLE_TEST_20261104.md)
- [完整操作说明](docs/OPERATIONS.md)
- [参数、话题、ESDF输出契约](docs/PARAMETERS_AND_TOPICS.md)
- [此前inside-only问题的代码审计](docs/ESDF_SEMANTICS_AND_SIMPLIFICATION_AUDIT_20261104.md)

## 开始使用

先停止同端口同域的旧场景与mapper。默认启动不会移动机械臂或执行环绕：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./run.sh gazebo_gui:=true
```

默认profile=realtime：3cm体素、640×480仿真输入（2026-11-04本轮调整；上一轮为1280×720）、3m融合范围，传感器目标30Hz，深度/颜色融合限频40Hz，ESDF更新门限33.4Hz，Mesh更新/层发布目标5Hz。目标值不是实际频率，最新实测见上方33.4Hz报告。

默认3d ESDF保存在官方Mapper，RViz看官方彩色Mesh。**本地官方3d模式不输出二维static_esdf_pointcloud，也不发布项目诊断esdf_points/metrics。**按需选择以下独立运行方式：

```bash
# 先停止上一轮mapper，再选择其中一种：
./run.sh esdf_mode:=2d
./run.sh mapper_backend:=diagnostic esdf_view_mode:=observed
```

2d查看官方投影切片；diagnostic查看原生三维已观测内外侧体素，额外增加导出负载。不要同时运行两套mapper。单独RViz使用 `./rviz.sh mesh`、`./rviz.sh 2d` 或 `./rviz.sh diagnostic`。不再加载历史artifact中的方块配置。

## 与workspaces的关系

默认构建本工程src中的场景和适配包，链接workspaces已安装的ROS/CUDA/NITROS依赖。upstream保留官方ROS/core/msg/plugin源码快照；`./build.sh --from-source`提供独立重编路径，使用 `IN_HAND_SOURCE_BUILD=1 ./run.sh`加载，该路径本轮未重新验收。默认构建不等于重编全部官方核心，也不等于完全独立发行。

profile=reference继承参考base的2cm、颜色/ESDF 5Hz及衰减/清图等数值，仍保留本项目机器人适配和颜色开关，不是整个workspaces运行的完全复制。[来源和配置删减记录](provenance/example_refactor_20261104.json)可追踪具体参考文件。

附加数字参数为空时继承运行档/自定义YAML，显式CLI才覆盖；布尔功能开关、ESDF模式、坐标系及单D435/TF约束仍由launch最终决定。

## 验证范围与历史资料

重构阶段完成构建和静态解析。用户后续授权复测后，已完成官方3d、官方2d和诊断3d仿真检查，20项Python检查及C++/GPU检查通过；官方3d实际ESDF调用约24.61Hz、深度融合18.51Hz，几何P95仍约35.29mm，尚未达到30Hz或证明几何更优。[新版实测与边界](docs/REFACTORED_EXAMPLE_TEST_20261104.md)。旧版20字节点云报告不能当作本版接口。

tests中的启动契约及GPU字段断言已在复测中执行；可用 `./test.sh`重复离线检查。旧orbit_scan仍依赖诊断metrics，需显式diagnostic后端，默认不运行扫描。

历史与基础阅读：[源码分析旧版](docs/CODE_ANALYSIS.md)、[项目联系](docs/WHOLE_PROJECT_RELATIONSHIP.md)、[原感知问题及本周修改](docs/ORIGINAL_PERCEPTION_AND_WEEKLY_CHANGES.md)、[未知区和运动残影分析](docs/ESDF_UNKNOWN_AND_MOTION_GHOST_ANALYSIS.md)、[目录来源](docs/ARCHITECTURE.md)、[许可证](THIRD_PARTY.md)。历史默认值与接口以重构报告及当前代码为准。
