# 原仓库感知结构、具体问题与本周改造对照

分析范围：2026-10-26 至 2026-10-29。本节“原仓库”以本周修改前的已提交代码 `3ac915bb9adde9f2b9a3717d7332d29501f8eca2` 为基线，不把本周新加的 ur7e_nvblox_native、nvblox_in_hand 当作最初已有内容。这也不是将 Git 历史最早、尚无完整感知的提交作为分析对象。

关键旧代码已从 Git 提取为[只读分析快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/baseline.json)，以免工作区源码后续修改后，旧实现和新实现混淆。

![原仓库感知链路与本周改造对照](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/images/original_vs_weekly_20261029.png)

图为源码结构与工作进展示意，不是仿真重建效果截图。

## 1. 原仓库的整体结构

原 src 下有 13 个功能目录，主要分为四组：

| 分组 | 功能目录 | 职责 |
| --- | --- | --- |
| 场景/机器人输入 | ur7e_d435_gazebo | UR7e、D435、障碍物、控制器、joint_states、TF |
| 感知基础 | realtime_nvblox、realtime_nvblox_2 | Python 运行框架、nvblox_torch 封装、查询、统计和 ROS 发布等 |
| 感知/重建 | ur7e_nvblox、ur7e_reconstruction | 机械臂 TF 深度融合、ESDF 查询显示，以及独立表面重建应用 |
| 任务与应用 | ur7e_perception_planning、ur7e_sage_planner、ur7e_real_static、ur7e_real_reconstruction、ur7e_auto_reconstruction、ur7e_auto_esdf、ur7e_gazebo_esdf_sage、ur7e_gazebo_esdf_sage_continuous | 地图适配、规划、静态会话、实机/自动采集及连续应用 |

这不是一个启动文件会全部运行的程序。原仓库已经有表面重建和自动采集等能力；不能因为 ur7e_nvblox 的 ESDF 显示不完整，就说整个仓库没有 Mesh 或颜色功能。

realtime_nvblox 与 realtime_nvblox_2 的 package.xml 均声明包名 realtime_nvblox。这种副本组织增加了包发现、构建范围和实际代码来源的排查成本，但不能仅凭同名就断言每次运行都发生冲突。

## 2. 原先主要感知路径如何工作

本次重点分析 `ur7e_nvblox/TfMapper` 的默认 dense 路径：

```text
Gazebo D435 深度 + CameraInfo
                  │
机械臂 joint_states → robot_state_publisher → 图像时间 TF
                  │
                  ▼
Python TfMapper：校验 → stride 深度采样 → latest-only 输入槽
                  ▼
nvblox_torch：深度融合 → 原生 ESDF 更新
                  ▼
固定空间查询点 → ESDF 批量查询 → GPU 到 CPU 读回
                  ▼
latest-only 结果槽 → 发布线程 → ESDF PointCloud2 → RViz
```

**关键事实：原来的地图底层已经是原生 nvblox，固定网格主要用于查询和输出。** 问题不是“完全没有计算真正 ESDF”，而是后处理路径与地图分辨率、显示语义和调度之间存在不匹配。

原实现也已经做了图像时刻 TF、旧帧检查、时钟回退处理、latest-only 输入/结果槽和单独发布线程。这些应保留为原代码已有优点，不能把它们全部写成本周新增。

## 3. 具体问题一：地图体素和显示采样间距是两个参数

旧默认配置为：

```text
voxel_size_m      = 0.03 m
esdf_output_mode  = dense
查询间距          = 0.05 m
查询范围          = 2.0 × 2.0 × 1.5 m
```

`prepare_fixed_grid()` 在固定中心周围准备查询坐标；点坐标来自 grid_resolution_m，而非 voxel_size_m。

按基线 grid_offsets 的端点生成规则，点数为：

```text
(2.0 / 0.05 + 1) × (2.0 / 0.05 + 1) × (1.5 / 0.05 + 1)
= 41 × 41 × 31
= 52,111 个查询点
```

因此，只修改底层 voxel_size，不改查询间距和范围时，这个 dense 输出的采样点数基本不变；底层地图实际分辨率和计算量则仍可能变化。这直接解释了“体素改了，屏幕点数却没明显跟着变化”的一种具体代码原因。

固定查询可用于规划快照，并非一概错误；但直接把它当作原生体素可视化，会造成理解偏差。基线已有 sparse 可选分支，但默认 YAML 选择 dense，不能把“源码有 sparse”当作“实际默认在运行 sparse”。

## 4. 具体问题二：未知点、自由空间与显示对象容易混淆

旧 `esdf_cloud()` 在 full_grid=True 时保留整个查询网格，并将 unknown 标成深灰色。这些坐标原本就是查询点，并不表示对应空间全部已观测或全部占据。

固定网格查询也不等于底层已经为全部未知区域分配了 ESDF 体素。应分别分析查询成本、地图实际分配内存和 RViz 渲染成本；已观测自由空间则本来就具有距离信息，不应一概作为“无用空点”删除。

这里需要同时说明一个边界：旧 RViz 配置默认订阅 esdf_observed，而这个分支会过滤 unknown 和距离带外数据。因此，“全量话题可以产生灰色未知点”有源码依据，但不能只凭显示名称或截图，断定某一次灰色点阵必然来自该分支；仍需核对当时的订阅话题和参数。

ESDF 距离颜色也不是物体 RGB 表面。只看距离点云，无法直接评价颜色纹理和三角网格平整度。原 TfMapper 主要订阅深度，不在这条入口中提供完整颜色融合/Mesh 显示；表面应用位于另一个包。

## 5. 具体问题三：查询和读回仍占用融合工作线程

基线 `_worker()` 中的实际顺序是：

```text
backend.integrate_depth()
→ 到 ESDF 周期时 backend.update_esdf()
→ query_prepared() 或 observed_esdf_numpy()
→ CPU 读回 / known 判断 / 结果整理
→ 才继续处理下一条深度输入
```

query_prepared 中还包含：

```python
output = self.mapper.query_layer(...)
return output.detach().cpu().numpy().reshape(-1)
```

CPU 需要得到结果后才能继续后续逻辑；相关工作位于后端锁和融合工作链中。发布线程虽然独立，解决了部分 DDS 发布阻塞问题，但没有把 ESDF 查询与读回从融合工作线程中分离。

此外，基线只在发布阶段检查订阅数量，工作线程的查询仍会执行。即使没有 RViz 订阅，也可能支付查询/读回成本。

这些是明确的调度和开销问题。它们可能使融合来不及处理所有输入帧，latest-only 槽会覆盖旧帧，而不是简单无限排队。但没有同条件性能剖析数据时，不能进一步断言“绝大多数时间一定都花在查询”或给出具体耗时占比。

## 6. 具体问题四：输入采样影响细节，显示方式又放大了观感差异

旧 depth_stride=2，代码通过 raw[::stride, ::stride] 采样。对于 1280×720 输入，融合尺寸变为 640×360，像素数量为原来的四分之一；细小边缘的观测也会减少。

原代码同步缩放了内参，因此不能仅凭降采样就指控它“没有调整 CameraInfo”。但它确实是与完整分辨率官方路径不同的输入条件。

3 cm 地图体素、5 cm 查询采样、距离点云显示与完整分辨率彩色 Mesh，本来就不能直接按截图平整度作同条件比较。手眼、时间同步、深度噪声和扫描覆盖也会影响结果，不能把所有表面模糊都归因于查询。原场景还声明深度噪声参数，实际影响需要运行数据确认。

## 7. 本周针对这些问题做了什么

| 原问题/需求 | 本周工作 | 当前进展和边界 |
| --- | --- | --- |
| 查询输出和地图计算混在一起理解 | 对照原代码、workspaces 和官方实现，拆分融合、ESDF、查询、发布、显示 | 明确了具体代码路径，不再将点阵观感等同于算法错误 |
| Python 路径后处理与融合耦合 | 在父工程接入原生 C++ 模块，逐步对齐官方 tick、层发布与统计 | 已有实现；不能仅因换 C++ 就宣称性能必然提升 |
| 希望保留官方结构并剔除无关工作流 | 新建独立 nvblox_in_hand，保留机械臂、场景、手眼与官方 ROS/core 模块 | 不改 workspaces 源文件；默认复用其已安装官方依赖 |
| dense 显示精度与原生体素脱钩 | 新工程首选官方增量 Mesh；可选三维视图读取原生已分配/已观测 ESDF 体素 | 没有固定 ROI 查询点阵；附加显示仍有全块读回开销，默认按需限频 |
| 输入细节与表面显示不同 | 新工程直接完整深度输入，保留颜色融合和原生 Mesh 显示 | 默认 2 cm，旧默认 3 cm；这是条件变化，不能直接作为公平效果提升结论 |
| 环境/入口排错困难 | 独立构建目录、overlay 检查、统一通信默认值、扫描预检查 | 旧同名副本没有从父仓库删除，新工程只是隔离使用 |
| 本周运行暴露 RViz 闪退 | 定位 GLSL150 资源缺失，使用官方插件兼容材质并提供标准 Marker 体素显示 | 虚拟显示回归通过；这是本周运行暴露的问题，不追溯成最初仓库性能差的原因 |
| 机械臂状态显示难确认 | 增加只读 joint FK 与同时间 TF 对照，核对重复发布源 | 工具已实现；尚未证明用户实际场景中的姿态异常已被在线定位 |

新工程默认保留 static_tsdf、单 D435、图像时刻机械臂 TF；不启动人物分割、任务识别、VSLAM、多相机和雷达工作流。这里是剔除无关运行链路，不是重写或删除官方共享框架的全部通用类型。

## 8. 哪些结果可以确认

- 新工程场景包和感知包构建、安装成功。
- 13 项 Python 离线测试和 C++ ESDF 显示策略测试通过。
- 独立 ROS 域/虚拟显示测试中，RViz 接收 48 条测试 Mesh，运行约 10 秒后正常退出，未再触发缺失 shader 异常。
- 文档覆盖了代码结构、完整启动、话题、频率、统计、扫描以及与父工程应用的关系。

这些证明结构改造和基础检查取得进展，不证明新默认已经达到 workspaces 的真实重建质量、覆盖率或实时性。测试三角网格不是重建截图，构建成功也不是在线算法验收。

## 9. 接下来怎样与整体工程衔接

第一步，在相同体素、图像分辨率、融合距离、场景、运动输入和显示需求下，记录旧路径、直接官方后端与新薄适配后端的频率/延迟和表面质量。否则无法区分参数变细与实现改善。

第二步，再接父工程 PlanningMap/Snapshot/FrozenMap：明确 unknown 语义、自由空间来源、数组布局、原点、地图版本和新鲜度。Mesh 和 ESDF Marker 不能直接替代规划地图。

第三步，接 SAGE 和执行链路，并重新处理独立 C++ 感知进程与规划进程之间的 GPU 资源竞争。原 Python 进程内锁不会自动约束新 mapper。

本周已经把“原感知链路的具体问题”转化为可审查的独立官方路径和运行工具；整体感知—规划—执行接续及同条件效果验收仍未完成。

## 10. 源码证据与延伸阅读

原基线证据均位于本目录 artifacts/baseline_analysis_20261029/sources，不能拿已修改的工作区同名文件冒充原版：

- [旧 TfMapper](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/sources/src/ur7e_nvblox/ur7e_nvblox/node.py)：融合、查询、读回和发布线程关系。
- [旧 EfficientBackend](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/sources/src/ur7e_nvblox/ur7e_nvblox/esdf.py)：固定网格与 sparse 备选实现。
- [旧默认配置](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/sources/src/ur7e_nvblox/config/gazebo.yaml)：3 cm 地图、5 cm 查询、dense、stride=2。
- [旧点云打包](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/sources/src/realtime_nvblox/realtime_nvblox/ros_publishers.py)：full_grid 保留 unknown 与 observed 过滤。
- [旧 RViz 默认显示](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/sources/src/ur7e_nvblox/rviz/esdf.rviz)：Observed ESDF 所订阅的话题。
- [新工程代码分析](CODE_ANALYSIS.md)、[整体工程衔接分析](WHOLE_PROJECT_RELATIONSHIP.md)、[本周最新验证范围](VALIDATION.md)。
