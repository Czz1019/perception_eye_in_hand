# 原生 C++ ESDF 接入、同条件仿真对照与修改说明

> **sparse-v2 当前状态：** 已修复测试工具的 Thrust 链接错误及构建过期检查，生产目标和可选工具均已编译、链接、安装成功；**未执行工具测试，未启动新版感知或仿真**。下一步按 [构建修复与仿真操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 启动应用，再查看 Topic。算法/接口说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧性能和截图不能作为 v2 验收结果。

日期：2026-10-27。操作入口：[完整操作手册](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)。本文记录本轮已执行的构建、CUDA 几何验证和 Gazebo 对照，取代早期设计文档中“原生模块尚未接入”的状态说明。

## 1. 完成情况与结论

新增 ROS 包 `ur7e_nvblox_native`，直接继承用户指定的 workspaces 官方 `nvblox::NvbloxNode`，链接已有 `nvblox_ros` C++ 库。图像转换、图像时刻 TF、TSDF 融合、三维 ESDF 传播、地图服务和地图层流式发布来自官方实现；本仓库补充机器人应用的输入约束、调度、版本管理、统计和有界体素导出。运行路径不经过 Torch 查询接口。

在 **3 cm 体素、640×360 深度输入、2×2×1.5 m 导出范围、±0.15 m 显示距离带** 下，与当前已改进的 Python/Torch 版顺序对照，静止和移动各持续约 60 秒：

| 后端 | 阶段 | 融合 / ESDF / 导出 Hz（内部计数） | 实际点云接收 Hz | 输入到点云延迟 P50 / P95 / P99，ms |
| --- | --- | --- | --- | --- |
| Python/Torch | 静止 | 17.70 / 17.70 / 17.70 | 17.71 | 13.55 / 17.75 / 22.09 |
| 原生 C++ | 静止 | 18.62 / 18.62 / 18.62 | 18.62 | 2.43 / 4.18 / 5.62 |
| Python/Torch | 腕部运动 | 17.25 / 17.25 / 17.25 | 17.15 | 15.61 / 19.29 / 24.22 |
| 原生 C++ | 腕部运动 | 17.77 / 17.77 / 17.77 | 17.79 | 2.71 / 4.61 / 5.21 |

中位延迟约降低 **5.6～5.8 倍**。两组静止快照都是 9,130 个点，体素中心和距离逐点一致。本轮主要证明相同精度下延迟和导出路径改善；**没有证明当前 Gazebo 场景稳定提供 30 Hz 输入，也没有证明实机已达 30 Hz**。两次顺序运行的上游交付频率略有差异，不能将输出 Hz 的小幅差异全部归因于算法。

![同精度实际点云对照](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/esdf_comparison.png)

![实测频率与延迟](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/performance.png)

[原生实时序列 GIF](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/native_live.gif) 是实际接收点云的每 0.5 秒采样预览，不是 30 FPS 录像；当时 mapper 的实际更新约 18 Hz。图中按有符号距离统一着色，与 RViz RGB 配色不同。静止场景不应为了“看到变化”重复生成不同的几何数据。

## 2. 官方来源与集成边界

参考目录：`/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox`，提交 `bc181942202b24fd47b2f3db8f77c3c82d484566`，ROS 包版本 3.2.5。本地参考 YAML 存在用户修改。本轮以此工作区为实现依赖，没有拿其他 `/home/czz/nvblox_ws` 包代替。

| 官方代码 | 本次如何使用 |
| --- | --- |
| `nvblox_ros/src/lib/nvblox_node.cpp` | 继承 NvbloxNode，复用图像队列、图像时刻 TF、MultiMapper、服务任务和图层发布 |
| `nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu` | 对照其 observed / is_inside / 平方体素距离转米语义和原生块读取方式 |
| nvblox `layer_to_3d_grid_impl.cuh` | 直接调用 `voxelLayerToDenseVoxelGridInAABBAsync`，按块读取层，不调用任意点 Torch 查询 |
| `nvblox_rviz_plugin` | 保留可选配置；本机缺少其要求的 GLSL150 资源，默认不加载此插件 |

这不是重新实现 ESDF 传播算法，也不是运行未经改动的官方 standalone 节点。适配器修改了调度、输入约束和导出方式，因此应称为“复用官方 C++ 模块的仓库适配器”。未完成无修改官方节点在同一输入下的独立性能基线，不把适配器初版结果冒充该基线。

## 3. 文件、职责与解决的问题

| 文件 | 修改内容 | 解决的问题 / 结果 |
| --- | --- | --- |
| [native_mapper.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_mapper.cpp) | NvbloxNode 子类，图像时刻约束、steady clock 调度、实际完成计时、地图版本和 epoch | 仿真时钟低频限制、旧图重复导出、指标口径混淆 |
| [block_export.cu](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.cu) / [头文件](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.h) | 官方块到规则网格模板 + CUDA 有效体素距离带压缩；缓存复用 | 避免构造/传输任意点查询张量及大量未知点 CPU 读回 |
| [native_geometry_test.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_geometry_test.cpp) | CUDA 平面、平移位姿、官方转换器逐元素比较、未知值和清空验证 | 防止优化引入坐标错位、距离符号或旧缓存错误 |
| [gazebo.yaml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo.yaml) | 显式配置 3 cm、3D、TF、积分/ESDF 参数和 ROI | 避免仅对齐 voxel_size 却使用不同权重、距离种子与 ESDF 最大距离 |
| [python_comparison.yaml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/python_comparison.yaml) | Python 同输入、同 native 体素中心导出配置 | 对照时不二次降采样，不用独立 5 cm 查询网格 |
| [depth_adapter.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/depth_adapter.py) | 公共深度单位转换、有限值/范围处理、图像和内参同步缩放 | 两个 mapper 接受同一种图像和标定契约 |
| [run_gazebo_validation.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py) | 独立观察器、外部后端、共同延迟定义、TF/新版本/生命周期断言、NPZ 采样 | 避免 Python 与观察器共进程造成不公平的 GIL 干扰 |
| [monitor.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/monitor.py) | 支持 native 的 input_queue_seen，分别打印融合/ESDF/导出/接收 | 不把 native 未提供的 depth_received=0 当作没有输入 |
| 根目录四个 native/common 脚本 | 隔离 overlay、匹配 CUDA/GXF 库、启动共同输入、原生节点、RViz | 已安装依赖能被当前节点正确找到，无需盲目安装不同版本 |

新增 CMake 显式查找 CUDA、glog、gflags、rosbag2_cpp，补齐官方导出库所需的 imported targets；CUDA 编译启用 relaxed constexpr。运行脚本从对应官方可执行文件的 `ldd` 解析已安装 GXF 依赖目录，避免只 source overlay 后库不可见。代码和依赖来源见 [THIRD_PARTY.md](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/THIRD_PARTY.md)。本机现有依赖足够，本轮没有安装新系统包或替换 NVIDIA 算法库。

## 4. 查询为什么慢，原生导出改变了什么

原来的自定义稠密查询位置由独立 `grid_resolution_m` 决定。仅改变地图 `voxel_size` 不会改变查询位置的数量，因此 RViz 点数和查询成本未必跟着变。未知区域也会进入查询，若全部编码成灰点发布，CPU 序列化、DDS 带宽和 RViz 渲染会额外消耗时间。Python 已改进版的 `native` 模式虽解决采样精度关联，仍然经过 Torch 任意点查询。

本次 C++ 使用地图自身的原生体素中心，在半开 ROI 中共 217,800 个候选单元。GPU 原生层读取之后，仅把 `observed && abs(distance) <= 0.15` 的点压缩后读回，默认 RViz 不接收未知点。

需要准确限定：本次 exporter **仍需读取 ROI 内 217,800 个规则网格位置**，并非完全不访问未知位置，也不是只遍历非空块的极致稀疏导出。融合仍要积分视线内自由空间，ESDF 仍需在已观测自由空间中传播距离，不能把所有“未占用体素”删掉。减少的是不必要的查询接口和导出/显示开销；未知空间也没有被伪造成自由空间。

最后一份滚动窗口中，完成 GPU 读回的 Python `query_readback` P50 约 1.009 ms，C++ `block_export_readback` P50 约 0.110 ms。阶段边界和实现步骤不同，这只是导出路径对照，不是相同 CUDA 内核的速度比。端到端的同一观察器指标更能反映应用收益；总延迟改善不能全部归因于查询内核。

原生模式下点数仍随体素精度、观测范围、距离带和场景变化；不会保证固定倍数。体素变细时，固定体积候选数量约按 `1 / voxel_size³` 增加，因此在相同范围内不能无限细化还要求恒定性能。

## 5. 另一个实测瓶颈：ROS 时钟与调度

该 Gazebo 场景的 `/clock` 实测约 **9.99 Hz**。参考 ROS 实现多项周期任务用 ROS `now()` 判断间隔，直接移植后，即使配置 30 Hz，也可能被稀疏到达的仿真时钟锁在约 10 Hz。初版适配器在相同场景确实测到约 9.9～10 Hz；记录为 `native_003_initial.json`，它不是无修改官方节点的完整基线。

最终适配器：

1. 用 steady clock 调度 ESDF、导出和图层任务；保留 ROS 图像时间和原始 TF 时间，不改输出 header 为 now。
2. 仅在有新融合图像时更新 ESDF；更新预算最多积累两帧，不靠积压补发旧地图。
3. 官方深度积分的图像时间节流上限设为 `integrate_depth_rate_hz: 1000.0`，避免 30 Hz 设定与量化后的 33 ms 图像间隔比较造成隔帧丢弃。它是内部接收上限，**不代表相机或 ESDF 是 1000 Hz**；输入目标仍为 30 Hz，ESDF/导出目标仍为 30 Hz。
4. 队列长度 2，拒绝时序倒退、零时间戳、超过 0.5 秒帧龄及错误 optical frame。ROS 时间倒退触发清图。

完整图像时刻 TF 仍由机器人关节状态、robot_state_publisher 和已配置手眼外参提供，没有增加相机里程计或重新估计机械臂 pose。

## 6. 对照控制条件与指标定义

| 条件 | 两组共同设置 |
| --- | --- |
| 设备 | RTX 4060 Laptop 8 GB，NVIDIA 580.178.04；本机 ROS 2 Humble / Gazebo |
| 场景 | 同一持续运行的 Gazebo，独立 ROS domain 127、Cyclone DDS、本机通信 |
| 原图 → 实际积分输入 | 1280×720 → 公共 stride=2 → 640×360 float32 米；两组均不再缩小 |
| 公共输入有效范围 | 0.2～4.0 m；CameraInfo K/P 同步缩放，header 和 optical frame 不变 |
| 地图 / 导出精度 | 0.03 m / 原生 0.03 m 体素中心；相同半开 ROI |
| ROI | 中心 [0,0,0.5] m，大小 [2,2,1.5] m，base_link；实际体素中心数量 217,800 |
| TSDF 实际参数 | 最大积分距离 7 m、截断 4 vox、inverse_square、max weight 5、invalid depth decay -1 |
| ESDF 实际参数 | min weight 0.0001、max site distance 1 vox、最大距离 2 m、3D |
| 显示 | 只订阅 observed 距离带 ±0.15 m；性能采样时关闭 RViz、Mesh、完整网格订阅 |
| 地图增长 | static_tsdf，禁用衰减/半径清图；两组相同，ROI 是导出范围，不是融合裁剪范围 |
| 测量进程 | mapper 独立进程 + 同一脚本的外部观察器；顺序运行，避免同时争抢 GPU |
| 运动 | 同一观察姿态，wrist_1 小幅 ±0.06 rad 运动；之后返回观察姿态 |

7 m 来自 Python 底层实际默认参数，而不是它外层 YAML 的 4 m 过滤值。两组有效输入已共同裁剪到 4 m。Python wheel 与官方 ROS 使用不同构建的 nvblox 核心，虽已显式对齐参数，仍不是仅切换编程语言的严格单变量实验。

“输入到点云延迟”是在**观察器接收到公共 Image** 时记 steady 时间，按保留的图像 header stamp 匹配点云接收时刻。包含 mapper 输入传输、排队、融合、ESDF、导出、发布和观察器接收等待，不包含相机曝光、原始图到公共预处理之前的开销或执行器响应。BestEffort 下观察器会漏帧，只计算成功配对的消息：

| 后端 | 静止配对 / 点云数 | 移动配对 / 点云数 |
| --- | --- | --- |
| Python | 1,018 / 1,062 | 980 / 1,030 |
| C++ | 1,086 / 1,118 | 1,034 / 1,068 |

内部阶段时延是最近 512 次的滚动统计，公共输入到输出指标使用各阶段整段配对数据。`input_queue_seen` 是处理侧首次见到队列项的计数，不是原始 DDS 接收总数。观察器图像接收 Hz 与 mapper 内部计数可以因 BestEffort 丢帧不同。

## 7. 几何、接口与生命周期验收

- 两份最终主对照 JSON 均 `PASS`，输出图像 stamp 严格递增，没有重复旧 stamp 充当新更新。
- C++ 做了 2,351 次 TF 姿态核验，最大矩阵误差约 3.74×10⁻⁸；Python 2,247 次，约 3.00×10⁻⁸。
- 静止快照原生中心交集为 9,130，两组所有相交点距离差 P50/P95/max 均为 0。它是本场景结果一致性，不能代替真实世界几何精度或所有场景正确性证明。
- CUDA 几何测试通过：`official_converter_max_error=0`；已观测平面单元 2,800、未知单元 33,924。该单元测试使用 5 cm 人工平面，不与主对照 3 cm 混淆。
- 已有 Python CUDA/采样/发布版本测试共 13 项通过；有一条 requests 版本兼容 warning，无测试失败。
- 去掉唯一点云消费者后 C++ 仍更新 ESDF 15 次，导出 0 次；Python 更新 20 次，导出 0 次。重订阅并 reset 后 epoch=2、恢复新数据。
- 继承的 `get_esdf_and_gradient` 服务成功返回 3 cm 三维网格；远离地图的测试 AABB 全部为 -1000。AABB 与体素边界相交语义使 0.09 m 范围实际覆盖 4×4×4 个体素，并不保证正好三格。
- `save_map` 成功保存 1,507,328 字节的 `.nvblx` 文件。官方序列化器提示 MeshBlock 不是支持的序列化层，因此此文件不是 RViz Mesh 快照；需单独使用对应 Mesh 输出接口保存表面。
- 完整 `/ur7e_nvblox/esdf_3d` 实测 217,800 个点，其中未知 205,762；此时是后续独立功能检查，不是静止主对照快照，计数不能直接拼入主对照表。

本机默认 RViz 点云界面已成功显示，见下面窗口自身截图。加载官方 Mesh 插件会因缺失 `rviz/glsl150/box.geom` 抛异常，即使配置中禁用也可能在插件构造时触发；默认配置因此完全移除该插件条目，另存可选配置。另一次显示进程曾发生无日志段错误；后续调试启动点云配置已成功显示，长时间 GUI 稳定性仍需继续观察，不将其归因于 ESDF 算法。

![真实 RViz 窗口截图，截图阶段不属于性能采样窗口](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/rviz_live.png)

## 8. 仍需注意的应用边界

1. 本轮是仿真验证。物理相机、真实手眼标定、机械臂运动时 TF 延迟、反光/无效深度和自遮挡未做实机验收。
2. 未知不是自由：C++ 完整点云用 `1e6`，继承的距离服务默认 `-1000`，Python 完整点云目前用 `100`。下游必须按接口分别识别，不能统一按正负号判断障碍。
3. 当前继承服务虽然叫 esdf_and_gradient，安装的转换实现实际输出 x/y/z 三维**距离数组**，没有额外的梯度通道；规划器需要梯度时必须另行验证计算契约。
4. `/esdf_3d` 是完整原生中心点云；它不等价于旧 5 cm 固定查询网格的尺寸、顺序或未知值契约。当前没有把原有 SAGE 消费者直接改接到新网格，不能直接宣称规划闭环已兼容。
5. static_tsdf 长期累积地图，导出 ROI 不限制所有 TSDF/ESDF 分配；长时间、大范围扫描的显存增长还需单独测试。默认未启用自体过滤、动态目标清除或运动障碍安全策略。
6. 导出与地图操作串行放在官方 processing callback group，避免同时读写同一地图；不是无锁快照。完整网格、地图保存、Mesh 可增加处理延迟。
7. 30 Hz 是配置目标。只有输入、TF、GPU 与消费者都满足预算时才有可能达到；`topic hz`、RViz FPS 及唯一 stamp 数各有不同含义。

## 9. 结果和复现资料

主数据：[C++ 原始 JSON](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/native_003.json)、[Python 原始 JSON](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/python_003.json)、[几何比较](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/geometry_comparison.json)、[服务验证](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/service_validation.json)。同目录 NPZ 保留实际静止、运动快照和时间序列；[绘图脚本](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/render_results.py) 可重绘。

`native_003_initial.*` 是时钟调度修正前的适配器阶段；`python_003_inprocess.*` 是早期共进程观察阶段，均只作排查记录，不进入最终主对照表。构建、CUDA/回归测试、时钟、实际参数、服务和监控日志归档于 `artifacts/native_cpp_20261027/evidence/`；源码哈希见同目录上一级的 `source_manifest.json`。
