# 仓库各感知实现的差异、优劣与复用建议

分析日期：2026-11-04。依据当前源码，而不是将最初版本的不足套用到已经修改过的版本。本文只分析和整理文档，没有更改功能代码或执行新的仿真。

## 1. 比较对象及层级

这些目录并非六个完全独立的 ESDF 算法：基础后端、ROS 运行节点、规划适配层是不同层级。多数路径最终都调用 nvblox；主要区别在输入、调度、导出、未知语义和应用接口。

| 对象 | 代码位置 | 层级 |
| --- | --- | --- |
| realtime_nvblox | [nvblox_backend.py](/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox/nvblox_backend.py) | nvblox_torch 后端封装，不单独等同于完整机器人应用 |
| realtime_nvblox_2 | [nvblox_backend.py](/home/czz/perception_eye_in_hand/src/realtime_nvblox_2/realtime_nvblox/nvblox_backend.py) | 基础库变体，增加原生体素候选的稀疏导出等逻辑 |
| ur7e_nvblox | [node.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py)、[esdf.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/esdf.py) | 机械臂 TF 驱动的 Python ROS mapper，复用基础后端 |
| ur7e_nvblox_native | [native_mapper.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_mapper.cpp)、[block_export.cu](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.cu) | 官方 ROS 节点的较完整 C++ 项目适配 |
| ur7e_perception_planning | [mapper.py](/home/czz/perception_eye_in_hand/src/ur7e_perception_planning/ur7e_perception_planning_core/mapper.py) | 继承 Python TfMapper 的规划地图适配层 |
| nvblox_in_hand | [mapper_node.cpp](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp) | 新独立工程，官方 ROS C++ 薄适配及场景组织 |

workspaces 中官方 ROS 实现是新旧 C++ 路径的参考和默认依赖来源，不应与项目适配节点混为同一套源码。新工程 upstream 是源码快照，默认构建仍复用 workspaces 的已安装库。

realtime_nvblox_2 内部包仍使用 realtime_nvblox 名称，不能只凭目录名判断 Python 实际导入哪份代码；部署时还要核对解释器搜索路径和安装来源。

## 2. 输入、位姿与调度对比

| 实现 | 输入与位姿 | 调度特点 | 优点 | 限制 |
| --- | --- | --- | --- | --- |
| 基础 Python 后端 | 接收调用方提供的深度、标定和位姿，将数据上传 Torch/CUDA | 单个 RLock 保护 mapper | 接口简单、容易离线复用和实验 | 不能独立证明调用方位姿同步正确；多线程访问仍串行 |
| ur7e_nvblox | CameraInfo + 图像时间 TF，拒绝零时间、过期和乱序图像；支持 depth_stride | LatestSlot 保留最新输入，融合/导出/发布各有线程 | 显式帧龄控制；避免无界输入积压；便于 Python 参数和策略修改 | 导出和融合共享后端锁，耗时查询仍会阻塞融合；默认 stride 会改变输入精度 |
| ur7e_nvblox_native | 官方输入链路外增加 frame/标定尺寸、正焦距、帧龄和顺序检查 | 保留官方 tick，ESDF 只在新深度之后推进；带版本和 GPU 事件轮询 | 诊断、有效更新计数和项目接口较完整 | 自定义逻辑较多，与官方版本升级的适配成本更高 |
| 规划 Mapper | 图像时刻 TF、机器人状态、自过滤 | 最新帧处理加地图查询和保守场计算 | 输入直接服务机器人规划约束 | 每轮不只是融合，不能用同等 Hz 要求与纯 mapper 简单比较 |
| 新 nvblox_in_hand | 原始深度/颜色 + 官方图像时间 TF | 官方队列与 tick，薄适配计数 | 路径清晰，保留官方 RGB、Mesh、层和服务 | 默认队列无独立严格帧龄上限；新适配没有旧原生节点全部校验/诊断 |

Python 节点默认值、父工程 launch 覆盖值和新工程默认值并不相同。例如当前 TfMapper 类默认 depth_stride=2、体素 5 cm、最大深度 4 m；新工程默认原始输入、2 cm、3 m。不能把这些默认运行的画面或频率直接作为同条件优劣排名。

## 3. ESDF 更新与导出方式是核心差别

### A. realtime_nvblox：调用核心 + 规则查询

NvbloxBackend 通过 nvblox_torch Mapper 完成 add_depth_frame、update_esdf 和 query_layer。prepare_esdf_grid 预分配规则采样缓冲，query_esdf_grid 在 GPU 查询并读回 NumPy。

优点是查询布局固定、适合明确 ROI 的下游接口和批量计算；缺点是导出成本与采样数量相关，包含未知点的大 ROI 会产生大量无效查询和数据传输。体素尺寸与查询采样间隔属于不同参数，图中点数未必随底层 voxel_size 同步变化。

这里生成规则查询点是在读取原生 ESDF，不是通过给查询点赋值来替代 ESDF 积分器。

### B. realtime_nvblox_2：从已分配 TSDF 取候选，再查询 ESDF

其 observed_esdf_numpy 读取 TSDF 块视图，用正权重筛选候选体素中心，再查询真正的 ESDF，并剔除 unknown 哨兵值。分批处理后只读回选出的结果。

优点是避免全世界大小的规则点阵，采样位置与原生体素尺度关联；缺点是受到 Python 绑定能力限制，仍需候选整理、ESDF 查询、张量临时数据和同步。TSDF 正权重只是候选，不等同于 ESDF observed；代码仍需查询结果验证。

### C. ur7e_nvblox：三种导出模式及线程隔离

当前 native 模式以 voxel_size 在有限 ROI 生成体素中心查询，dense 使用独立 grid_resolution，sparse 使用分配块候选导出。native 这个名字不表示已经直接访问 C++ ESDF layer。

export_loop 检查订阅者、地图版本和帧龄；只有已观测显示需求时，query_prepared_observed 会先在 GPU 过滤距离和 unknown，再读回。完整网格需求仍会读取完整查询结果。

这些比早期每帧强制查询有改进，但查询还在后端锁内。拆成线程可以隔离消息构造/发布，却不代表融合和地图查询已经能安全并发。

### D. ur7e_nvblox_native：原生块 + ROI + GPU 筛选

BlockExporter 先选择与 ROI 相交的已分配 ESDF 块，再在 CUDA gather 中检查 observed、有限距离、inside 和距离带；批次最大 512 块，紧凑输出 occupied/band/distance 的请求组合。

优点是直接使用原生 ESDF 的状态，避免规则 ROI 的所有点查询，CPU 主要接收筛选结果；还提供 allocated/selected blocks 等诊断。缺点是仍遍历相关块，并有每批计数同步与结果读回，不是真正只更新变化块的异步导出，也不能仅凭 CUDA 实现就保证一定更快。

### E. 新 nvblox_in_hand：原生全层读回 + CPU 筛选

InHandMapper 调用官方 processEsdf 后，若有订阅且满足显示频率，则 serialize(layer, getAllBlockIndices())。序列化完成同步后，CPU 遍历全部导出体素，筛选 observed 和 12 cm 距离带，构造 CUBE_LIST。

优点是实现短、易核对官方 observed/inside 语义，没有固定大 ROI 查询点阵；缺点是先读回后筛选，成本可能随全部分配块增长。默认最多 1 Hz、无订阅不执行只能减少调用次数，不能消除单次全层工作。

与旧原生模块相比，这是可维护性更简单，但导出效率和诊断能力不一定更优的取舍。

## 4. 规划感知为何更重，但不能简单删掉

PlanningMapper 除原生融合外，还执行机器人自身深度过滤、固定布局距离查询、射线自由空间证据更新、保守距离场生成，以及 PlanningMap/Snapshot 发布。

其原生距离与 planning_distance 并不等价：规划场还考虑未知空间约束及自由来源。某些经确认的初始净空有独立来源标记，不应与直接观测自由混同。

优点是能连接既有 SAGE、静态地图会话及机器人碰撞约束；缺点是每轮融合线程中包含查询和保守场计算，容易将应用层开销叠加到感知时延。优化方向是把有版本的一致规划快照按需生成，而非删去未知约束来追求更高频率。

新工程尚未实现 PlanningMap 发布，也没有调用旧 robot_depth_mask。不能因 Mesh/ESDF 能显示就认为已具备原规划模块同等功能。

## 5. 对两个已知问题的影响

| 问题 | 各路径共同风险 | 差异与优化判断 |
| --- | --- | --- |
| 墙角 unknown | 遮挡、无效深度、范围、低权重 | Python dense 会明确保留查询位置；稀疏/距离带显示会省略未知及部分自由空间，画面差异不等于地图算法不同 |
| 相机运动重影 | 图像实际采样时刻与 TF 不一致、外参误差、深度异常 | 图像时间查询 TF 并不能保证图像时间戳本身正确；各路径都需验证采集端 |
| 机器人历史表面 | 机械臂自身进入静态 TSDF | 规划 Mapper 已有自过滤可参考；新工程需要按同一图像时刻接入并验证 |
| 显示拖尾 | 队列、同步读取、传输和消费者绘制 | Python 共享锁、旧原生批次同步、新适配全层读回是不同瓶颈，需分段测量 |
| 旧动态障碍残留 | 静态 TSDF 历史保留及缺少新的有效自由观测 | 需证据清除/动态策略，不能将无效深度当作自由；全局强衰减又会增加 unknown |

## 6. 当前性能证据能说明什么

唯一在本轮上下文中完成的简单同机实测是新 nvblox_in_hand：2 cm、3 m、初始姿态、无 RViz/Mesh 消费者。基础 ESDF 调用约 5.09 Hz；订阅三维显示后约 4.76 Hz，深度融合从 16.69 降到 11.83 Hz，附加显示路径样本均值约 106 ms。

这是导出路径存在明显负载的证据，但不是六套实现的公平排名。106 ms 包括 CUDA 同步等待、序列化、CPU 遍历和消息发布，不全是纯拷贝时间。旧原生 GPU 导出在相同输入下能提升多少，尚无本次对照结果。

## 7. 推荐保留和复用什么

1. 以新工程的独立目录、官方 ROS 调度、RGB/Mesh/层/服务组织为主线，减少重复启动入口。
2. 借鉴旧原生模块的标定/帧龄检查、版本控制和非阻塞 GPU 事件计时；先明确计数和时间语义，再迁移。
3. 对新工程显示路径，优先试验旧 BlockExporter 的 ROI/GPU 筛选思想；在保留相同显示范围和完整性条件下比较，不能通过缩小 ROI 冒充算法加速。
4. 借鉴 Python 最新帧策略，补充新工程有界帧龄和丢帧原因诊断；勿把最新 TF 当成最新帧策略。
5. 从规划模块提取机械臂自过滤能力，同时保留遮挡未知语义；后续建立独立的 PlanningMap/Snapshot 适配模块。
6. 对普通三维可视化使用增量块/受控距离带；大范围稠密查询保留给明确需要它的规划请求，避免成为每帧必经步骤。

不建议再维护一套独立 TSDF/ESDF 数学实现。优先整合已存在的适配能力，并用统一输入验证精度、覆盖、延迟和频率。

## 8. 下一次公平比较

使用相同录制的 RGB-D 与机械臂 TF，统一体素、分辨率、深度范围、截断距离、权重和显示消费者。分别运行纯核心、已观测距离带、Mesh/RViz、规划快照四种负载；记录输入接收/限频/融合计数、队列帧龄、GPU 完成、导出分段耗时、变化/总块数、输出字节及质量指标。

已有腕部小幅转动用于运动稳定性，地图逐渐增长用于规模测试。每项重复运行，不仅比较 Hz，还比较相同几何的表面误差、unknown 覆盖和残影。CPU、GPU、队列与显示时间应分别报告。

相关资料：[实测记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/RESULTS.md)、[优化路线](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/REALTIME_TEST_INTERPRETATION_AND_OPTIMIZATION.md)、[整体仓库说明](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/REPOSITORY_CODE_GUIDE_20261103.md)。
