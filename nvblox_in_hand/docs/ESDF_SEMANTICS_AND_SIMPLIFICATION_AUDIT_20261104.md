# ESDF内部点原因、workspaces简化范围与当前优化审计

后续版本提示：本文记录本次示例主体重构之前的状态，当前默认已改为官方组件，诊断点云改为24字节并默认observed。新版代码、参数和验证边界见 [workspaces示例重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md)。

日期：2026-11-04。本轮只读取代码和已有来源记录、编写说明；没有构建、运行仿真、移动机械臂或执行测试，没有修改程序或参数。文中的后续方案尚未实施。本报告对前一份“点云优化”报告的输出语义作进一步纠正。

## 1. 结论：当前默认输出确实不满足完整距离场观察需求

ESDF点集中在障碍物内部，首先是当前三维适配器的显式筛选结果。不是把自由空间点变换到了障碍物内部，而是自由空间点根本没有被输出。仅凭这个现象不能诊断坐标变换错误，也不能证明底层距离场正确或错误。

当前默认 `esdf_view_mode=surface`，实际含义是 **已观测且被判定为inside的原生体素中心集合**，不是完整ESDF，不是几何表面点，也不是机器人周围自由空间的避障距离集合。此前采用这个默认值并把“参考workspaces某个筛选条件”作为依据，不能满足“保留workspaces有效感知功能、只剔除冗余任务”的目标。这是适配层语义选择不当，不能用显示更细或点数更少说明功能已等同。

核心TSDF/ESDF仍来自官方ROS迁移库；当前附加三维点云是本项目代码。两者必须分别评价。

## 2. 内部点是怎样被筛选出来的

### 2.1 直接代码链

入口 [mapping.launch.py](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/mapping.launch.py) 默认设置：

```python
mapper_backend='observed_view'
esdf_mode='3d'
esdf_view_mode='surface'
```

[mapper_node.cpp](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp) 将surface转换为 `EsdfPointSelection::kSurface`，然后调用本项目GPU导出器。

[esdf_pointcloud_exporter.cu](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu) 的关键条件是：

```cpp
if (!v.observed) return;
// 非法平方距离另外剔除
if (selection == int(EsdfPointSelection::kSurface) && !v.is_inside) return;
const float d = v.is_inside ? -abs_d : abs_d;
```

因此，inside=false的已观测自由体素即使具有有效的正距离，也会被默认模式丢弃。剩下点的位置仍是原始体素中心，不是最近障碍表面的位置。显示Mesh时，这些点还可能被Mesh遮挡。

### 2.2 inside、site与真实表面不是同一概念

原生 [esdf_integrator.cu](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/nvblox_core/nvblox/src/integrators/esdf_integrator.cu:113) 的TSDF分支定义：

```cpp
isVoxelObserved:     tsdf_voxel.weight >= min_weight
isVoxelInsideObject: tsdf_voxel.distance <= 0.0f
isVoxelNearSurface:  fabsf(tsdf_voxel.distance) <= max_site_distance_m
```

同文件 `updateEsdfVoxelToChanges()` 将inside与非高置信自由空间条件结合，并用 `inside && near_surface` 标记site。新site的 `squared_distance_vox` 被设为0。

这说明：

- inside是融合模型的符号分类，不是相机已经看到整个实体内部，更不是可靠的物体分割真值。
- inside点可能位于观测表面后方的TSDF截断带；不能由点的存在推断整个障碍物内部已被测量。
- site是距离传播的离散种子，也不是Mesh的精确零交叉顶点。
- 当前3 cm体素、4 voxel截断距离对应12 cm尺度；max_site_distance_vox=2对应6 cm近表面判定尺度。它们是参数尺度，不能据此保证每个表面都出现固定厚度。
- 对本地实现，部分inside site的距离会是0或负零。因此“非负距离”不应直接当作自由空间标志，可靠分类还需要observed和inside状态。

若需要真正的重建表面，应该使用原生TSDF零交叉提取的Mesh；单独筛选inside体素或筛选ESDF距离接近0都不等于精确表面提取。

## 3. 为什么这与workspaces看到的效果不同

### 3.1 前一轮参考了不适合充当输出契约的辅助类

workspaces的 [layer_publishing.cpp](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:187) 确有：

```cpp
bool operator()(const EsdfVoxel & voxel) {
  return voxel.is_inside && voxel.observed;
}
```

但本轮在该workspaces的nvblox_ros源码范围搜索 `EsdfVoxelFilter`，只找到类定义与构造函数，未找到调用点。该文件中实际调用的TSDF/Color层筛选包含 `TsdfVoxelFilter`。所以不能说“官方完整ESDF就是只输出inside”，也不能拿这个未接入的辅助类证明本项目复现了用户原来的显示链。

此前确实复制参考了这个条件，但参考对象选择不充分。前一报告中对它来源的说明成立，对它代表完整官方输出的推断不成立。

### 3.2 官方二维切片的实际规则

[NvbloxNode::processEsdf](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/nvblox_node.cpp:767) 先调用 `multi_mapper_->updateEsdf()`，仅在esdf_mode=2d时执行 `sliceAndPublishEsdf()`。

[esdf_slice_conversions.cu](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/conversions/esdf_slice_conversions.cu:54) 把切片中的非unknown值转换为带intensity的点，没有附加“仅inside”的条件。因此可以看到障碍外侧的有效自由空间距离分布。点的z固定为切片高度，是二维投影/切片输出，不能与三维inside集合直接比较。

官方二维建图模式及其高度范围投影，也不自动等价于“从三维地图任意切一张图”。比较时要记录esdf_mode、投影高度和显示话题。

### 3.3 官方服务保留了正负距离，和当前默认显示不同

[SignedDistanceFunctor](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu:28) 对observed体素读取平方距离，对inside加负号，其余保留正距离；unknown返回指定默认值。

`/nvblox_node/get_esdf_and_gradient` 服务按AABB导出原生地图。这是有界数据输出，不是先构造查询网格再反向建立ESDF。稠密栅格作为规划数据格式本身是合理的，问题在于是否反复全图导出、是否保留未知语义、是否把服务查询与核心建图混为一谈。

另一个需要纠正的细节：本地接口虽叫EsdfAndGradients，但所查转换函数实际填充三维标量距离数组及origin/voxel_size，没有显式逐体素xyz梯度通道。下游若需要梯度，应核实并实现相应计算，不能仅凭服务名宣称已经拿到完整梯度。

### 3.4 当前三种模式的功能边界

| 当前模式 | 保留内容 | 缺少的内容/限制 |
| --- | --- | --- |
| surface，默认 | observed且inside且距离合法 | 丢弃全部outside；名称容易被理解为真实表面；不适合作完整避障场 |
| band | observed且绝对距离≤12 cm（可配） | 有外侧近障碍距离，但忽略更远的自由体素，不是完整场 |
| observed | 全部observed且有限合法距离 | 比前两者完整，但没有显式inside/observed字段，缺点不区分unknown与被裁剪区域；距离可能处于上限饱和值 |

当前PointCloud2只含x/y/z、distance、intensity、rgb，没有梯度、置信度、inside标志或独立unknown掩码。distance/intensity还是同一个存储值的两个字段名。因此当前话题应称为“用于诊断的ESDF采样点云”，不能直接当作完整规划地图协议。

## 4. 现在究竟哪些来自workspaces

| 部分 | 当前实现与来源 | 是否重写核心 |
| --- | --- | --- |
| 图像队列、输入转换、CameraInfo、图像时刻TF | 官方NvbloxNode/Transformer及conversions | 否 |
| TSDF/Color融合、ESDF更新、块更新跟踪 | 官方Mapper/MultiMapper/CUDA integrator | 否 |
| Mesh生成、增量层发布、地图读写 | 官方核心与ROS接口 | 否 |
| 原生消息、服务、RViz Mesh插件 | workspaces依赖；upstream保留源码快照 | 否 |
| 机械臂、D435安装与障碍物场景 | perception父项目的独立副本 | 属于项目场景，不是官方示例场景 |
| bringup、mapping、D435入口和参数组合 | 本工程重新编排 | 是适配编排 |
| robot_pose.py | 本工程路径/健康观察节点 | 不生成mapper位姿 |
| 三维PointCloud2、筛选和GPU导出 | 本工程新增 | 是附加数据输出，不是官方原样节点 |

默认 `./build.sh` 构建src中的两个本地包，链接workspaces已安装的官方库。upstream是源码快照，默认不会重编译它；提供 `--from-source` 独立构建路径，但本轮没有执行或验收该路径。

本轮只读核对以下五个快照文件与workspaces同路径文件，字节内容一致：nvblox_node.cpp、transformer.cpp、layer_publishing.cpp、核心esdf_integrator.cu、核心mapper.cpp。这说明这五处源码没有被改写；不代表所有依赖、安装二进制与全部workspaces配置均被逐项复验。

## 5. 已做的简化：有效部分与不足

| 简化内容 | 当前做法 | 保留效果及边界 |
| --- | --- | --- |
| 人物检测、分割、任务识别 | 新入口不启动相关节点，不接mask | 符合静态机械臂重建需求；没有证明与此前static基线相比产生额外加速 |
| 人物/动态分图 | 入口固定static_tsdf | 保留静态重建；运动机器人必须另做自过滤，不能指望静态融合自动清除 |
| LiDAR、ZED、多相机与导航平台 | 新入口单D435、use_lidar=false，不引入其他设备工作流 | 符合目标设备范围 |
| VSLAM与IMU/双红外分流定位 | 使用机械臂TF替代；不启动相应定位链 | 符合pose来源要求；质量取决于时间、手眼、运动学、相机外参 |
| 机器人与场景 | 复制保留父项目配置 | 满足场景沿用，不保证与workspaces原场景观测条件等同 |
| 官方核心结构 | 保留upstream中的ROS/core/messages/plugin模块 | 共享框架内通用dynamic/human/lidar类型仍保留，避免破坏依赖及ABI |
| 运行环境 | 单独src/build/install，提供统一入口 | 隔离父项目；默认仍依赖workspaces安装库，并非完全独立分发包 |

当前还不是“原组织结构几乎完整保留，只裁掉冗余任务入口”的最终形态：核心组织保留了，但示例bringup被重新写成较小的launch，三维输出也自行定义。还保留了NITROS/GXF等官方依赖，并未裁成纯ROS标准消息核心。不能把未启动的工作流与已从共享库彻底删除的代码混称为“全部删除”。

仿真URDF仍有两路1 Hz红外传感器声明，因此“未使用红外定位”也不等于仿真中冗余传感器已全部剔除。这是后续可以核实并处理的输入侧冗余。

## 6. 已做的优化分别解决了什么

| 改动 | 解决的问题 | 没解决的问题 |
| --- | --- | --- |
| 使用官方ROS/CUDA核心替代父项目早期Python查询/桥接路径 | 新链路不依赖固定ROI逐点查询重建显示，保留官方稀疏层和增量更新 | 尚不能证明新场景与workspaces相同质量 |
| 原始输入分辨率直接进入核心 | 避免额外stride缩图造成的输入细节损失 | 不消除深度噪声、遮挡、时间误差 |
| 图像时刻机械臂TF直接交给官方Transformer | 采用所需pose源，避免拿路径节点当前位姿替代图像位姿 | 图像时间是否真实对应采样时刻仍需核验 |
| GPU筛选紧凑读回替代全体素CPU筛选 | 降低附加点云的主机读回与构造负载 | 仍逐次遍历所有分配块，并同步CUDA流，尚非增量点云输出 |
| PointCloud2替代CUBE_LIST | 减少消息构造和数据体积，便于标准工具读取 | 单独改点样式不会提高重建精度 |
| 开始到开始限频、同一更新版本不重复导出 | 避免导出耗时叠加等待、避免重复发送同一版本 | 核心更新仍受tick、限频和输入处理节奏制约 |
| Mesh/层5 Hz，融合/ESDF目标30 Hz | 降低非每帧必需输出的目标负载 | 是负载配置取舍，不是新ESDF算法；实际达标须测量 |
| inside-only默认筛选 | 减少显示样本 | 语义不符合完整距离场需求；不能作为应保留的功能优化 |

上一轮相同band条件的显示负载下降属于附加输出优化；核心ESDF更新频率约24.4 Hz，未出现同量级核心更新提升。历史结果不能包装为“ESDF算法本身加速很多”，更不能用inside-only减少点数证明质量或完整性提升。本轮不新增任何实测数值。

## 7. 当前仍存在的代码与配置问题

1. **输出语义不充分。** surface名称与实际inside集合不符；默认隐藏外侧距离，点云无inside与unknown契约，是当前最直接的问题。
2. **还没有规划数据闭环。** 有原生层和服务，不等于perception中的规划器已经使用该地图。应明确服务/本地GPU查询、坐标、分辨率、unknown策略、版本与时间含义。
3. **更新次数不等于新信息。** 适配器每次processEsdf调用后直接递增esdf_updated；官方调用可能没有新待更新块。现有计数是更新入口调用次数，不应解释为每次都融合新深度、改变场或完成GPU计算。
4. **默认几何参数不等同workspaces所有示例。** 本地workspaces的gazebo_d435_nvblox示例配置是2 cm、颜色关闭、ESDF目标5 Hz、world坐标；当前3 cm、颜色开启、ESDF目标30 Hz、base_link。这个示例未必就是用户此前全部运行入口，但它足以说明不能笼统宣称“配置一致”。
5. **没有机器人自过滤。** 机械臂会被深度看到并进入静态TSDF；运动过程中旧表面可能保留。删除人物识别不等于删除所有动态影响。
6. **衰减和清图默认关闭。** 虽保留开关，历史表面处理策略没有真正改进。盲目开启衰减又可能损失静态完整性，需要按用途设计。
7. **输入质量和采样时间仍待核实。** 仿真有stddev=0.02的噪声声明，实际传感器作用需确认；插件使用world SimTime生成消息，不能仅据此保证是曝光/采样时刻。TF查询成功只保证该时刻可查，不保证时刻选对。
8. **图层开关与负载不是完全独立。** launch的publish_layer_rate_hz取Mesh/Layer目标最大值；某些关闭开关使用重映射。关闭RViz项、关闭发布和关闭内部计算必须分别核实。
9. **地图规模约束不足。** 默认workspace_bounds_type=unbounded、清图关闭、全块点云导出，长时间扫描可能增加块数、传输量与长尾。
10. **文档历史描述混杂。** 早期1 Hz、CUBE_LIST、默认关闭与当前GPU点云、30 Hz目标同时存在于历史文档。当前语义以本报告及实际代码为准，不能按旧报告重现当前程序。

## 8. 建议下一步按功能优化，不按画面优化

### 第一阶段：先定义并纠正输出契约

保留官方Mapper及ESDF更新。把Mesh用于重建表面，把原生ESDF用于距离计算；将inside体素集合明确标成占用侧诊断层，不再默认代表整个ESDF。主三维距离输出应保留所需工作范围内有效的outside正距离，inside状态与unknown策略单独表达；不能用distance>=0替代inside标志。

完整规划数据建议沿用官方有界ESDF导出或原生GPU层访问，并明确有效性/unknown约定。RViz可选择有限高度切片、外侧距离带或全部已观测样本，这些是读取层，不应改变实际地图或作为唯一规划数据源。unknown保留为未知，具体保守策略由应用制定。

### 第二阶段：恢复与参考工程可追踪的一致性

对用户实际使用的workspaces入口建立配置清单，比较最终参数、传感器、体素、范围、pose源、频率、订阅层与实际加载库。复用官方感知launch、配置分层、增量Mesh/Layer、地图服务及调度；将D435与机械臂TF作为独立输入适配，仅去掉人物/语义、其他设备、VSLAM等无关入口。

当前较小bringup若继续保留，必须给出与官方入口逐项对应关系和缺失功能清单，不能靠“链接了官方库”代表整体功能、配置和效果均已复现。

### 第三阶段：优先处理真实质量与时延原因

核对深度采样时刻和TF；按图像时刻的机器人几何实施自过滤；检查深度单位、内外参、噪声与视场。再评估截断带、融合权重、体素分辨率及历史清理。更小体素必须与计算代价一起评估，不能用调点尺寸代替。

在地图完整性满足要求后，再处理队列等待、无新块重复更新、点云全块扫描、GPU/CPU同步以及冗余相机负载。变化块导出应包括删除和清图事件，保证增量缓存不残留旧点。异步导出必须有地图生命周期和版本一致性设计，不能简单并行读写。

未来验收应分别报告表面误差/完整性、unknown覆盖、运动残留持续时间、成功深度融合率、实际地图变化、ESDF计算耗时与数据年龄；RViz绘制fps或点样式只作显示指标。本轮按要求不执行这些测试。

## 9. 代码阅读入口

- [当前mapper适配](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp)
- [当前GPU点云筛选](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu)
- [当前启动及参数覆盖](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/mapping.launch.py)
- [当前基础参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/nvblox/base.yaml)
- [官方核心更新与输出](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/nvblox_node.cpp)
- [官方ESDF符号和种子规则](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/nvblox_core/nvblox/src/integrators/esdf_integrator.cu)
- [目录与官方核心组织](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ARCHITECTURE.md)
- [来源清单](/home/czz/perception_eye_in_hand/nvblox_in_hand/provenance/source_manifest.json)

当前结论是：工程已复用官方核心并完成若干输入/编排简化及附加输出性能优化，但完整ESDF输出契约和真实重建质量优化尚未完成，不能称为已经达到workspaces等同效果。
