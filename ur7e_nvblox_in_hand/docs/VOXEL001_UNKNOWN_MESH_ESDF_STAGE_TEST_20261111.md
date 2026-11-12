# unknown 的含义与 1 cm 体素 Mesh / 3D ESDF 分阶段测试

日期：2026-11-11。测试对象：**ur7e_nvblox_in_hand**，Python ROS接入、nvblox_torch C++/CUDA计算后端。不是旁边的 nvblox_in_hand C++ ROS 工作区。

## 1. 结论

本轮完成用户要求的 **voxel_size=0.01 m** 测试，沿用上一轮2cm测试的640×480输入、场景、运动轨迹、参考视角、静止60帧数据及几何评估方法。新增单帧/静止多帧质量实验和一次正常实时回放，**没有修改生产源代码、配置默认值和仿真场景**。只新增本轮测试与分析产物、本文档及README入口；没有操作实机。

- **unknown 是没有足够有效观测的空间，不能认作空闲，也不等于物体内部。** 本轮最终ROI有34,852个unknown：12,864个没有TSDF块，21,988个虽然分配了块但体素权重为0。没有发现“TSDF权重已满足门限、ESDF却仍unknown”的样本。
- 1cm下，最终三个视角汇总Mesh轴向误差P95 **16.66 mm**，优于上轮2cm的43.23 mm；但仅物体区域P95仍有 **55.22 mm**，不能根据大面积台面的较好结果宣称薄结构已经准确。
- 最终已观测自由空间ESDF误差MAE **5.41 mm**、P95 **9.69 mm**，unknown比例 **10.37%**。表面误差和距离场误差是不同统计量，不可直接互换。
- 深度融合调用 **28.59 Hz**，3D ESDF更新调用 **23.73 Hz**，点云实收 **23.30 Hz**；输入到点云接收的延迟代理值P50 **90.33 ms**、P95 **145.70 ms**。
- **仍存在严重距离异常。** 多视角原生地图有2个已观测自由体素输出+2m，参考距离约0.242/0.245m；保存体素的父方向为零，却存在0.39/0.36m的已观测路径通向种子。单帧还存在一个孤立体素输出−2m，静止60帧后不再饱和；两者不能混作同一个原因。

本轮性能来自一次约87s仿真记录回放，关闭RViz/Gazebo GUI、保持真实Mesh和ESDF订阅者。不是实机、屏幕刷新或多轮统计的结果。**测试结束，自己的回放、Mapper和监测进程已停止。**

## 2. unknown 到底指什么

### 2.1 三种状态必须分开

| 状态 | 含义 | 能否当作可通行空间 |
|---|---|---|
| 已观测自由空间 | 有观测支持该位置在障碍物外侧 | 还需结合距离误差与机器人安全余量 |
| 已观测内部/占用侧 | 融合结果判断该位置处于表面内部 | 不可通行；符号也可能受离散误差影响 |
| unknown | 缺少判断依据，或有效观测不足 | 不能无条件当作自由空间 |

相机看到一只试管，只能直接约束可见表面及其前方被观察的空间；试管背面、架子遮挡的角落、台面下方，不能凭空认为没有障碍。**未知是观测状态，不是一个具体的距离值。** unknown不是“距离等于0”，也不是“距离等于最大值2m”。

### 2.2 当前代码如何处理

[backend.py:36](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:36) 将 `unobserved_esdf_policy` 设置为 **kIgnore**。项目固定版本的官方头文件 [esdf_integrator_params.h:22](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/esdf_integrator_params.h:22) 明确：没有TSDF或权重过低的位置保持unobserved，ESDF不从这些位置、也不穿过这些未知体素传播。

本轮 `esdf_min_weight=0.1`。体素属于被分配的区块，不等于体素已经被观测；nvBlox按8×8×8体素分配块，同块内一些体素仍可能完全没有融合数据、权重为0。

[native/export.cu:25](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/native/export.cu:25) 只导出 observed 且距离数值有效的ESDF体素。查询接口另外返回 observed/valid 状态，不能只读取距离而丢弃这个标志。

### 2.3 常见产生原因

| 原因 | 本项目中的具体体现 | 改善方向 |
|---|---|---|
| 视场没有覆盖 | 物体背面、桌边外、相机背后 | 从新增角度观察，不能仅原地增加帧数 |
| 遮挡 | 试管、瓶子挡住架子内部和墙角 | 侧向/俯仰补视角，允许保留不可见区域 |
| 深度无效或过滤 | 0、NaN、距离小于0.1m或超过3m | 检查原始深度、范围设置和过滤统计 |
| 表面后方超出融合带 | 1cm×4体素=4cm，表面后方更深区域通常没有该次深度的有效更新 | 从另一侧观测；不应无限延伸占用推断 |
| 已分配但未更新 | 体素块按512个体素分配，部分格点权重仍为0 | 检查逐体素权重，而不是只看块存在 |
| 权重不足 | 正权重仍低于esdf_min_weight | 先看确实是低正权重还是零权重，再决定是否调门限 |
| 输入/TF不及时 | 尚未融合，或因队列覆盖、过期等丢弃观测 | 检查输入链路、时间匹配和接入统计 |
| 工作区限制、清图/衰减 | 其他配置可能裁剪或移除旧观测 | 本轮unbounded、clearing_hz=0、decay_hz=0，不是本轮主动清图所致 |

本次数据只能直接判明“是否有块、权重与ESDF状态”；**不能仅靠这三项把每个零权重体素进一步精确归因于遮挡、无效深度还是区块选择。** 这些需要逐射线观测来源追踪。

### 2.4 看见空白不一定就是unknown

- `stride=2`只显示部分真实已观测格点；点之间的间隙并不意味着完整地图unknown。
- `band`或`inside`显示模式会过滤部分有效点；本轮使用observed，未按距离带过滤。
- Mesh缺三角面还可能与邻居有效性、权重门限、遮挡及表面提取有关。
- 本轮+2m异常体素是 **observed=true**，不属于unknown；它们是需要另查的距离传播/状态问题。

## 3. 输入、参数与公平对照边界

### 3.1 实际输入和运行路径

- 仿真记录：[input_bag](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag)。包含Depth、RGB、各自CameraInfo、TF、关节和/clock。
- 实验台几何：[lab_scene.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/lab_scene.yaml)，含试管架、试管、瓶子、托盘和尺寸标块。
- 精度参考视角：[views](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/views)。本轮与2cm使用相同文件。
- 静止输入：[static_inputs.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/static_inputs.json)，与2cm文件逐字节相同；60帧深度及60帧颜色，跨度约1.965s，相机最大漂移0.00185mm/0.000123°。
- 动态TF约20Hz、图像约30Hz。每路使用自身内参和图像时间戳对应的optical TF，不将未对齐深度解释成彩色图像。
- 域162启动前四个关键话题发布者均为0，见 [preflight.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/preflight.json)。测试时未启动新Gazebo或实机；保留用户已有其他进程。
- [test_manifest.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/test_manifest.json) 记录输入/代码校验和及对照边界。

实际数据流：

```text
Depth + CameraInfo → 图像时间戳TF → 米制转换/无效深度过滤 → add_depth_frame → TSDF
RGB + CameraInfo   → 自身时间戳TF → RGB数据转换            → add_color_frame → Color
TSDF → update_esdf → 全图原生observed导出 → stride2 → ROS点云打包 → 订阅端
TSDF + Color → update_color_mesh → 全量Mesh回读 → ROS MarkerArray / PLY
```

### 3.2 实际参数

完整生效配置：[voxel001/params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/voxel001/params.yaml)。

| 参数 | 本轮1cm | 上轮2cm |
|---|---:|---:|
| voxel_size | 0.01m | 0.02m |
| truncation_vox | 4 | 4 |
| 物理TSDF截断距离 | **4cm** | **8cm** |
| ESDF表面种子门限 | 2体素=2cm | 2体素=4cm |
| esdf_view_stride | 2 | 2 |
| 显示采样间距 | **2cm** | **4cm** |
| depth_hz / color_hz | 40 / 40 | 40 / 40 |
| esdf_hz / esdf_view_hz | 33.4 / 33.4 | 33.4 / 33.4 |
| mesh_hz / layer_hz | 5 / 5 | 5 / 5 |
| 分辨率、输入QoS | 640×480、reliable | 相同 |
| 深度范围 | 0.1–3m | 相同 |
| max_weight | 5 | 5 |
| weighting_mode | inverse_square_tsdf_distance_penalty | 相同 |
| ESDF最大距离/最小权重 | 2m / 0.1 | 相同 |
| mesh_min_weight、layer_min_weight | 0.1 / 0.1 | 相同 |
| raycast_subsampling_factor | 4 | 4 |
| workspace_bounds_type | unbounded | 相同 |
| decay_hz、clearing_hz | 0 / 0 | 相同 |
| input_queue_size、max_input_age_sec | 3 / 0.5s | 相同 |

**对照限制：** 固定的是体素单位截断和种子门限，物理范围也随体素变了；输出采样更密，输出负载不同。两轮为独立运行、完整融合的帧集合略有不同，整机其他进程负载也不保证完全相同。因此本轮比较是这两套实际配置的效果，不是“只改体素分辨率、其他物理量完全不变”的严格单变量实验。

本轮所有源文件保持原样。默认base/realtime配置可能相互覆盖，复现应使用归档快照和显式参数，不要只改某一个YAML就认定最终生效。

## 4. 分阶段精度测试

| 阶段 | 操作 | 要回答的问题 |
|---|---|---|
| A 原始深度 | 同三个观察视角与场景几何射线真值比较 | 输入表面本身是否差 |
| B 单帧 | 空地图仅融合首帧深度和颜色，导出TSDF/Mesh/ESDF | 静止、未多帧叠加时是否已损失细节 |
| C 静止60帧 | 融合同一停留段全部60帧深度、60帧颜色 | 重复观测是否累积、是否自动补遮挡 |
| D 多视角 | 正常ROS回放完整轨迹并保存结果 | 补视角对覆盖、误差、历史污染的影响 |
| E 原生/输出核对 | TSDF插值、ESDF原生与查询/stride比较、读取原始BLOX | 错在融合/地图层，还是导出/显示 |

B/C是直接调用同一Backend的离线质量隔离实验，不经过节点的丢帧门控；它们的执行速度**不是实时融合Hz**。D才采用实际节点的时间匹配、队列、调度和发布逻辑。

### 4.1 误差定义

- Mesh：在记录的深度相机内参和位姿下，对真实保存PLY作射线求交，与程序生成场景的几何深度比较，统计**相机轴向绝对误差**，不是全局最近点距离。
- 比较仅限有输入深度、有参考几何的可见ROI，排除明显前景机器人遮挡。机器人不在参考几何中，简单遮挡排除并不等于完整自体遮罩；边缘和历史机器人仍可能影响尾部误差。
- 圆柱/球采用离散参考网格，架环用场景资产；不是高精度实物计量。
- ESDF：同一物理评估体积x=[0.16,1.0]、y=[−0.5,0.5]、z=[0,0.4]m，本轮查询**336,000个1cm体素中心**，上轮为42,000个2cm中心。查询只作离线精度评估，不是在线生成地图的路径。
- 仅对真值自由空间且observed的位置统计ESDF距离误差；unknown单独统计。两种体素中心和observed集合不同，不能把样本数直接对比。
- ESDF外部参考按场景实体并集的外部距离计算，不评价并集内部的近似距离。
- P50/P95是绝对误差或延迟的50%/95%分位数；不等于准确率、平均值或最大值。

### 4.2 原始深度

相同三个视角合计373,340个有效像素：MAE **1.66mm**、P50 **0.53mm**、P95 **3.06mm**、RMSE **9.37mm**，最大约256mm。主体输入较好，但边缘/遮挡少数离群值仍存在。

### 4.3 Mesh结果：同一个initial视角

| 阶段 | MAE | P95 | 去台面物体区域P95 | 覆盖率 | 顶点/三角面数 |
|---|---:|---:|---:|---:|---:|
| 单帧 | 2.63 mm | 10.70 mm | 23.57 mm | 94.41% | 55,348 / 82,686 |
| 静止60帧 | 2.63 mm | 10.71 mm | 23.58 mm | 94.41% | 55,349 / 82,689 |
| 多视角结束 | 3.25 mm | 13.28 mm | 29.09 mm | 99.68% | 124,811 / 192,694 |

最终三个视角汇总：MAE **4.33mm**，P95 **16.66mm**，覆盖率 **99.62%**。但去台面后的物体区域P95 **55.22mm**，最大轴向误差约931mm；后者可能包含未建模机器人历史和遮挡影响，不能当作所有物体的统一精度。

![1cm Mesh分阶段效果](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/mesh_stage_comparison.png)

原始RGB相机与深度相机有视差，RGB图仅作视觉辅助；本轮没有计算逐像素颜色准确率。

静止60帧中，仅0.6165%的像素变化过，有效像素时间标准差P50/P95均为0mm。TSDF中位权重从 **0.297** 增至 **5.0**，达到权重上限的比例从0.48%增至52.48%，多视角结束约94.91%。说明多帧融合执行并增加权重，但重复相同观察不能恢复遮挡区域，也不会自动消除系统性离散误差。

### 4.4 TSDF是否已包含表面损失

对有8个有效TSDF邻居的Mesh顶点作三线性插值：

| 阶段 | 有效TSDF体素数 | 检验顶点数 | 顶点检验比例 | Mesh顶点处TSDF残差P95 |
|---|---:|---:|---:|---:|
| 单帧 | 792,800 | 51,104 | 92.33% | 0.000099 mm |
| 静止60帧 | 1,629,563 | 51,103 | 92.33% | 0.000100 mm |
| 多视角结束 | 4,314,471 | 120,507 | 96.55% | 0.000078 mm |

这些已检验顶点基本位于TSDF零表面。微小残差是**Mesh与TSDF的一致性**，不代表对场景有亚微米精度；没有覆盖所有边界顶点。单帧Mesh已经损失细节，而Mesh跟随TSDF，说明不能只修RViz或渲染来解决几何模糊。

![1cm TSDF截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/tsdf_stages.png)

TSDF保留投影截断距离，本身不是全空间精确欧氏距离。图中的无观测区不能被解释成数值为零的表面。

## 5. ESDF、unknown与异常结果

### 5.1 距离精度与观测完整性

| 阶段 | observed / 336,000 | unknown | 自由空间MAE | P95 | 最大绝对误差 | >10cm误差体素数 |
|---|---:|---:|---:|---:|---:|---:|
| 单帧 | 248,620 | 26.01% | 5.80 mm | 10.94 mm | 2004.31 mm | 57 |
| 静止60帧 | 248,622 | 26.01% | 5.79 mm | 10.94 mm | 121.19 mm | 56 |
| 多视角结束 | 301,148 | 10.37% | 5.41 mm | 9.69 mm | 1757.80 mm | 58 |

“真值自由距离≥2cm，却标为内部”的样本数为120 / 120 / 189，占各阶段已观测自由空间样本约0.049% / 0.049% / 0.064%。不同分辨率的样本数量应以比例和物理范围比较；近表面符号偏差与深部自由空间误判也应分开。

最终原生observed体素 **4,314,471个**，stride2导出 **542,659点**，相邻显示格点2cm。单帧/静止60帧导出99,566 / 204,930点。全图体素增加也可能发生在ROI之外，所以同视角ROI observed基本不变，而全图体素数仍能增加。

![1cm原生3D ESDF截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/esdf_accuracy_stages.png)

### 5.2 unknown究竟产生在哪一阶段

以下由保存地图的原始TSDF/ESDF体素BLOB检查，不依赖RViz颜色猜测：

| 阶段 | 无TSDF块 | 有块但体素权重0 | 正权重但低于0.1 | TSDF有效而ESDF未知 | unknown总数 |
|---|---:|---:|---:|---:|---:|
| 单帧 | 36,608 | 50,772 | 0 | 0 | 87,380 |
| 静止60帧 | 36,608 | 50,770 | 0 | 0 | 87,378 |
| 多视角结束 | 12,864 | 21,988 | 0 | 0 | 34,852 |

结论：本轮ROI unknown全部对应**没有有效TSDF观测**，不是已具备合格TSDF后被ESDF遗漏。最终unknown中约36.91%没有TSDF块，63.09%有块但体素权重0；不存在低正权重不足的样本。因此仅把`esdf_min_weight`从0.1调低，不能给这些位置创造真实观测。

单一角度增加60帧后unknown几乎不变；增加视角后从26.01%降至10.37%。这直接体现了**补视角比原地重复输入更能改善遮挡覆盖**。

![unknown与observed分阶段示意](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/unknown_stages.png)

图为z=0.115m截面；标题百分比使用整个三维ROI。橙色表示无有效TSDF观测，青色表示observed。

1cm的最终unknown略高于2cm的9.48%，并不矛盾：更高分辨率不会穿透遮挡，物理截断带从8cm变4cm，体素中心不同，本轮参与融合的帧也略少。没有单因素实验，不能把这0.90个百分点全部归因于某一因素。

### 5.3 observed不等于距离一定正确：饱和值异常

多视角阶段发现两个自由空间体素：

| 位置（m） | 输出 | 参考距离 | 最近原生site距离 | 已观测六连通路径到site |
|---|---:|---:|---:|---:|
| (0.735, −0.455, 0.375) | +2m | 0.2422m | 0.2460m | 0.39m |
| (0.765, −0.405, 0.365) | +2m | 0.2446m | 0.2486m | 0.36m |

原始保存BLOX中二者都是 `squared_distance_vox=40000`、`parent_direction=[0,0,0]`、`observed=true`、`is_inside=false`、`is_site=false`，因此sqrt(40000)×0.01=2m。TSDF为+0.04m且权重约0.516/0.504。这些数值在保存层中已经存在，并非加载、查询、颜色映射才产生。

两点都能沿observed体素连通到有效site，路径远小于2m，因此不能仅以未知空间隔断解释。优先排查增量传播的父方向、待更新区块、跨块传播和TSDF状态转换。**本轮没有内核级跟踪或其他官方版本同输入对照，尚不能直接认定是哪一行CUDA代码的缺陷。**

这两个点均未被stride2抽中。只看较稀的显示点云可能漏掉完整地图的问题。

![1cm ESDF异常点局部](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/esdf_outlier_detail.png)

单帧另有一个点(0.545,−0.215,0.065)m：输出−2m、参考外部距离约4.31mm，父方向也是0，但本轮局部六连通检查找不到到site的已观测路径（仅访问自身一个体素）。它可能涉及孤立观测分量缺少可传播种子、表面符号和截断观测的组合问题，**不能与上面两个已连通自由体素直接归为同一种错误**。静止60帧后该点不再饱和，但这不意味着所有异常已解决。

### 5.4 查询与导出核对

- 三阶段ROI查询observed状态与原生导出完全一致，不一致数 **0**。
- 单帧/静止/多视角，共同observed的248,620 / 248,622 / 301,148个位置，查询距离与原生层距离差 **0**。
- 三阶段stride2输出均为完整原生格点的精确子集。

因此当前证据把严重距离异常定位到**保存的ESDF地图状态及其上游更新**，不支持“ROS查询填充制造错误值”作为本次主要解释。查询网格是本次离线评估工具；在线节点直接导出真实observed体素。

## 6. 分阶段实时性与2cm对比

稳定统计窗口76.08s，去掉回放前10s；非空点云接收窗口77.67s。输入保持约29.96Hz；频率均按实际墙钟时间计算。

| 环节 | 本轮1cm | 上轮2cm |
|---|---:|---:|
| 深度输入 | 29.96 Hz | 29.95 Hz |
| 颜色输入 | 29.96 Hz | 29.94 Hz |
| 深度融合调用 | 28.59 Hz | 29.37 Hz |
| 颜色融合调用 | 28.80 Hz | 29.47 Hz |
| 3D ESDF 更新调用 | 23.73 Hz | 28.59 Hz |
| 3D ESDF 发布调用 | 23.71 Hz | 28.59 Hz |
| Mesh 发布调用 | 4.85 Hz | 4.98 Hz |
| 订阅端非空 ESDF 实收 | 23.30 Hz | 26.17 Hz |

这些融合/更新指标是**host调用返回后的计数**，不是每个CUDA内核独立完成的频率。原有metrics标明`gpu_completion_measured=false`。实际接收频率也不是RViz屏幕刷新频率。

| 延迟指标 | 本轮1cm | 上轮2cm |
|---|---:|---:|
| 输入CameraInfo匹配接收→ESDF点云接收P50 | 90.33ms | 49.73ms |
| 同上P95 | 145.70ms | 59.59ms |
| 同上最大值 | 345.10ms | 180.02ms |
| 深度接收→融合开始P95 | 79.15ms | 44.11ms |
| ESDF更新调用间隔P95 | 70.18ms | 49.32ms |
| 点云接收间隔P95 | 86.27ms | 68.64ms |

输入延迟使用相同深度时间戳的CameraInfo到达时间做代理，排除真实相机曝光、驱动和RViz渲染；不能称为完整实机端到端延迟。P95表示95%的有效测量不超过该值，不是最大值。

### 6.1 哪些阶段开销增长

| 阶段 | 1cm平均host耗时 | 1cm P95 | 2cm平均host耗时 |
|---|---:|---:|---:|
| 深度融合 | 2.57 ms | 4.66 ms | 1.67 ms |
| 颜色融合 | 4.20 ms | 9.31 ms | 1.30 ms |
| ESDF 更新 | 6.69 ms | 13.55 ms | 1.60 ms |
| ESDF 全图导出及GPU等待 | 9.69 ms | 18.64 ms | 1.47 ms |
| Mesh 更新/回读 | 31.50 ms | 45.02 ms | 7.35 ms |
| ESDF 打包/发布 | 18.01 ms | 26.85 ms | 2.15 ms |
| Mesh 打包/发布 | 24.09 ms | 35.56 ms | 8.93 ms |

工作线程上述5个阶段调用的总墙钟占用约73.36%，不含所有调度、等待、TF重试等；输出线程单独运行，不能直接把两线程时间相加当单帧延迟。GPU异步调用和等待也会跨阶段分摊。

![1cm与2cm精度和实时性对照](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/comparison_001_vs_002.png)

### 6.2 为什么深度约29Hz，ESDF只有约24Hz

1. **地图与全图导出规模上升。** 2cm/1cm最终ESDF块数分别2,135 / 11,943，原生observed体素584,170 / 4,314,471，stride2显示点72,633 / 542,659。这个场景并非严格等于理论8倍，但规模增长很明显。
2. **当前桥接导出会遍历全层。** [native/export.cu](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/native/export.cu) 每次`cudaDeviceSynchronize()`后枚举全部块，按最多512块分批压缩，取回计数、同步，再拷回CPU并同步。即使核心融合是增量的，**这条显示导出路径仍不是仅发布变化块的增量路径**。体素减小后，这项平均耗时由1.47ms增至9.69ms。
3. **Mesh更新和回读共享GPU工作线程。** [backend.py:107](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:107) 调用官方Mesh更新并将整个返回网格搬回CPU，本轮一次平均31.50ms、P95 45.02ms；它虽只以约5Hz执行，也会穿插阻塞融合与ESDF工作。
4. **输出负载显著增加。** 最后一条ESDF消息约13.02MB，上轮约1.74MB；平均实收载荷250.71MB/s，上轮37.93MB/s。ESDF打包发布平均18.01ms，上轮2.15ms；还存在Mesh打包平均24.09ms的负担。这里是消息载荷，不是宣称网卡实测吞吐。
5. **更新有独立调度和dirty状态。** [node.py:204](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:204) 在同一个work循环里穿插融合、ESDF和Mesh，且ESDF限频33.4Hz。忙时多个深度融合可由一次ESDF更新统一反映；平均每次更新前约1.21次深度融合，P95为2次、最大3次，不要求每一帧都独立导出一次距离场。

所以，问题不能只理解为“深度融合没到30Hz”或“查询函数很慢”。本轮**融合之后的ESDF更新、全图导出、Mesh回读与输出处理**是更突出的开销阶段。

### 6.3 TF等待与排队的变化

单次TF lookup平均0.046ms、P95 0.091ms，仍远小于几十毫秒。深度接收后到首次尝试TF平均 **31.22ms**，上轮仅6.05ms；首次尝试TF到成功平均 **9.28ms**，上轮19.72ms。

后一个等待变短不代表系统变快：1cm工作更忙，首次处理输入前已经排队较久，开始查TF时所需样本可能已经到达。总体接收→融合开始平均40.57ms，上轮25.83ms，延迟仍上升。

稳定窗口深度队列覆盖69帧、颜色覆盖51帧；门控跳过35/37帧，上轮覆盖7/0帧。输入本身仍约30Hz，说明本轮确有处理/调度和队列压力。一次回放的差异不能证明每个损失都由某一个函数独立造成。

全GPU低频采样利用率均值29.7%、P95 48.5%，包含整卡及其他用户进程。不能以此代替CUDA内核剖析，也不能据此声称GPU始终满载。

![实际host阶段时间线](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/pipeline_actual_timeline.png)

## 7. 问题阶段与后续改善

| 阶段 | 本轮证据 | 判断及下一步 |
|---|---|---|
| 输入/有效观测 | 原始深度P95 3.06mm；最终unknown全为无块或权重0 | 主要遮挡覆盖需要新视角；门限调低不能替代缺失观测 |
| TSDF表面表示 | 单帧物体区域P95 23.57mm；单帧已损失细节 | 细结构、截断和投影加权仍需单因素分析；6mm架条仍小于1cm体素 |
| 静止多帧 | 权重增长，但Mesh/unknown基本不变 | 多帧在执行；重复相同深度不能自动恢复细结构与背面 |
| 运动补视角 | 覆盖94.41%→99.68%，unknown26.01%→10.37% | 有补全收益；未启用自体剔除、衰减，历史污染仍可能存在 |
| Mesh提取 | 已测96.55%最终顶点贴合TSDF零表面 | 不能仅靠换渲染修复几何；保留PLY/TSDF直接检查 |
| ESDF状态/传播 | 已连通种子却存在+2m、父方向0的异常点 | 优先比较增量更新与全量重建，追踪跨块更新及父节点状态 |
| 导出/ROS输出 | 全图扫描、同步、回读及大点云打包增长 | 后续考虑变化块/局部区域输出、设备端复用、减少重复拷贝与同步 |
| 任务调度 | 首次处理前排队增长，Mesh单次回读数十ms | 后续隔离Mesh/显示任务成本，避免可视化拖慢地图更新 |

建议先解决距离正确性：区分unknown、无可用种子的孤立observed分量、已有有效传播路径却仍饱和的体素。随后才优化全图导出和Mesh回读。**不建议通过美化Mesh、把unknown全填free、隐藏大误差点来宣称优化成功。**

后续验收同时记录：逐物体Mesh误差、自由空间距离高估尾部、unknown比例、更新与实收频率及P95延迟；仅达到30Hz不等于可用于精细抓取。

## 8. 数据与复现

### 8.1 直接打开本次数据

- [多视角Mesh PLY](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/multiview/mesh.ply)：CloudCompare可直接打开，带顶点RGB。
- [ESDF显示点PLY](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/multiview/esdf_observed.ply)：542,659点，用distance标量着色。
- [完整ESDF原生数据](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/multiview/esdf_native.npy)：4,314,471行，列为xyz、signed distance、observed、inside、site。
- [完整地图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/multiview/map.blox)：TSDF、Color、ESDF等层；Mesh单独保存为PLY。
- [精度结果](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/quality_summary.json)、[实时性结果](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/pipeline_summary.json)、[unknown阶段分解](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/unknown_stage_breakdown.json)、[原生异常体素核对](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/native_outlier_checks.json)。

现有metrics的`native_esdf_points`与`esdf_exported_points`都来自导出后的同一计数，不要把字段名当作完整原生体素数；本报告用stride=1原生导出确认全量大小。

保存BLOX时日志有MeshBlockLayer不被序列化的提示，但地图层成功保存且加载导出成功，Mesh另行PLY保存成功。因此归档时同时保留PLY和BLOX。

### 8.2 编译并回放同一仿真输入

以下方式不启动Gazebo、不发实机或仿真运动命令，直接回放已保存轨迹。开启RViz后其渲染开销会改变整机性能，不能直接照搬本轮无GUI测量值。

终端1：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
source ./env.sh
export ROS_DOMAIN_ID=162 ROS_LOCALHOST_ONLY=1

ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true \
  mapper_config:=/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111/baseline_params.yaml \
  voxel_size:=0.01 truncation_vox:=4.0 \
  depth_hz:=40.0 color_hz:=40.0 \
  esdf_mode:=3d esdf_hz:=33.4 esdf_view_hz:=33.4 \
  esdf_view_mode:=observed esdf_view_stride:=2 \
  mesh_hz:=5.0 input_qos:=reliable \
  decay_hz:=0.0 clearing_hz:=0.0 \
  run_rviz:=true rviz_mode:=diagnostic
```

等Mapper ready后，终端2：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=162 ROS_LOCALHOST_ONLY=1
ros2 bag play /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag \
  --disable-keyboard-controls
```

记录内含/clock，不要再向域162发布其他仿真时钟。再次播放前重启Mapper，使地图为空，避免旧地图和时间倒回干扰。

终端3检查：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=162 ROS_LOCALHOST_ONLY=1
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox truncation_vox
ros2 param get /ur7e_nvblox esdf_view_stride
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

话题分别是 `/ur7e_nvblox/esdf_points`、`/ur7e_nvblox/mesh`、`/ur7e_nvblox/tsdf_layer`、`/ur7e_nvblox/color_layer`、`/ur7e_nvblox/camera_path`，不要和其他旧实现名称混用。轨迹来自机械臂TF，不是SLAM估计。

回放结束保持Mapper运行，保存新结果：

```bash
TEST_OUT="$PWD/artifacts/manual_voxel001_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUT"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/map.blox'}"
ros2 param dump /ur7e_nvblox > "$TEST_OUT/params.yaml"
```

### 8.3 脚本可追溯性

本轮所有脚本在 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel001_accuracy_pipeline_20261111`：`run_test.py`为正常回放；`traced_mapper.py`为原调用外层的测试观测，不改变算法调度；`export_quality.py`为单帧/静止60帧和原生层导出；`analyze_quality.py`、`analyze_realtime.py`、`analyze_unknown.py`、`check_native_outliers.py`分别检查精度、阶段耗时、未知来源和原始异常。`geometry.py`生成场景几何参考，`make_comparison_figures.py`只读取真实结果画图。

固定输出目录脚本拒绝覆盖已有实验；不要为重复运行删掉本轮证据，按上面的手动方式写入新目录。本轮没有额外注入TF偏移、关闭颜色或改变输入分辨率等无关对照。
