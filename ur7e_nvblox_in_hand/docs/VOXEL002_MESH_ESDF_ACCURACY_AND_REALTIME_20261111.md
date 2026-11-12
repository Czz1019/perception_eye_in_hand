# 2 cm 体素：Mesh、3D ESDF 精度与分阶段实时性测试

测试日期：2026-11-11。对象：`ur7e_nvblox_in_hand`，即 Python ROS 接入、`nvblox_torch` C++/CUDA 后端这一套，**不是**旁边的 `nvblox_in_hand` C++ ROS 工作区。

## 1. 结论与边界

本轮完成一次正常仿真记录的实时回放，以及单帧、静止 60 帧、多视角三阶段的几何检查。生产源代码、用户配置和已有仿真场景没有修改；新增内容均为本次测试脚本、数据、图片和本文档。测试结束，自己的 Mapper、回放和监测进程已退出；没有驱动实机。

- 深度输入 **29.95 Hz**，实际融合调用 **29.37 Hz**；颜色融合调用 **29.47 Hz**。
- 3D ESDF 更新调用与发布调用均为 **28.59 Hz**，测试订阅端实际收到完整非空点云 **26.17 Hz**。本轮没有证明 RViz 屏幕刷新也是这个频率。
- 输入消息到 ESDF 点云接收的延迟代理值：P50 **49.73 ms**，P95 **59.59 ms**。不包含相机曝光、真实驱动以及 RViz 渲染。
- 同一初始视角下，单帧 Mesh 对几何真值的轴向误差 P95 **32.38 mm**；多视角后 **35.64 mm**，可见区域覆盖率从 **94.44%** 增至 **99.44%**。更多视角明显补全覆盖，没有证明几何精度同步提高。
- 多视角后，ROI 内已观测自由空间的 ESDF 距离误差 MAE **11.99 mm**，P95 **20.76 mm**；ROI unknown 从 **24.29%** 降至 **9.48%**。
- **少量 ESDF 严重异常仍需处理**：4 个自由空间体素输出 2 m，但几何参考距离只有约 0.157–0.198 m，最大误差 **1.843 m**。原始保存的 ESDF 体素、加载后的原生导出和查询三者一致。这不能用较好的平均值掩盖。

这是本机一次约 87 秒记录回放的结果，不是多轮统计置信区间，也不是实机精度或整机抓取验证。本次没有额外注入 TF 偏移。

## 2. 输入、配置和使用的代码

### 2.1 输入与运行方式

使用同一份既有实验台仿真记录，含机械臂小幅左右观察、停留、RGB、Depth、对应 CameraInfo、关节信息和 TF。不是重新随机生成不同轨迹后作比较。

- 输入数据库：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag/input_bag_0.db3`。
- 精度评估的 initial / left / right 原始观察：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/views`。
- 实验台几何：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/lab_scene.yaml`，配套 `scripts/lab_scene.py` 和 `meshes/lab`。
- 隔离 ROS_DOMAIN_ID=161。启动前检查 `/clock`、`/tf`、输入深度、Mapper metrics 无发布者，见 [preflight.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/preflight.json)。
- 相机输入 640×480，图像约 30 Hz，动态 TF 约 20 Hz，关节约 100 Hz。正常时间匹配，无人为时间戳偏移。
- **实时性测量关闭 Gazebo GUI 和 RViz**，同时保持 ESDF 和 Mesh 实际订阅者。渲染图来自实际保存的 Mesh 数据离线渲染，不是伪造 RViz 截图。用户开启 GUI 后应另外测量整机开销。

### 2.2 本轮实际生效参数

完整快照：[params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/voxel002/params.yaml)。

| 参数 | 本轮值 | 作用 |
|---|---:|---|
| voxel_size | 0.02 m | TSDF、3D ESDF 原生体素精度 |
| truncation_vox | 4.0 | TSDF 截断距离 0.08 m，并非要求表面误差为 8 cm |
| depth_hz / color_hz | 40 / 40 Hz | 融合准入门限，不会把约 30 Hz 输入变成 40 Hz |
| esdf_hz / esdf_view_hz | 33.4 / 33.4 Hz | 更新/发布上限，不是吞吐承诺 |
| esdf_mode / esdf_view_mode | 3d / observed | 导出真实已观测 ESDF，不把 unknown 当作自由空间 |
| esdf_view_stride | 2 | 显示点云空间间距 4 cm；完整计算和精度评估仍为 2 cm |
| esdf_max_distance_m | 2.0 m | 最大传播距离；本次异常体素正好达到该值 |
| esdf_min_weight / esdf_max_site_vox | 0.1 / 2.0 | 观测权重、表面种子距离门限；后者对应 4 cm |
| mesh_hz / layer_hz | 5 / 5 Hz | Mesh/层的生成发布上限；未订阅层不触发对应导出 |
| mesh_min_weight / layer_min_weight | 0.1 / 0.1 | 输出有效权重门限 |
| weighting_mode | inverse_square_tsdf_distance_penalty | 深度与距离相关的 TSDF 加权策略 |
| max_weight | 5.0 | TSDF 累积权重上限 |
| min_depth_m / max_range_m | 0.1 / 3.0 m | 深度接入范围 |
| raycast_subsampling_factor | 4 | 视锥/区块选择射线抽样，不是 Mesh 顶点间距 |
| workspace_bounds_type | unbounded | 配置内 min/max 框不在此模式下裁剪工作区 |
| decay_hz / clearing_hz | 0 / 0 | 本轮不作周期衰减或半径清图 |
| input_qos / input_queue_size | reliable / 3 | 图像接入 QoS 与有界队列 |
| max_input_age_sec | 0.5 s | 过期输入丢弃上限 |

注意：当前源码 `config/base.yaml` 与 `config/realtime.yaml` 的体素设置并不相同；base 中的 clearing_hz 和负 clearing_radius_m 也需要成对理解。本轮**没有覆盖用户文件**，使用归档的有效配置并显式指定 2 cm、clearing_hz=0。后文复现命令同样显式覆盖，不能只看某一个 YAML 就认定最终配置。

### 2.3 实际计算路径与代码入口

```text
Depth + Depth CameraInfo ─ 按深度图像时间戳查询自身 optical TF ─ 深度米制转换 ─┐
                                                                         ├─ nvBlox TSDF 增量融合
RGB + Color CameraInfo ─ 按彩色图像时间戳查询自身 optical TF ─ 颜色增量融合 ─┘
                   TSDF ─┬─ ESDF 更新 ─ 原生 observed 体素导出 ─ stride2 ─ PointCloud2
                         └─ Mesh 更新/回读 ─ ROS 打包/发布，及独立 PLY 保存
```

深度仍使用深度 optical frame，颜色使用颜色 optical frame；每路有自己的内参和时间戳，并非把未对齐深度强行解释为 color frame。固定颜色/深度外参由 TF 提供。

- [node.py:154](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:154)：CameraInfo 检查、过期/频率门控、按图像时间查询 TF。
- [node.py:204](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:204)：单一 GPU 工作线程融合、ESDF 更新、Mesh 与层导出。
- [node.py:269](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:269)：独立输出线程打包和 ROS 发布。
- [backend.py:28](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:28)：将截断、权重、ESDF、Mesh 等参数传入官方后端。
- [backend.py:76](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:76)：深度数据解码、毫米→米、无效值处理和 add_depth_frame。
- [backend.py:94](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:94)：update_esdf / 原生 ESDF 导出；mesh() 调用官方 update_color_mesh。
- [native/export.cu](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/native/export.cu)：读取真实 EsdfVoxel 的 observed、squared_distance_vox、inside 和 site 标志。

实际加载核心库来自本机 `nvblox_torch/lib/nvblox/libnvblox_lib.so`，不是 workspaces 的 `nvblox_ros`。wheel 固定提交 `e4a2b00f45ecf989682d72a33a2c29defe39fd2d`。平台记录为 RTX 4060 Laptop / ROS2 Humble / PyTorch 2.9.1+cu128 / nvblox_torch 0.0.10.dev1+cu12ubuntu22，详见工程 THIRD_PARTY.md 和本轮 measurement.json 中的 loaded_libraries。

## 3. 分阶段测试方法

| 阶段 | 操作 | 用来定位的问题 |
|---|---|---|
| A 原始输入 | 对三个记录视角的深度作解析场景射线真值比较 | 几何误差是否已经存在于输入 |
| B 单帧 | 空地图融合 1 帧深度和对应首帧颜色，导出 TSDF / Mesh / ESDF | 不经过机械臂运动，是否已丢失细节 |
| C 静止多帧 | 同一初始停留段，实际融合 60 帧深度和 60 帧颜色 | 多帧是否增加权重；是否因运动才产生问题 |
| D 多视角正常回放 | 使用现有 ROS 节点完整融合与调度，保存最终地图 | 覆盖补全、移动观测后的误差和实时性 |
| E 输出核对 | Mesh 顶点处插值 TSDF；原生 ESDF 对查询、stride 输出 | 是融合/距离场问题还是渲染/导出问题 |

静止 60 帧跨约 1.965 秒，最大相机平移漂移 **0.00185 mm**，最大旋转漂移约 **0.000123°**。B/C 是离线质量隔离实验，逐帧全部融合，不经过正常节点限频；**不能拿它们的运行速度当实时频率**。D 才是实际 ROS 调度的实时测量。

### 3.1 精度定义与评估边界

- Mesh：用实际保存 PLY 的三角面，在记录的深度相机位姿/内参下作射线求交，再与程序生成的台面、瓶子、试管架等几何真值比较。单位为**相机轴向深度误差**，不等同于全局点到面距离。
- 统计含台面、也另外给出物体区域。只在有真实输入深度、有真值且排除明显前景遮挡的像素评估。机器人未作为参考几何建模，简单前景排除并非完美的整段机器人遮罩。
- 场景包含小尺寸物体和孔洞，圆柱/球的参考网格有离散近似；不代表经过高精度计量标定的实物真值。
- ESDF：在工作台 ROI 的 **42,000 个原生 2 cm 体素中心**查询；仅对几何真值为自由空间且实际 observed 的位置评价距离误差，unknown 单列。ROI 中体素中心范围 x=0.17…0.99 m，y=−0.49…0.49 m，z=0.01…0.39 m。
- ESDF 外部参考距离按场景实体并集计算。没有将实体内部的近似距离用于本次自由空间精度结论。
- P95 是绝对误差的第 95 百分位，意味着约 95% 被评估样本的误差不超过该值。它会掩盖最差 5%，所以必须同时报告最大值、异常数及 unknown。
- RGB 参考图与深度相机存在真实外参视差，仅作视觉辅助；本轮没有给出逐像素颜色/纹理精度指标。

## 4. Mesh 与 TSDF 的测试结果

### 4.1 原始深度对几何真值

三个视角合计 **373,340** 个有效像素，MAE **1.66 mm**、P95 **3.06 mm**，但 RMSE **9.37 mm**、最大误差约 **256 mm**，说明少数边缘/遮挡异常仍存在。不能因为总体原始深度清楚就宣称所有输入像素都正确。

### 4.2 同一个 initial 视角比较

| 地图阶段 | Mesh MAE | Mesh P95 | 物体区域 P95（去台面） | 可见参考区域覆盖率 |
|---|---:|---:|---:|---:|
| 单帧 | 6.03 mm | 32.38 mm | 47.86 mm | 94.44% |
| 静止 60 帧 | 6.02 mm | 32.38 mm | 47.86 mm | 94.44% |
| 正常多视角结束 | 6.82 mm | 35.64 mm | 52.29 mm | 99.44% |

多视角结束的三个观察方向合并：MAE **7.88 mm**、P95 **43.23 mm**、物体区域 P95 **66.42 mm**、覆盖率 **99.38%**。这组跨视角汇总不能替代上表的同视角比较。补洞后参与统计的像素集合也增加了，不能单凭 P95 上升就认定“运动让所有已有表面都变差”。

![记录 RGB 与实际 Mesh 分阶段对照](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/mesh_stage_comparison.png)

图中每行对应 initial / left / right。第一列为原始 RGB，后面依次为单帧、静止 60 帧、多视角结束的真实 PLY 渲染。静止地图本就不应覆盖所有后续视角。

6233333333333333333333333Mesh 轴向误差热图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/mesh_error_stages.png)

### 4.3 静止多帧为何几乎不变

这 60 帧中仅 **0.6165%** 像素发生过变化，有效像素时间标准差的 P50/P95 都为 **0 mm**。深度是毫米量化，主要表面重复测量几乎一样；少数边界有明显变化，最大变化 821 mm，不代表整幅图的噪声达到该值。

TSDF 中位权重从单帧约 **0.324** 上升到静止多帧 **5.0**；达到权重上限的体素比例从约 **0.47%** 上升到 **60.49%**，多视角结束约 **95.71%**。这说明融合在累积权重，不能根据“60 帧结果和 1 帧差不多”就判定多帧融合没有执行。相同的系统性离散误差不会因为重复输入相同深度自动消失。

### 4.4 Mesh 是否把好的 TSDF 错误渲染坏了

对拥有八个有效邻居的 Mesh 顶点作三线性 TSDF 插值：

| 阶段 | 检验的顶点数 | 顶点覆盖比例 | 顶点处 TSDF 绝对残差 P95 |
|---|---:|---:|---:|
| 单帧 | 13,697 | 89.62% | 0.000101 mm |
| 静止 60 帧 | 13,697 | 89.51% | 0.000099 mm |
| 多视角 | 31,385 | 96.43% | 0.000084 mm |

**这些已检验顶点高度贴合 TSDF 零表面。** 残差是浮点量级的一致性检查，不是 Mesh 对真值的亚微米精度。它支持“几何模糊在 TSDF 表面表示阶段就已存在”，并削弱“只因 RViz 渲染导致几何差”的解释；没有检验到的边界顶点不能一并保证。

![真实 TSDF 截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/tsdf_stages.png)

TSDF 是投影截断距离，不能把整段 TSDF 数值直接和欧氏 ESDF 当成同一种距离误差。图中留白表示未观测或权重不足，不是自动填成自由空间。

### 4.5 最可能影响表面的阶段

1. **空间表示能力。** 场景有 6 mm 宽架条、直径 23 mm/30 mm 试管和薄环，2 cm 体素只能用约 1–1.5 个体素表示管径，无法承诺保持清晰薄壁与狭缝。voxel_size 并不是 Mesh 必然达到的最大表面误差。
2. **截断与加权。** 4 体素意味着 8 cm 融合邻域；近距离多个表面、遮挡边界以及薄结构可能在投影 TSDF / 插值中互相影响。当前测试证明单帧即存在误差，但**没有独立隔离截断或权重参数的因果效应**，不能断言截断是唯一原因。
3. **运动过程的历史污染。** 本轮是静态 TSDF，未做机器人逐像素剔除，且衰减和清图关闭；进入视场的机械臂和遮挡历史可能保留。由于单帧也差，运动污染不是全部原因。
4. **颜色表现与几何分开判断。** 体素颜色融合/顶点颜色不是高分辨率 UV 纹理。颜色模糊不能证明 TSDF 坐标错位；本次只量化几何与距离场，未宣称颜色问题已解决。

## 5. 3D ESDF 精度、unknown 与异常

| 阶段 | ROI observed / 42,000 | unknown | 已观测自由空间 MAE | P95 | 最大误差 | stride2 输出点数（全图） |
|---|---:|---:|---:|---:|---:|---:|
| 单帧 | 31,799 | 24.29% | 12.05 mm | 21.05 mm | 106.17 mm | 16,087 |
| 静止 60 帧 | 31,799 | 24.29% | 12.05 mm | 21.04 mm | 106.17 mm | 27,856 |
| 多视角结束 | 38,020 | 9.48% | 11.99 mm | 20.76 mm | **1,842.60 mm** | 72,633 |

全图原生 observed 体素数分别为 **126,690 / 222,404 / 584,170**。全图体素数增长与 ROI 内 observed 数不一定一致：新增观测也可能位于评估框之外。显示点数由真实 observed 体素和 stride 决定，不是预先在固定空间补点。

多视角地图有 **9 个**已观测自由体素误差超过 10 cm，其中 **4 个**达到 2 m。对真值距离≥2 cm、却被判作内部的自由体素，单帧/静止/多视角分别为 **50 / 50 / 65 个**。靠近表面会受离散化和表面偏移影响，但大误差不能一概忽略。

![3D ESDF 的 z=0.11m 截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/esdf_accuracy_stages.png)

这张图只展示一个截面，且距离色标上限为 0.2 m，不能用它排除其他高度的异常。下一张特意显示异常所在高度，色标放到 2 m。

### 5.1 2 m 异常点的独立核对

| 坐标（m） | 输出距离 | 真值距离 | 已观测六连通路径到表面种子的长度 |
|---|---:|---:|---:|
| (0.83, 0.15, 0.35) | 2.000 m | 0.1574 m | 0.20 m |
| (0.85, 0.11, 0.35) | 2.000 m | 0.1682 m | 0.26 m |
| (0.83, 0.11, 0.37) | 2.000 m | 0.1788 m | 0.26 m |
| (0.85, 0.07, 0.37) | 2.000 m | 0.1976 m | 0.32 m |

进一步检查已保存 `map.blox` 的 SQLite 原始 voxel BLOB：4 个体素都是 `squared_distance_vox=10000`、`parent_direction=[0,0,0]`、`observed=true`、`is_site=false`、`is_inside=false`。因此按 2 cm 计算，距离正好是 sqrt(10000)×0.02=2 m。

这些值在保存文件中已经存在，并非加载后、PointCloud2 打包或颜色显示才制造的。对应 TSDF 体素为 +0.08 m，自由空间观测权重不为零；局部真实 observed 体素形成通向种子的六连通路径，长度远小于 2 m。**仅用 unknown 隔断来解释这四个点不充分。**

这将排查重点推进到 ESDF 的父节点/距离传播、更新区块集合、跨块边界、TSDF→ESDF 状态转换和增量更新一致性。尚未做内核级时序跟踪，也没有在本轮和另一官方版本作同输入复现，所以不能直接宣布“官方 CUDA 算法已被证明有 bug”。

这些点均未被本次 stride2 格点抽样选中；也就是说，**正常显示点云可能看不见完整 2 cm 地图中的错误体素**。全精度查询和保存数据核对比只看图更重要。

![ESDF 严重异常点，z=0.35m](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/esdf_outlier_detail.png)

### 5.2 是否是查询逻辑或 stride 制造距离错误

- 42,000 个 ROI 查询位置的 observed 状态，与原生导出匹配，**不一致数为 0**。
- 多视角 **38,020 个**共同 observed 位置，查询距离与原生层距离差 **为 0**。
- 单帧和静止阶段也得到相同一致性结论。
- 三阶段 stride2 输出都与原生层“格点索引各轴为偶数”的子集完全一致。

因此本轮证据不支持“先铺固定点，再查询错误填数”是当前导出路径的主要问题。测试的 ROI 网格是**精度评估工具**，不是正常建图节点生成 ESDF 的方式。

## 6. 各环节实时性

稳定统计窗口约 **76.09 s**，去掉开始约 10 s；完整接收窗口约 **77.69 s**。不同工具的窗口略有差异，因此用各自原始计数/时间计算，不强行把接收数和发布数当作逐条严格配对。

| 环节 | 实测频率 | 含义 |
|---|---:|---|
| 深度接收 | 29.95 Hz | ROS 节点收到深度消息 |
| 颜色接收 | 29.94 Hz | ROS 节点收到颜色消息 |
| 深度融合 | 29.37 Hz | add_depth_frame 返回并记数 |
| 颜色融合 | 29.47 Hz | add_color_frame 返回并记数 |
| 3D ESDF 更新 | 28.59 Hz | update_esdf 调用记数 |
| 3D ESDF 发布 | 28.59 Hz | PointCloud2 publish 调用记数 |
| 订阅端非空 ESDF 实收 | 26.17 Hz | 真的完成消息接收 |
| Mesh 发布 | 4.98 Hz | 约等于设定的 5 Hz 上限 |

**融合/更新调用频率不等于单独 GPU 核函数完成频率。** 本轮没有逐核 CUDA Event 测量；原有 metrics 明确 `gpu_completion_measured=false`。导出回读会包含等待，但不能把所有阶段 host 时间简单相加当作曝光到显示延迟。

注意现有 metrics 的 `native_esdf_points` 与 `esdf_exported_points` 都取自同一个导出后计数，不能把该字段名直接理解为完整原生体素数。本报告用 stride=1 原生导出核实全图 584,170 个 observed 体素。

最后一帧实际接收 ESDF 为 **72,633 点**，消息序列化约 **1.743 MB**，接收侧平均数据载荷约 **37.93 MB/s**。这已经是有实质带宽/打包开销的输出。

![各阶段频率和调用耗时](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/pipeline_rates_and_costs.png)

### 6.1 延迟与耗时

| 项目 | 平均 | P95 |
|---|---:|---:|
| 深度收到→融合开始 | 25.83 ms | 44.11 ms |
| 颜色收到→融合开始 | 27.36 ms | 44.71 ms |
| 深度首次 TF 尝试→成功 | 19.72 ms | 43.58 ms |
| 颜色首次 TF 尝试→成功 | 21.30 ms | 43.89 ms |
| 单次 TF lookup 执行 | 0.024 ms | 0.060 ms |
| 深度融合 host 调用 | 1.671 ms | 3.165 ms |
| 颜色融合 host 调用 | 1.304 ms | 2.459 ms |
| ESDF 更新 host 调用 | 1.603 ms | 2.834 ms |
| ESDF 导出及 GPU 等待 | 1.473 ms | 2.493 ms |
| ESDF 打包/发布 | 2.146 ms | 3.500 ms |
| Mesh 更新/回读 | 7.349 ms | 10.615 ms |
| Mesh 打包/发布 | 8.925 ms | 12.490 ms |
| 输入 CameraInfo 时间匹配接收→点云接收代理值 | 48.775 ms | **59.590 ms** |

最后一行用与结果输入时间戳相同的 CameraInfo 到达时间作输入代理，本轮匹配比例 **99.90%**；其 P50=49.73 ms，最大 180.02 ms。它不是实际相机曝光到屏幕的端到端延迟，也不是“ESDF 内核耗时”。

ESDF 更新调用间隔 P50 **31.83 ms**、P95 **49.32 ms**，非空点云接收间隔 P50 **32.37 ms**、P95 **68.64 ms**。所以平均接近 30 Hz 不代表每帧严格等间隔。

![实际阶段时间线](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/pipeline_actual_timeline.png)

### 6.2 为什么还没有稳定 30 Hz

1. **等待 TF，而非查询函数本身特别慢。** 深度约 63.7%、颜色约 65.7% 的融合帧有 TF 重试。图像约 30 Hz，动态 TF 约 20 Hz，按图像时刻插值常需等后一个 TF 样本到达。一次查询平均只有 0.024 ms，但“尚无对应时刻 TF”会等待几十毫秒，并造成随后短间隔连续融合。深度融合间隔小于 10 ms 的比例约 25.8%，大于 40 ms 的约 58.1%。
2. **门控仍会跳过少量帧。** 稳定窗口深度/颜色各有 37 帧 gate_skipped，深度队列覆盖 7 帧。40 Hz 只是最小间隔 25 ms 的门限，输入时间戳局部抖动仍可能触发；不能说输入均值 30 Hz 就必定每帧间隔都大于 25 ms。
3. **同线程还有 Mesh 工作。** 工作线程要完成融合、ESDF、Mesh 更新/回读；5 Hz 的 Mesh 每次仍占数毫秒。输出线程另有 Mesh 打包，与 Python CPU/GIL、ROS 通信和系统调度相互影响。此处有测得开销，但本轮未关闭 Mesh 做隔离，不能把全部频率差都归它。
4. **调度不是理想定时器。** `due()` 以执行时的 `now+period` 安排下次，受上述等待和任务穿插影响；ESDF 还要求有新深度，不会无意义更新未变化地图。33.4 Hz 配置不是保底值。
5. **发布不等于所有接收者都收到。** 接收端 QoS/history、序列化与反序列化、大消息传输和 CPU 处理可能影响实收。不能只凭 28.59→26.17 Hz 就断言是网络丢包。

本次全 GPU 利用率采样均值约 **15.4%**、P95 **29.5%**，包含其他可能的整卡工作且是低频采样，不能当作本节点 CUDA 内核剖析；但现有数据不足以支持“GPU 一直满载”这个解释。

## 7. 问题定位与后续优化顺序

| 阶段 | 已有证据 | 当前判断 | 下一步最有价值的工作 |
|---|---|---|---|
| 原始图像/反投影 | 有效参考像素深度 P95 3.06 mm | 主要表面输入明显优于最终 Mesh，仍有边缘离群点 | 保留逐物体/遮挡边界掩码，防止台面掩盖细结构 |
| 时间对齐/等待 | 查询很快，等 TF 达几十 ms | 实时延迟的重要来源；本轮未证明几何错位由 TF 引起 | 在仿真统一控制与图像时间基准，提高 TF 可用及时性，保留按图像时间查 TF |
| TSDF 表面 | 单帧已经模糊；静止重复只增加权重 | 重点是 2 cm 表示、薄结构与投影截断融合 | 固定输入、精度不变，单因素比较截断/权重；按物体尺寸而非仅凭视觉评分 |
| Mesh | 已检验顶点贴合 TSDF 零面 | 不能指望仅换 RViz 渲染修复几何 | 继续直接 PLY 与 TSDF 一致性检查，不用平滑掩盖偏差 |
| 移动融合 | 覆盖增加，未提升精度 | 自体/遮挡历史、静态假设需进一步隔离 | 用同一轨迹加机器人遮罩；单独验证衰减对残影和完整性的取舍 |
| ESDF 传播/状态 | 原始层 4 个 2 m 异常，附近有可连通种子 | 优先级最高的距离正确性问题 | 比较增量更新和全量重建，导出父方向/区块更新集合，检查跨块及 site 状态转换 |
| 导出/查询 | 原生距离、查询相同；stride 是精确子集 | 本次未发现数值转换错误；显示抽样可漏看异常 | 保留全精度异常扫描，再谈可视化抽样与带宽优化 |
| 发布/显示 | 发布28.59，接收26.17 Hz | 还存在通信/消费开销 | 分离计算、打包、传输和 RViz 四项指标，评估增量/局部输出 |

建议优先顺序：**先处理 ESDF 严重距离异常，再验证 TSDF 薄结构/截断，最后降低 TF 等待和 Mesh/输出开销。** 把 unknown 无条件填成自由空间、只改颜色或只改点大小都不能解决这些问题。本轮不实施这些算法修改。

下一轮验收应同时要求：相同输入下严重距离异常不再出现、逐物体 Mesh 误差改善、unknown 没有被错误转为 free，以及接收 P95 延迟和输出间隔改善；不能只看 Hz。

## 8. 查看数据和复现指令

### 8.1 直接查看本次结果

- [多视角 Mesh PLY](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/multiview/mesh.ply)：CloudCompare 打开即可，开启顶点 RGB。
- [多视角 observed ESDF PLY，stride2](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/multiview/esdf_observed.ply)：CloudCompare 用 distance 标量着色，包含 72,633 点。
- [完整原生 ESDF numpy](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/multiview/esdf_native.npy)：584,170 行，不是 stride 后的点；列为 x/y/z、signed distance、observed、inside、site。
- [完整地图 map.blox](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/multiview/map.blox)：包含 TSDF、Color、ESDF 等地图层；Mesh PLY 独立保存。
- [精度原始汇总](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/quality_summary.json)、[实时性汇总](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/pipeline_summary.json)、[异常原始字段/连通性核对](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/esdf_connectivity_checks.json)。

保存日志存在官方序列化器“不识别 MeshBlockLayer”的输出；Mesh 已单独以 PLY 成功保存，两项服务 success=true，地图也成功加载并导出 TSDF/ESDF。**不要把 .blox 当作已经内嵌全部 Mesh 三角面的文件**，保存 Mesh 时仍单独 save_ply。

### 8.2 编译并回放同一仿真记录，在 RViz 观看

这是复现同一数据的方式，不启动实机、不需要启动 Gazebo、不向机械臂发运动命令。每个终端都使用本工程环境与相同域。开启 RViz 的结果不能直接当成本轮无 GUI 性能值。

终端 1：编译并启动 Mapper / RViz。已有可用构建也可直接 source 环境再 launch。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
source ./env.sh
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1

ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true \
  mapper_config:=/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111/baseline_params.yaml \
  voxel_size:=0.02 truncation_vox:=4.0 \
  depth_hz:=40.0 color_hz:=40.0 \
  esdf_mode:=3d esdf_hz:=33.4 esdf_view_hz:=33.4 \
  esdf_view_mode:=observed esdf_view_stride:=2 \
  mesh_hz:=5.0 input_qos:=reliable \
  decay_hz:=0.0 clearing_hz:=0.0 \
  run_rviz:=true rviz_mode:=diagnostic
```

等日志提示 Mapper ready 后，终端 2 回放一次：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
ros2 bag play /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag \
  --disable-keyboard-controls
```

包里已有 `/clock`；不要叠加另一个 Gazebo 或其他时钟来源。若需要再来一遍，先重启 Mapper，避免倒回的时钟和旧地图干扰比较。

终端 3：核对话题、实收和生效配置：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
ros2 topic list -t
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox truncation_vox
ros2 param get /ur7e_nvblox esdf_view_stride
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

停止 hz 后也可看 `/ur7e_nvblox/mesh`。当前 Python 包 3D ESDF 点云名是 `/ur7e_nvblox/esdf_points`，不要混用旧实现的 `/esdf_3d`。

回放完保持 Mapper 不退出，保存到一个新的目录：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
TEST_OUT="$PWD/artifacts/manual_voxel002_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUT"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/map.blox'}"
ros2 param dump /ur7e_nvblox > "$TEST_OUT/params.yaml"
```

### 8.3 本轮测试脚本与产物位置

测试脚本均位于 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel002_accuracy_pipeline_20261111`：

- `traced_mapper.py`：包装既有调用，补充 host 阶段时间和 TF 重试记录，不替换融合算法。
- `run_test.py`：本轮新实时回放、监测、保存、关闭自己的进程。
- `prepare_inputs.py` / `export_quality.py`：提取静止段，执行单帧/60帧后端实验，读取多视角最终保存地图。
- `geometry.py` / `analyze_quality.py`：几何参考、Mesh 射线比较、TSDF 一致性与 ESDF 精度统计。
- `analyze_realtime.py`：实际计数、时间窗口、等待/耗时和图表。
- `check_inputs_and_outliers.py` / `check_esdf_connectivity.py`：静止输入变化、权重、异常附近种子与已观测连通路径核对。

已有脚本固定了本次输出目录且拒绝覆盖；不要直接反复运行覆盖证据。复测可按上面的手动命令写入新目录。报告内每个数字均可追溯到本次 JSON、PLY、NPY、BLOX 和日志。
