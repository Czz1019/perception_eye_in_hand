# 5 mm体素 Mesh / 3D ESDF：精度、实时性与阶段问题定位

日期：2026-11-11。对象：**ur7e_nvblox_in_hand**，Python ROS接入，nvblox_torch中的nvBlox C++/CUDA计算。不是旁边的nvblox_in_hand C++ ROS工作区。

## 1. 结论：必须区分失败条件和完成条件

本轮按用户要求将voxel_size设为 **0.005 m**，完成单帧、静止60帧、多视角的Mesh/ESDF精度检查及正常ROS数据流实时性测试。**未修改生产代码、源配置默认值或仿真场景**；新增测试配置、脚本、数据和文档，没有驱动实机。

首先原样尝试前两轮的 **3m融合范围、unbounded工作区**。5mm运行在回放中途CUDA显存分配失败，没有完成全程、也未成功保存最终Mesh/地图，不能当作有效全程精度测试。

随后明确限定覆盖实验台及相机作业空间的工作区，保留同一输入和轨迹完成5mm测试，并额外完成**同边界1cm对照**，避免将裁剪工作区收益误当成体素分辨率收益。

| 条件 | 状态 | 能说明什么 |
|---|---|---|
| 5mm、3m、无界地图 | **CUDA显存不足失败** | 原配置直接迁移到5mm的资源上限问题 |
| 5mm、3m、实验台边界 | **全程完成** | 本文5mm分阶段精度和实时性结果 |
| 1cm、相同实验台边界 | **全程完成** | 同工作区下的最终精度和实时性对照 |

完成条件下的主要结果：

- 5mm最终Mesh三视角汇总轴向误差MAE **2.21mm**、P95 **3.56mm**；去台面物体区域P95 **18.51mm**，仍不能把全场景统计当作每个细小物体的抓取精度。
- 5mm最终已观测自由空间ESDF误差MAE **2.59mm**、P95 **4.79mm**，ROI unknown **11.16%**。
- 5mm深度融合调用 **26.21Hz**，3D ESDF更新/发布调用 **23.38Hz**，订阅端实际接收 **22.97Hz**；延迟代理P95 **172.17ms**。
- 同边界1cm对应深度融合 **29.35Hz**、ESDF更新 **28.12Hz**、实际接收 **26.55Hz**，延迟代理P95 **63.96ms**。
- **严重距离异常未解决**：最终5mm地图有33个已观测自由体素输出+2m，最大误差约 **1.993m**。检查的6个最大误差饱和点均父方向为0，且能以1–3.5cm的已观测路径到达site，不能只用unknown遮挡解释。

这是仿真记录回放结果，实时性关闭GUI并保持ESDF和Mesh订阅，不包含相机曝光、实机驱动和RViz渲染。每种实时条件各运行一次，不是多轮置信区间。当前自己的Mapper、回放、监测、离线GPU导出均已结束。

## 2. 相同输入及生效参数

### 2.1 数据和参考场景

- 同一640×480仿真记录：[input_bag](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag)，含Depth/RGB、各自CameraInfo、TF、关节及/clock。
- 同一原始初始/左/右视角：[views](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/views)。
- 同一静止60帧：[static_inputs.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/static_inputs.json)，与上一轮逐字节一致；60深度+60颜色，跨度约1.965s，相机漂移约0.00185mm/0.000123°。
- [lab_scene.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/lab_scene.yaml)：实验台、试管架、瓶子、托盘和尺寸标块。
- 域163隔离，启动前/clock、/tf、深度输入和Mapper metrics发布者均为0。
- 实际核心是固定e4a2b00提交wheel的C++/CUDA库，未链接workspaces的nvblox_ros。沿用本工程env.sh和原有构建，没有更换驱动、CUDA或wheel。

### 2.2 完成条件的参数

5mm实际配置：[voxel0005/params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/voxel0005/params.yaml)。1cm实际配置：[voxel001/params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/voxel001/params.yaml)。

| 参数 | 5mm | 同边界1cm |
|---|---:|---:|
| voxel_size | 0.005m | 0.01m |
| truncation_vox | 4 | 4 |
| 物理TSDF截断距离 | **20mm** | **40mm** |
| ESDF site门限 | 2体素=10mm | 2体素=20mm |
| esdf_view_stride | 2 | 2 |
| 显示格点间距 | **10mm** | **20mm** |
| depth_hz / color_hz | 40 / 40 | 相同 |
| esdf_hz / esdf_view_hz | 33.4 / 33.4 | 相同 |
| mesh_hz / layer_hz | 5 / 5 | 相同 |
| 输入 | 640×480、约30Hz、reliable | 相同 |
| 深度范围 | 0.1–3m | 相同 |
| max_weight | 5 | 相同 |
| weighting_mode | inverse_square_tsdf_distance_penalty | 相同 |
| esdf_mode / view_mode | 3d / observed | 相同 |
| ESDF最大距离 / 最小权重 | 2m / 0.1 | 相同 |
| mesh_min_weight / layer_min_weight | 0.1 / 0.1 | 相同 |
| raycast_subsampling_factor | 4 | 相同 |
| decay_hz / clearing_hz | 0 / 0 | 相同 |
| input_queue_size / max_input_age_sec | 3 / 0.5s | 相同 |
| workspace_bounds_type | bounding_box | 相同 |
| workspace x | −0.10…1.10m | 相同 |
| workspace y | −0.60…0.60m | 相同 |
| workspace z | −0.05…1.05m | 相同 |

工作区参数作用于区块选择，边界附近可能按体素块扩展，不表示逐体素精确裁切。它保留评估区域与桌上物体，减少地面、远背景和无关自由空间分配；不等于将最大融合距离3m改短。

**公平性边界：** 本轮同工作区、同输入、同轨迹，但物理截断距离、site尺度和显示间距随体素变化，实时接入保留下来的帧集合也略不同。所以它比较的是两套实际配置，而非“只有分辨率变化、其他物理量完全相同”的严格单变量实验。其他用户进程未被关闭，整机背景负载也不能保证完全一致。

## 3. 无界5mm失败：发生了什么

### 3.1 可核实记录

[失败日志](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/unbounded_failed/mapper.log) 最后记录：

```text
CUDA error = 2 ... unified_vector_impl.h:231
cudaMallocAsync(&new_buffer, sizeof(T) * capacity, cuda_stream)
Error string: out of memory
```

- 失败前metrics窗口约 **49.04s**，深度/颜色接收仍约30Hz，融合调用约 **2.51Hz**。
- 最后接收点云约 **3,371,192点**，仅按24字节/点，载荷已约81MB/条。
- 整卡显存采样峰值 **7,275MiB**，GPU总显存8,188MiB；失败申请还需要额外连续/临时空间，不能因采样未到100%就否认显存不足。
- 没有完整最终地图、Mesh或完整phase_trace；该频率是失败前短窗口，不应与完成测试的稳定窗口混为一组。
- CUDA直接终止进程，最后一次metrics中的error字段为空不代表程序最终成功。

### 3.2 问题阶段

细体素增加区块、TSDF/ESDF及临时导出缓冲规模；同一工作体积从1cm变5mm，理论格点数可增长到约8倍。无界输入还把不需要的地面和背景纳入。

当前桥接每次导出遍历全层，并创建/扩容设备与CPU缓冲；同时地图、颜色、Mesh和ROS大消息占用各自内存。日志只能确认`cudaMallocAsync`在动态数组申请失败，**没有调用栈，不能精确认定是哪一个特定导出数组导致最后一次申请失败**。GPU哈希容量扩张日志也不是实际有效块数量，不能据此直接计算所有有效体素。

因此：5mm不能只改voxel_size而不管理工作范围、地图生命周期和输出负载。限定实验台工作区是一项明确测试条件，不是隐瞒失败或降低输入分辨率。

## 4. 分阶段测试和精度定义

| 阶段 | 实际操作 | 定位目的 |
|---|---|---|
| A 原始输入 | 三个记录视角的深度与场景几何射线比较 | 检查输入表面 |
| B 单帧 | 空地图融合1帧深度和首帧颜色 | 判断不经过运动时是否已有细节损失 |
| C 静止60帧 | 相同停留段，逐帧融合60深度+60颜色 | 检查权重累积和重复观察的作用 |
| D 多视角 | 原ROS节点完整回放并保存 | 实际接入、补视角和实时性 |
| E 原生状态核对 | TSDF零面、原始ESDF、查询/stride和父方向 | 区分地图层问题与发布/显示问题 |

B/C直接调用同一个Backend，不经过正常限频/队列；只用于质量隔离，不能把其速度称为实时Hz。D采用实际节点逻辑。

Mesh按记录深度相机内参、位姿对保存PLY作射线求交，与场景真值比较，单位是**相机轴向误差**。仅在有输入深度、真值且排除明显前景遮挡的有效区域统计。机器人未包含在参考几何中，前景排除不是完美的历史自体掩码；边缘和遮挡仍可能影响尾部。圆柱/球等参考有网格近似，不是实物高精度计量。

ESDF评估物理体积x=[0.16,1.0]、y=[−0.5,0.5]、z=[0,0.4]m：5mm查询 **2,688,000个体素中心**，1cm查询336,000个。只对真值自由空间且observed的位置评价距离；unknown单列。不同分辨率的中心/样本集合不同。查询是离线评估工具，不是在线铺点建图。

原始深度沿用相同373,340个有效参考像素，MAE1.66mm、P953.06mm；少量边缘离群仍存在，最大约256mm。

P95是绝对误差的95%分位数，不是最大误差，也不是准确率。大面积台面容易主导整体统计，所以另列物体区域。

## 5. Mesh与TSDF结果

### 5.1 同一initial视角的阶段比较

| 阶段 | Mesh MAE | P95 | 去台面物体P95 | 覆盖率 | 全图顶点 / 三角面 |
|---|---:|---:|---:|---:|---:|
| 单帧 | 1.38mm | 2.53mm | 8.07mm | 95.61% | 93,786 / 144,335 |
| 静止60帧 | 1.38mm | 2.53mm | 8.06mm | 95.62% | 93,802 / 144,341 |
| 多视角结束 | 1.75mm | 3.16mm | 11.02mm | 99.61% | 129,766 / 201,190 |

最终三视角合并：MAE **2.21mm**、P95 **3.56mm**、覆盖率 **99.50%**；物体区域MAE **4.64mm**、P95 **18.51mm**，最大约256mm。整体几何明显改善，但不能从3.56mm宣称每个试管或夹缝都有这个精度。

![5mm Mesh分阶段](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/mesh_stage_comparison.png)

图中每行是初始/左/右查看位置，每列是原始RGB/单帧地图/静止60帧地图/最终多视角地图。同一列三张Mesh是**同一份地图的不同查看角度**，不是在三个位置各自重建一份单帧地图。原始RGB与深度视角存在外参视差，不能直接当逐像素纹理真值。

静止60帧确实增加了TSDF权重：全图中位权重从单帧约0.298增至5.0；权重达到5的体素比例从0.71%增至100.00%。因此表面变化小不能等同于融合未执行；相同静止深度重复也不能消除所有离散/系统性误差。

### 5.2 同工作区1cm最终对照

| 最终指标 | 5mm | 同边界1cm |
|---|---:|---:|
| initial视角Mesh P95 | 3.16mm | 13.28mm |
| 三视角Mesh MAE | 2.21mm | 4.24mm |
| 三视角Mesh P95 | 3.56mm | 16.61mm |
| 三视角物体区域P95 | 18.51mm | 55.20mm |
| 三视角表面覆盖率 | 99.50% | 99.61% |

更细分辨率改善了可见几何细节，但覆盖率并未必提高；实际观测、丢帧、表面孔洞和尺度门限仍影响覆盖。

![同边界5mm与1cm最终Mesh](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/mesh_5mm_vs_10mm.png)

### 5.3 Mesh是否忠实于TSDF

为了控制细体素分析内存，5mm一致性检查限定在评估ROI内的Mesh顶点，使用周围8个有效TSDF体素插值；前轮部分报告检验全图顶点，比例不能直接横向当作算法优劣。

| 阶段 | ROI顶点数 | 有8邻居且完成检验 | ROI检验比例 | 顶点处TSDF残差P95 |
|---|---:|---:|---:|---:|
| 单帧 | 61,553 | 56,263 | 91.41% | 0.000146mm |
| 静止60帧 | 61,580 | 56,286 | 91.40% | 0.000146mm |
| 多视角结束 | 89,393 | 82,951 | 92.79% | 0.000184mm |

已检验顶点紧贴TSDF零表面。微小残差只是地图与Mesh一致性，不是对真值的亚微米精度；边界未检验顶点不作保证。若仍看到几何阶梯，不应仅归咎于RViz。

![5mm TSDF截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/tsdf_stages.png)

### 5.4 上一轮瓶盖颜色问题是否自动解决

没有。按前一轮相同的高饱和彩色候选规则读取5mm PLY，单帧713个候选中111个低于对应真实盖底超过20mm，多视角1048个中219个超过20mm。

这不是语义分割准确率、表面积或跨分辨率点数优劣指标；它只证明**现象仍存在于保存数据中**。几何P95明显改善，不代表颜色对应关系已正确；截断缩到20mm也没有自动消除数厘米的候选颜色下移，不能只用“体素粗/带宽宽”作为全部解释。后续应继续核对合成深度、遮挡、RGB投影和体素取色。

## 6. 3D ESDF精度和unknown

### 6.1 分阶段结果

| 阶段 | observed / 2,688,000 | unknown | 自由空间MAE | P95 | 最大误差 | 饱和+2m / −2m体素 |
|---|---:|---:|---:|---:|---:|---:|
| 单帧 | 1,963,881 | 26.94% | 2.85mm | 6.15mm | 2009.50mm | 28 / 1 |
| 静止60帧 | 1,963,909 | 26.94% | 2.86mm | 6.13mm | 1999.19mm | 27 / 0 |
| 多视角结束 | 2,388,100 | 11.16% | 2.59mm | 4.79mm | 1992.50mm | 33 / 0 |

多视角最终：

- 原生observed体素 **5,489,002**；stride2导出 **690,127点**，显示格点间距1cm。
- 已观测自由空间样本2,350,810个；误差>10cm有522个，约 **0.0222%**。
- 真值自由距离≥2cm却标作内部有5个。普通近表面离散符号偏差、孤立体素和严重距离高估应分开分析。
- 同边界1cm ESDF MAE **5.41mm**、P95 **9.70mm**、unknown **10.37%**，最终正饱和自由体素2个。

![原生3D ESDF近表面点带](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/esdf_3d_observed_band.png)

图来自真实stride2导出，绘图只选ROI内绝对距离≤8cm，并额外抽样控制画图点数；未生成固定空间假点，也未填unknown。该图为查看结构，**距离异常被8cm筛选排除，不能用它证明所有距离正确**，下一节单独展示异常。

![5mm 3D ESDF截面及误差](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/esdf_accuracy_stages.png)

### 6.2 unknown具体发生在哪

unknown意为没有可靠观测，不能当作free。当前kIgnore策略不会穿越未知体素传播。真实遮挡、未覆盖、无效/范围过滤和未参与融合都会使体素缺少有效TSDF。

| 阶段 | 无TSDF块 | 有块但权重0 | 正权重但<0.1 | TSDF合格而ESDF未知 | unknown总数 |
|---|---:|---:|---:|---:|---:|
| 单帧 | 380,672 | 343,447 | 0 | 0 | 724,119 |
| 静止60帧 | 380,672 | 343,419 | 0 | 0 | 724,091 |
| 多视角结束 | 97,536 | 202,364 | 0 | 0 | 299,900 |

本轮ROI未知仍全部对应无有效TSDF观测。把权重门限简单调低不能产生不存在的观测；没有发现满足TSDF门限后被漏成unknown的样本。

静止60帧unknown约26.94%几乎不变，多视角后降到11.16%。更细体素不等于能穿透遮挡；它也缩小了物理截断带和site范围，且细分格点的统计集合改变，所以不能要求unknown一定比1cm低。

![unknown阶段变化](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/unknown_stages.png)

### 6.3 严重饱和值：已经定位到原生地图状态

最终33个自由体素输出+2m。对最大误差的前6个读取原始BLOX体素并检查局部已观测连通性：

| 示例位置(m) | 输出 / 参考距离 | 到site的已观测六连通路径 |
|---|---|---:|
| (0.5625,0.3825,0.0075) | 2m / 0.0075m | **0.010m** |
| (0.5575,0.3675,0.0325) | 2m / 0.01138m | **0.035m** |
| (0.5625,0.3825,0.0175) | 2m / 0.0175m | **0.025m** |

检查的6个均为observed=true、parent_direction=[0,0,0]，对应平方体素距离160000，即sqrt(160000)×0.005=2m。它们在保存层中已经存在，不是PointCloud2颜色或ROS转换产生的。

单帧/静止阶段检查的最坏点则是仅1–5个体素的孤立六连通分量，局部没有到site路径，不能将这类点和最终已连通异常归为同一原因。对比说明：某些观测分量后续已与种子连通，但距离/父节点未正确反映这一状态，值得优先跟踪增量传播、跨块更新、site转换和dirty集合。

只检查了每阶段误差最大的6个饱和点，不代表已追踪全部33个的内核执行过程；没有证明某一行CUDA代码必然是根因。

![5mm距离异常点截面](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/esdf_outlier_detail.png)

observed只是观测状态，不保证其距离已有可靠的可用父节点。不能仅因平均误差小或P95小，就让规划器把这种2m值当作准确的安全距离。

### 6.4 查询、导出是否制造了错误

- 原始BLOX按固定EsdfVoxel布局分批读取，避免全量复制导致分析内存膨胀，没有重新计算或修正ESDF。
- 三阶段查询observed与原始层一致，不一致数0。
- 在1,963,881 / 1,963,909 / 2,388,100个共同observed位置，GPU查询与原始保存层距离差0。
- 实际桥接stride2输出与原始层按格点取出的子集逐值相同。

这将本轮严重错误定位到已有地图层及其更新，而非离线评估网格或显示抽样。评估查询共268.8万点，直接Backend每次10万点分批执行；未突破ROS查询服务的20万点限制去改变生产服务。

## 7. 同工作区实时性和阶段耗时

稳定计数窗口约76.08s，去掉回放前10s；点云接收窗口约77.61s。与原始记录频率相同，输入并未为了5mm降低分辨率或帧率。

| 环节 | 5mm | 同边界1cm |
|---|---:|---:|
| 深度输入 | 29.95Hz | 29.96Hz |
| 颜色输入 | 29.94Hz | 29.96Hz |
| 深度融合调用 | 26.21Hz | 29.35Hz |
| 颜色融合调用 | 26.10Hz | 29.44Hz |
| ESDF更新调用 | 23.38Hz | 28.12Hz |
| ESDF发布调用 | 23.38Hz | 28.12Hz |
| Mesh发布调用 | 4.72Hz | 4.97Hz |
| ESDF订阅端非空实收 | 22.97Hz | 26.55Hz |

以上融合/更新是host调用返回的计数，**不是单独CUDA内核完成Hz**；原metrics标明gpu_completion_measured=false。点云实收也不是屏幕刷新频率。

| 延迟与负载 | 5mm | 同边界1cm |
|---|---:|---:|
| 输入CameraInfo到ESDF接收P50 | 135.82ms | 50.83ms |
| 同上P95 | 172.17ms | 63.96ms |
| 同上最大值 | 244.36ms | 189.54ms |
| 深度收到→融合开始P95 | 96.28ms | 44.45ms |
| 最后ESDF点数 | 690,127 | 92,799 |
| 最后消息大小 | 16.56MB | 2.23MB |
| 平均实收载荷 | 353.58MB/s | 55.03MB/s |
| ESDF块数 | 13,872 | 1,958 |
| 全图原生observed体素 | 5,489,002 | 733,819 |

延迟用相同输入时间戳的CameraInfo接收作为输入代理，**不包含真实相机曝光、驱动、RViz渲染**。载荷不是实测网卡吞吐；同机也有序列化、拷贝和反序列化成本。

### 7.1 host阶段耗时

| 阶段 | 5mm平均 | 5mm P95 | 1cm平均 |
|---|---:|---:|---:|
| 深度融合 | 3.03ms | 4.76ms | 1.69ms |
| 颜色融合 | 2.15ms | 3.33ms | 1.09ms |
| ESDF更新 | 9.30ms | 16.97ms | 1.84ms |
| ESDF导出及GPU等待 | 14.89ms | 21.34ms | 1.61ms |
| Mesh更新/回读 | 42.15ms | 52.63ms | 9.22ms |
| ESDF打包/发布 | 28.08ms | 33.95ms | 2.88ms |
| Mesh打包/发布 | 25.62ms | 37.84ms | 11.85ms |

![同边界频率和耗时](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/pipeline_rates_and_costs.png)

工作线程前5项调用的墙钟占用之和约 **89.78%**，同边界1cm约22.40%。这不是GPU利用率，也不包含全部调度/等待；输出线程另行运行，不能把两线程时长简单相加当单帧延迟。

整卡显存采样峰值：完成的5mm约2,846MiB、1cm约1,498MiB；无界失败前约7,275MiB。整卡利用率包含其他用户进程，不能作为节点独占CUDA核函数剖析。

### 7.2 最可能限制实时性的阶段

1. **ESDF更新及全图导出。** 更新平均9.30ms，导出/等待14.89ms。`native/export.cu`调用全设备同步，枚举全部区块，分批压缩、取回计数、同步和回读。核心融合是增量的，但这条显示导出仍每次扫描全层。
2. **Mesh共享工作线程。** 一次更新/回读平均42.15ms、P95 52.63ms，超过30Hz的33.33ms帧预算；虽以约5Hz运行，仍会穿插延迟融合和ESDF工作。
3. **大消息打包/发布。** 点云平均28.08ms，接近一帧预算；Mesh打包另有25.62ms。输出线程/ROS通信及Python调度的竞争可能影响接收与工作队列，尚未单因素关闭某项来量化其独立因果贡献。
4. **输入排队。** 5mm深度收到至首次TF尝试平均58.40ms，而1cm为6.54ms；不是查询函数本身慢。单次TF lookup平均0.068ms，等待首次尝试到成功平均2.79ms。工作更忙，开始查时TF常已到达，所以TF重试减少不能当作延迟改善。
5. **队列覆盖和门控。** 稳定窗口5mm深度/颜色覆盖254/262帧，1cm仅9/2帧；门控跳过30/31帧。输入仍约30Hz，但处理和输出压力使部分观测未参加融合。

![真实线程时间线](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/pipeline_actual_timeline.png)

33.4Hz只是调度上限，不会保证达到30Hz。一次ESDF更新可以覆盖其间多次已完成深度融合，不需要为每张图都输出一次完整地图；但吞吐、延迟和距离正确性仍需分别满足实际需求。

## 8. 与此前1cm/2cm结果怎样比较

此前1cm/2cm是unbounded，不能与本轮bounded结果不加说明地作“纯体素对比”。例如此前1cm ESDF更新23.73Hz，本轮同样1cm、限定工作区后为28.12Hz；工作区/输出负载影响显著，但两次背景负载与融合帧集合也不同，不能把全部差额归给单一参数。

参考结果：

| 条件 | Mesh三视角P95 | ESDF自由空间P95 | ESDF更新 | 接收延迟代理P95 |
|---|---:|---:|---:|---:|
| 本轮5mm、bounded | 3.56mm | 4.79mm | 23.38Hz | 172.17ms |
| 本轮1cm、相同bounded | 16.61mm | 9.70mm | 28.12Hz | 63.96ms |
| 上轮1cm、unbounded | 16.66mm | 9.69mm | 23.73Hz | 145.70ms |
| 上轮2cm、unbounded | 43.23mm | 20.76mm | 28.59Hz | 59.59ms |

前两行才是本轮匹配工作范围的主要对照。所有精度项均有各自observed/可见样本集合与近表面尺度变化的限制。

## 9. 阶段定位与后续优先级

| 阶段 | 本轮证据 | 结论与改善方向 |
|---|---|---|
| 地图范围/内存 | 无界5mm中途OOM，边界后全程完成 | 先管理任务空间和地图生命周期，保留清晰的覆盖边界 |
| 原始输入 | 相同30Hz、640×480 | 未通过降低输入规格获得本轮结果 |
| 单帧TSDF/Mesh | initial P95约2.53mm，优于粗体素，但薄结构/离群仍有 | 表面表示明显改善，剩余问题不能全归运动；按物体细分统计 |
| 静止多帧 | P95/unknown基本不变 | 重复近似相同深度不恢复遮挡；融合次数不是唯一问题 |
| 多视角 | initial覆盖95.61%→99.61%，unknown26.94%→11.16% | 补视角有效，但新增困难表面/动态自体仍会影响尾部 |
| 颜色融合 | 5mm仍有盖下方彩色候选 | 继续隔离RGB投影、合成遮挡深度和颜色容差，不能只缩体素 |
| ESDF父节点/传播 | 已连通site却输出2m且父方向0 | 最高优先级的距离正确性问题：对比增量与全量、跟踪site/区块更新 |
| 显示导出 | 全图扫描、同步与回读平均14.89ms | 后续考虑变化块、局部ROI、复用设备缓冲和减少同步 |
| Mesh与ROS输出 | Mesh回读42.15ms、点云打包28.08ms | 计算更新与显示任务分离，降低全量输出对队列的阻塞 |

不能为了Hz把unknown无条件填free，也不能只隐藏饱和值或把Mesh平滑后当作几何改进。验收应同时要求：严重自由距离高估消失、逐物体误差达标、unknown保留、实收频率和P95延迟满足应用。

本轮证明5mm能明显改善场景几何，但不证明它已可直接用于试管精细抓取。对于夹爪局部接触，仍需更可靠的局部几何与距离有效性判断；33个2m高估点应先解决。

## 10. 查看数据及完整复现指令

### 10.1 保存结果

- [5mm最终Mesh PLY](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/multiview/mesh.ply)。
- [5mm ESDF显示点PLY](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/multiview/esdf_observed.ply)，690,127点，CloudCompare可按distance着色。
- [5mm完整原生ESDF](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/multiview/esdf_native.npy)，5,489,002行，xyz、signed distance、observed、inside、site。
- [5mm完整地图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/multiview/map.blox)。Mesh三角面另以PLY保存，不假定全部内嵌于BLOX。
- [同边界1cm Mesh](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/voxel001/mesh.ply)、[同边界1cm精度](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/matched_1cm_quality.json)。
- [5mm精度](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/quality_summary.json)、[阶段实时性](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/pipeline_summary.json)、[unknown分解](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/unknown_stage_breakdown.json)、[异常核对](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/native_outlier_checks.json)。
- [原无界失败摘要](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/unbounded_failed/failure_summary.json)、[失败日志](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/unbounded_failed/mapper.log)。

### 10.2 编译与回放相同测试，在RViz观看

下面显式使用**完成测试的工作区边界**；仅回放已有仿真记录，不驱动实机、不启动新的Gazebo。打开RViz后多了渲染负载，不能直接照搬无GUI频率。

终端1：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
source ./env.sh
export ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1

ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true \
  mapper_config:=/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111/bounded_params.yaml \
  voxel_size:=0.005 truncation_vox:=4.0 \
  depth_hz:=40.0 color_hz:=40.0 \
  esdf_mode:=3d esdf_hz:=33.4 esdf_view_hz:=33.4 \
  esdf_view_mode:=observed esdf_view_stride:=2 \
  mesh_hz:=5.0 input_qos:=reliable \
  workspace_bounds_type:=bounding_box \
  workspace_min_x:=-0.10 workspace_max_x:=1.10 \
  workspace_min_y:=-0.60 workspace_max_y:=0.60 \
  workspace_min_z:=-0.05 workspace_max_z:=1.05 \
  decay_hz:=0.0 clearing_hz:=0.0 service_timeout_sec:=120.0 \
  run_rviz:=true rviz_mode:=diagnostic
```

Mapper ready后，终端2：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1
ros2 bag play /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/moving/input_bag \
  --disable-keyboard-controls
```

包里已有/clock，不要与另一Gazebo时钟混用。重新播放前重启Mapper，避免保留旧地图或时钟倒回干扰。

终端3检查：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox workspace_bounds_type
ros2 param get /ur7e_nvblox esdf_view_stride
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

停止hz后还可查看`/ur7e_nvblox/mesh`、`/ur7e_nvblox/tsdf_layer`和`/ur7e_nvblox/camera_path`。camera_path来自机械臂TF，不是SLAM估计。

回放结束后保持Mapper运行，保存到新目录：

```bash
TEST_OUT="$PWD/artifacts/manual_voxel0005_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUT"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/map.blox'}"
ros2 param dump /ur7e_nvblox > "$TEST_OUT/params.yaml"
```

### 10.3 脚本与验证边界

所有本轮脚本都在 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/voxel0005_accuracy_pipeline_20261111`。`run_test_unbounded_failed.py`归档失败配置；`run_test.py`为完成的5mm边界测试；`run_matched_1cm.py`为同边界1cm实时性对照；`traced_mapper.py`只包装既有调用记录时间，不替换算法。

`export_quality.py`用同一Backend融合单帧/60帧，并直接按固定voxel结构分批读取保存地图，实际stride2仍调用原桥接导出；`analyze_quality.py`对真实几何与地图评价；`analyze_unknown.py`检查原始观测状态；`check_native_outliers.py`读取父方向并检查局部连通；`analyze_matched_1cm.py`只加载同边界1cm最终地图作精度检查，不重新融合；`analyze_saved_caps.py`诊断彩色候选。

本轮没有修改CUDA核心、深度/颜色接入或生产ROS发布逻辑；没有为了得到好看的效果删除异常或填充未知空间。固定测试目录拒绝覆盖，复测请按上述手动命令保存新目录。
