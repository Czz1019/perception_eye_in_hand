# 感知重构、约30Hz实时性、融合问题与仿真测试汇总

后续补充：已重新完成融合门限30/33.4/40Hz各三轮对照，首次补齐融合33.4Hz实测。见[九轮复测报告](FUSION_18HZ_MULTIRUN_RETEST_20261105.md)。以下表格保留其原测试条件和历史数值。

日期：2026-11-05。范围：perception_eye_in_hand项目的感知演进，当前运行主体为 `/home/czz/perception_eye_in_hand/nvblox_in_hand`。本次仅汇总源码和已有实测证据，没有重新运行仿真/性能测试，没有修改程序或感知参数。

## 1. 先明确当前已经做到什么

当前短时Gazebo测试中，640×480 RGB-D、3cm体素、3m融合范围下，深度/颜色进入融合约29.7～30Hz，ESDF更新入口调用约32.5～32.9Hz，附加三维ESDF点云发布约29.2～29.6Hz；Mesh仍约4.74Hz。

“约30Hz”不是一个笼统指标：**图像输入、进入融合、ESDF调用、点云发布、实际新输入关联和RViz绘制是不同环节。** 当前没有证明每秒30份全新完整地图，也没有证明30Hz GPU完成率或实机性能达标。

重建范围在扩大视角后增加，但自体污染、unknown盲区、运动残影、表面模糊与串色仍未解决。高位左右扫描后局部墙面误差P95反而增大，因此应把“吞吐恢复”和“重建质量”分开验收。

## 2. 重构前后代码到底有什么不同

### 2.1 版本划分，避免把已修改的旧文件当原始代码

原仓库基线采用已保存提交 `3ac915bb9adde9f2b9a3717d7332d29501f8eca2` 的只读快照，而不是当前已被多轮修改的ur7e_nvblox源码。父工程原生C++包则按当前文件说明其能力，不把它们倒推为每个历史版本都有的功能。

| 阶段 | 感知主体与组织 | ESDF输出方式 | 主要问题或边界 |
| --- | --- | --- | --- |
| 原ur7e_nvblox基线 | Python TfMapper调度、nvblox_torch后端、latest-only输入和结果槽、独立发布线程 | 默认固定ROI dense查询；查询/读回仍在融合工作链内 | 地图体素和查询间距分离；输出/读回影响下一帧处理；unknown显示易混淆 |
| 父工程ur7e_nvblox_native | C++派生官方ROS节点，保留官方队列/Mapper，并加入原生块导出与专用诊断 | 直接读取原生ESDF块，按ROI/距离带导出，接口含occupied/distance等 | 仍有项目自定义节点、输出协议和调度职责；不是当前默认启动入口 |
| nvblox_in_hand重构前的适配版 | 已接官方ROS/core，但以项目InHandMapper和三维显示适配为主要入口 | 一度默认surface，实际只筛inside点 | inside不等于完整ESDF或精确表面，造成“点都在障碍物内部”的误解 |
| 当前nvblox_in_hand重构版 | 参考workspaces/nvblox_examples分层编排，默认直接加载官方NvbloxNode组件 | 原生3D场在Mapper；官方2D模式提供切片；项目3D导出显式可选 | 保留主体功能，分清计算和显示；几何/颜色输入质量问题仍存在 |

**表的含义：**原仓库底层本来就使用nvblox，不能说以前是假ESDF、现在才是真ESDF。主要重构发生在运行组织、输入适配、查询/显示职责和输出语义，而非重新发明距离传播算法。

当前父工程native_mapper.cpp的processEsdf包含“新深度时间戳未推进则返回”的保护，并有GPU计时相关代码；当前nvblox_in_hand诊断节点主要保留官方更新调用计数，没有把父包全部保护/计时能力迁入。这说明重构不是每一方面都自动更优，统计口径也不能跨包直接等同。

来源：[原基线分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ORIGINAL_PERCEPTION_AND_WEEKLY_CHANGES.md)、[原基线快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/baseline_analysis_20261029/baseline.json)、[父工程原生节点](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_mapper.cpp:206)、[重构前inside语义审计](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_SEMANTICS_AND_SIMPLIFICATION_AUDIT_20261104.md)。

### 2.2 主要实现差异对照

| 方面 | 原基线/旧适配方式 | 当前重构实现 | 实际作用 |
| --- | --- | --- | --- |
| 入口与模块 | 自定义节点承担输入、融合调度、查询和显示等工作 | realsense_example→bringup→sensors/perception/visualization分层 | 运行职责清晰，能核对实际组件/配置来源 |
| 建图计算 | 已有nvblox底层 | 默认官方nvblox::NvbloxNode、MultiMapper/Mapper和CUDA核心 | 保留官方TSDF/Color/ESDF/Mesh算法，不改为另一套自定义数学实现 |
| 位姿 | 原基线已经按图像时刻查机械臂TF | 保留图像时间TF，Mapper直接读取；Pose/Path仅为旁路 | 没有用VSLAM或路径话题替代真实机械臂相机位姿 |
| 图像 | 原TfMapper支持stride降采样；基线stride=2 | 当前Gazebo原生生成640×480，两路独立CameraInfo | 避免混淆传感器尺寸与后端stride；不意味着所有历史测试输入相同 |
| ESDF显示坐标 | 原dense查询坐标由独立grid_resolution生成 | 诊断导出原生体素中心，随实际体素大小和observed集合变化 | 改voxel_size不再只改变底层而保持一套独立显示格点 |
| unknown | 原full_grid分支可输出unknown灰点；默认observed显示另有过滤 | 诊断只导出有限observed值；原生有界服务保留unknown | 没有强行补齐自由空间；点云缺点仍不能单独推断unknown或自由 |
| inside/outside | 旧适配surface曾只输出inside | observed默认保留inside和outside，提供明确标志 | 消除“inside集合等于完整ESDF”的错误语义 |
| 颜色和Mesh | 原TfMapper重点是深度ESDF，其他包已有独立重建能力 | 同一个官方Mapper支持Color与原生增量Mesh | 不应说原整个仓库没有Mesh；当前链路是统一官方主体 |
| 可选诊断成本 | 原基线即使无显示需求也可能进行查询/读回 | 默认官方后端不做项目三维导出；诊断按订阅需求执行 | 减少无关输出开销，但诊断仍扫描块、同步和读回，并非零成本 |
| 功能简化 | 历史应用涉及多任务、多传感器等 | 单D435、机械臂TF；不启动无关人物识别、VSLAM、LiDAR及其他相机入口 | 剔除无关工作流，未删TSDF/Color/ESDF/Mesh核心；自体过滤仍需补齐 |

原dense默认2×2×1.5m、5cm查询间距，共41×41×31=52111个查询点。只改地图voxel_size不改查询间距时，显示采样数量不会相应变化；这不等于底层为全部unknown分配了体素。固定栅格本身可作为规划快照格式，问题是把它当作原生体素显示及在高频融合链上反复查询。

### 2.3 当前代码结构与数据流

```text
nvblox_in_hand/
├── run.sh / run_esdf.sh / env.sh / build.sh
├── src/ur7e_d435_gazebo/       原机器人、场景、D435与控制器副本
├── src/nvblox_in_hand/
│   ├── launch/bringup.launch.py                 总编排
│   ├── launch/sensors/d435.launch.py            可选实机驱动
│   ├── launch/perception/nvblox.launch.py       官方组件/诊断后端与参数适配
│   ├── launch/visualization/                    独立RViz配置选择
│   ├── config/nvblox、robot、profiles           官方基础+相机+机械臂+运行档
│   ├── scripts/robot_pose.py                   图像时间TF旁路位姿/路径
│   ├── src/mapper_node.cpp                     显式可选诊断派生节点
│   ├── src/esdf_pointcloud_exporter.cu         原生observed ESDF体素导出
│   └── rviz/                                  Mesh、2D、诊断3D显示
├── upstream/、provenance/                      官方源码快照和来源记录
├── artifacts/                                 实测数据/地图/截图
└── docs/                                      分析与操作说明
```

实际计算链为“深度+独立CameraInfo+图像时刻TF→TSDF；RGB+独立CameraInfo+其图像时刻TF→现有TSDF上的Color融合；TSDF→ESDF和Mesh；ESDF→可选三维诊断导出”。workspaces中的nvblox_examples主要负责启动编排，真正算法在其调用的nvblox_ros/nvblox_core中。

当前默认构建链接 `/home/czz/workspaces/isaac_ros-dev/install` 的官方ROS/CUDA/NITROS依赖；程序改动在perception工程，不是去修改workspaces。保留源码快照不等于每次构建都重编全部官方核心，也不保证端到端零拷贝。

### 2.4 与workspaces参考档的数值差异

| 参数/条件 | reference继承档 | 当前realtime档 |
| --- | --- | --- |
| 体素 | 0.02m | 0.03m |
| 深度融合门限 | 30Hz | 40Hz |
| 颜色融合门限 | 5Hz | 40Hz |
| ESDF更新门限 | 5Hz | 33.4Hz |
| Mesh/层公共发布门限 | 5Hz | 5Hz |
| TSDF衰减/半径清图 | 2Hz / 1Hz，参考半径7m | 关闭 |
| 当前相机输入/融合范围 | 本项目仍适配单D435和机械臂，不能当成整个workspaces应用原样复制 | 640×480目标30Hz / 3m |

**表的含义：**重构保留了官方实现组织，但当前运行档主动改变了吞吐和分辨率/历史策略。3cm与参考2cm并非同精度，关闭清理也不是与参考完全等同；这些都可能影响质量和长期负载，不能仅凭共用官方代码就承诺相同效果。reference表项是配置，不是本批实测性能。

## 3. 为什么后来能接近30Hz

### 3.1 第一步：融合门限30→40，恢复被主动过滤的图像

本地官方shouldProcess采用严格判断 `图像时间戳差 > 1 / 融合门限`。限频30Hz要求大于33.333ms，但输入相邻时间戳经常为33ms：33ms不通过，下一帧约66ms才通过。于是输入接近30Hz，实际进入融合却只有约18Hz。

门限改40Hz后，要求大于25ms，正常约33ms帧间隔可以通过。恢复的是输入利用率，不是让相机生成40帧，也不是证明CUDA积分核函数速度翻倍。

| 同轮对照指标 | 融合门限30 | 融合门限40 |
| --- | ---: | ---: |
| 实际深度融合 | 18.23Hz | 29.71Hz |
| 实际颜色融合 | 17.96Hz | 29.85Hz |
| ESDF更新调用 | 24.42Hz | 24.58Hz |
| Mesh发布 | 4.75Hz | 4.74Hz |
| 深度回调/进入融合计数 | 653 / 401 | 655 / 654 |
| 颜色回调/进入融合计数 | 650 / 395 | 657 / 657 |

**表的含义：**更多已收到的图像参与融合；ESDF仍约25Hz，说明融合限频问题和ESDF节拍问题不同。计数快照端点有约1帧边界差，不能把比值写成严格零丢帧证明。

### 3.2 第二步：ESDF门限30→33.4，绕过10ms tick量化

ESDF由约10ms主tick检查；30Hz要求间隔大于33.333ms，通常到约40ms才满足，即约25Hz。33.4Hz门限对应29.94ms，名义30ms时已可满足，因此更新入口调用接近33Hz。

这次没有实现独立相位累加定时器，没有改距离传播算法；33.4只是当前节拍下有效的参数余量，不保证其他负载条件下精确30Hz。

| 同轮配置 | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | 三维点云Hz | Mesh Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| 融合40、ESDF30、显示30，诊断 | 29.62 | 29.57 | 24.62 | 24.58 | 4.75 |
| 融合40、ESDF33.4，官方组件 | 29.78 | 29.87 | 32.78 | 不提供项目导出 | 4.74 |
| 融合40、ESDF33.4、显示33.4，诊断 | 29.74 | 29.87 | 32.59 | 29.44 | 4.74 |
| 融合40、ESDF33.4、显示40，诊断 | 29.76 | 29.67 | 32.53 | 32.24 | 4.74 |

**表的含义：**核心ESDF调用和三维导出具有各自门限。最后一行消息超过30Hz并没有产生更多传感器图像；官方组件一行没有项目点云不能填为“ESDF没计算”。

### 3.3 第三步：将显示门限从5调高，并明确输出职责

早期诊断点云约4.84Hz，是按当时显示目标5Hz运行，不能解释为“30Hz目标只能达到4.84”。当前诊断显示门限33.4Hz，短时点云发布约29Hz。新增run_esdf.sh只是显式选择diagnostic/3d/导出/诊断RViz的便捷入口，不是新建图算法。

将输入1280×720改640×480减小了单次处理负载，但当时融合仍约18Hz、ESDF仍24.61Hz，说明仅降低分辨率没有解决门限问题。

### 3.4 不能把“约30Hz发布”写成“30Hz全新地图”

| 同轮诊断配置 | 点云发布Hz | 关联深度时间戳推进Hz | 相邻关联时间戳重复比例 |
| --- | ---: | ---: | ---: |
| ESDF30 / 显示30 | 24.58 | 19.91 | 19.0% |
| ESDF33.4 / 显示33.4 | 29.44 | 19.75 | 32.9% |
| ESDF33.4 / 显示40 | 32.24 | 19.82 | 38.5% |

**表的含义：**诊断按ESDF调用版本导出，允许重复关联同一深度时间戳。该推进率由已记录时钟与点云年龄重建，只表明输出关联输入是否推进，不是几何变化率，也不代表只有约20Hz图像进入TSDF；两次输出之间可能融合多张图像。不能把该旧轮的19.75Hz直接套到最新左右扫描轮，最新轮未重复计算该项。

上述三步对照原始报告：[融合限频](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/FUSION_RATE_MARGIN_OPTIMIZATION_20261105.md)、[ESDF与显示门限](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)。

## 4. 接近30Hz的究竟是什么话题、什么内容

下表“最新实测”以高位左右扫描为例；未测项目明确不填频率。

| 话题/指标 | 类型与内容 | 频率或状态 | 注意事项 |
| --- | --- | --- | --- |
| /camera/depth/image_rect_raw | Image；640×480、16UC1，毫米深度 | 输入约29.84Hz | 是原始输入，不是融合结果 |
| /camera/color/image_raw | Image；640×480、rgb8 | 输入约29.90Hz | 同为30Hz不意味着RGB与深度严格一一同步 |
| 两路camera_info | CameraInfo；各自内参/frame | 随输入，未单独统计Hz | 不能把深度CameraInfo替代彩色CameraInfo |
| 官方ros/depth/integrate计数 | 深度进入积分函数次数 | 29.85Hz | 内部统计，不是一个地图话题；不代表GPU完成率 |
| 官方ros/color/integrate计数 | 颜色进入积分函数次数 | 29.78Hz | 不等于每张RGB都有有效新颜色体素 |
| 官方ros/update_esdf或ros/esdf计数 | ESDF更新入口调用 | 32.78Hz | 无独立“每帧ESDF完成”话题；可能没有新块变化 |
| /nvblox_node/esdf_points | PointCloud2；原生observed体素中心及距离/状态 | **29.22Hz**，仅diagnostic | 当前所谓“约30Hz三维ESDF显示”主要指这个话题 |
| /nvblox_node/mesh | nvblox_msgs/Mesh；增量三角网格和RGB顶点色 | **4.74Hz** | 不是30Hz，增量包也不一定含整张网格 |
| /nvblox_node/tsdf_layer、color_layer | VoxelBlockLayer；官方体素层 | 5Hz公共调度目标，未逐话题实测 | 按订阅需求生成，不可直接填30Hz |
| /nvblox_node/tsdf_layer_marker、color_layer_marker | Marker；标准RViz体素显示 | 同属层发布需求，未逐项实测 | 方块显示不是ESDF算法的另一种实现 |
| /nvblox_node/static_esdf_pointcloud | PointCloud2；二维距离切片 | 仅2d模式有效；旧2D轮24.36Hz | 本地3d模式不能据此检查更新；未验证最新参数下2D达到30Hz |
| /nvblox_node/static_map_slice、static_occupancy_grid | DistanceMapSlice/OccupancyGrid；二维输出 | 取决于2d模式和实际发布需求 | 不是完整三维场 |
| /nvblox_node/metrics | String；计数、主机耗时、最近点数 | 约1秒报告，非30Hz地图流 | 仅diagnostic；JSON内Hz不等于metrics话题Hz |
| /nvblox_in_hand/camera_pose | PoseStamped；CameraInfo时刻的机械臂光学TF | 随成功解析输入，最新未单独测Hz | 是TF旁路观察，不是SLAM结果，也不是Mapper位姿中转 |
| /nvblox_in_hand/camera_path | Path；有限长度机械臂相机轨迹 | 代码wall-clock间隔至少0.2s，约≤5Hz目标；未实测 | 不是30Hz路径，也没有额外SLAM算法 |
| /nvblox_in_hand/pose_health | String；TF解析/超时状态 | 代码1秒定时报告 | 不代表融合成功率 |

**表的含义：**目前“达到30Hz”的实质是融合入口吞吐及诊断三维距离点云消息接近30Hz；不是所有话题都30Hz。默认 `./run.sh` 官方组件不提供项目esdf_points/metrics，查看它们用 `./run_esdf.sh`，二者互选，不能同时启动补齐话题。

### 三维点云每个点包含什么

point_step=24字节：xyz坐标、distance/intensity（同一float偏移12）、rgb，以及observed/is_inside/is_site标志。坐标为原生体素中心，距离带符号，unknown不填充；inside/site可能有零或负零，不能用distance>=0单独判自由。点云没有完整unknown掩码，也没有逐点梯度，不应直接作为完整规划安全契约。

完整有效性需结合原生层/有界服务。服务get_esdf_and_gradient在本地转换中实际提供有界标量距离数组、origin和voxel_size，未见显式xyz梯度通道，不能仅凭接口名宣称拿到完整梯度。大服务请求也有处理/读回成本，测试时放在频率采样之外。

## 5. 已经做过哪些测试

### 5.1 分辨率与后端检查

| 轮次 | 输入 | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | 附加输出 |
| --- | --- | ---: | ---: | ---: | --- |
| 重构后官方3D初测 | 1280×720 | 18.51 | 18.60 | 24.61 | Mesh4.74Hz；不提供项目3D点云 |
| 官方2D初测 | 1280×720 | 18.02 | 18.39 | 24.47 | 二维点云24.36Hz |
| 改640×480、旧门限 | 640×480 | 18.43 | 18.11 | 24.61 | Mesh4.75Hz |
| 640×480诊断检查、旧门限 | 640×480 | 未在此表重复汇总 | 未在此表重复汇总 | 24.78 | 三维点云4.84Hz，显示目标5Hz |

**表的含义：**原生640×480和CameraInfo核对完成，但输入变小并未修复同频限流。1280与640不是同一帧数据回放；以上不能作为严格单变量的硬件性能基准。后续门限对照才具体解释了18Hz→约30Hz。

### 5.2 颜色显示调整检查

| 配置 | 三维距离色带 | ESDF调用Hz | 3D点云Hz | 同一快照的12段色带利用 |
| --- | --- | ---: | ---: | --- |
| 原色带 | 0～1m，与参考二维上下界一致 | 32.70 | 29.55 | 使用5段，最拥挤一段约60.62% |
| 当前三维色带 | −0.09～0.24m | 32.76 | 29.44 | 使用12段，最拥挤一段约18.27% |

**表的含义：**收紧三维距离范围使颜色区分更明显，未改变ESDF值、点数规则、几何或Mesh真实RGB；二维显示仍保留0～1m。色带利用率是在同一份点云上算的，不是屏幕像素统计，不能当成重建质量指标。

### 5.3 相机上方扫描与高位左右扫描

所有下列轮次保持640×480、3cm、3m和相同融合/ESDF门限，独立新地图；运动幅度、时长、相机高度与窗口交互有所不同。它们是短时重建验证，不是仅改变一个变量的严格性能因果实验。

| 轮次 | 实际动作 | 目标动作时长 | 高度和方向 |
| --- | --- | ---: | --- |
| 上方轻转 | wrist_1±0.04rad，约±2.3° | 25秒 | 约0.678～0.686m，朝下 |
| 扩大转角v2 | wrist_1±0.08rad，约±4.6° | 30秒 | 约0.674～0.690m，方向总跨度9.14° |
| 扩大转角v3 | wrist_1±0.16rad，约±9.2° | 40秒 | 约0.666～0.699m，方向总跨度18.28° |
| 抬高+左右扫描 | URDF逆解六关节协调，Y±0.08m | 40秒 | 约0.82m，X/Z目标固定、光轴朝下 |

**表的含义：**前三轮的单关节转动带来前后/上下耦合；最后一轮按要求改为真正的左右平移，经过逆解和碰撞检查，不是只转动RViz视角。目标时长不含进入/返回等耗时，运动统计阶段约32～47秒。

| 轮次 | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | 3D点云Hz | Mesh Hz | 点云输入年龄P50/P95 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 上方±2.3° | 29.79 | 29.73 | 32.69 | 29.64 | 4.75 | 51/79ms |
| 上方±4.6° | 29.82 | 29.79 | 32.92 | 29.37 | 4.75 | 57/86ms |
| 上方±9.2° | 29.96 | 29.85 | 32.74 | 29.40 | 4.74 | 48/75ms |
| 抬高+左右16cm | 29.85 | 29.78 | 32.78 | 29.22 | 4.74 | 51/79ms |

**表的含义：**相机运动时吞吐仍接近输入；输入年龄不是屏幕端到端延迟，不能与前面官方“滚动均值的中位数”混在一起比较。此处没有逐次测GPU完成时间。

| 轮次 | ESDF点数：前→后 | Mesh三角形：前→后 | 墙面误差P95：前→后 | 墙面接近率：cube1 / cube2（前→后） |
| --- | --- | --- | --- | --- |
| 上方±2.3° | 14747→16997 | 3010→3359 | 36.97→37.04mm | 39.49→40.09% / 35.61→36.26% |
| 上方±4.6° | 14747→19394 | 3010→3742 | 36.97→37.60mm | 39.49→40.42% / 35.61→37.24% |
| 上方±9.2° | 14747→25233 | 3010→4689 | 36.97→37.60mm | 39.49→41.14% / 35.61→37.69% |
| 抬高+左右16cm | 24184→33352 | 4306→6156 | 35.00→49.94mm | 54.77→58.26% / 48.84→58.67% |

**表的含义：**点和网格规模随视角变化增加，几何误差没有同步改善。最后一轮的扫描前基线已在高位，不能把24184与低位14747差值全部当作左右扫描收益。墙面误差只在已知两墙邻域按网格顶点计算；接近率是参考表面采样点3cm内接近网格的比例，包含背面，不是整个场景面积精度或可见面覆盖率。

高位左右扫描的实测方向核对：

| 轨迹指标 | 实测 |
| --- | ---: |
| 相机中心高度 | 0.8200m，比原位姿高约13.8cm |
| 左右总跨度ΔY | 16.00cm |
| 前后残余ΔX | 1.977mm |
| 上下残余ΔZ | 0.072mm |
| 光轴相对起始方向最大偏移 | 0.0316° |

**表的含义：**已基本满足“抬高、主要左右晃动”，不再由单关节俯仰同时带来明显前后摆动。残余位移是实际仿真TF采样，不是只列期望值；世界Y左右也不必等同于任意RViz视角的屏幕水平方向。

## 6. 融合中遇到的问题及其代码原因

| 问题/现象 | 当前代码和证据 | 判断与改善方向 |
| --- | --- | --- |
| 固定点阵、改voxel_size显示点数不变 | 原dense输出由独立5cm查询网格决定 | 新诊断改原生体素中心；固定查询仍可用于有界规划快照 |
| ESDF点只在内部 | 旧surface筛选实际inside-only | 已改observed默认，inside/site/表面明确区分 |
| unknown盲区、墙角空白 | TSDF/ESDF只有已观测与有效数据；遮挡/视场外未覆盖 | 仍存在；多视角补观测、规划保留unknown约束，不能填零冒充自由 |
| 输入30但融合约18Hz | 严格同频时间门限过滤正常33ms帧 | 已改融合门限40，短时融合恢复约30 |
| 融合30但ESDF约25Hz | 10ms tick与33.333ms门限量化 | 已用33.4门限绕过一部分限制；未实现相位累加调度 |
| 发布30但新关联时间戳推进更低 | 按ESDF调用版本导出，存在重复关联输入 | 尚未修复；需要逐帧/脏块关联，避免只追求消息Hz |
| 运动残影、自体混入墙和地面 | static_tsdf，机器人进入RGB/深度，无自体遮罩；衰减关闭 | 仍存在；优先自体几何过滤，再设计历史清理 |
| 表面粗糙、薄墙/棱角模糊 | 3cm体素；10cm墙仅约3.3格，TSDF截断4格=12cm | 几何细节受限；修正输入后再比较更细体素与合理表面带 |
| 不同表面串色 | 颜色依赖已有TSDF的合成深度/可见性；双线性采样、体素颜色加权平均 | 几何错误会传递成颜色错误；先几何/遮挡，再颜色置信与边缘处理 |
| 颜色块状或发灰 | Mesh取近邻颜色体素，缺颜色块存在默认颜色；不是高清纹理图 | 不等于所有RGB输入失效；需区分平均色、缺着色和光照 |
| 时间/外参是否错误 | 静态15mm RGB基线与当前TF链初步相符；节点按图像时刻查TF；Gazebo回调用SimTime标记帧 | 没有直接证据认定外参或RGB/BGR错误；渲染时间对应仍需后续核验 |
| 墙与地面连成一张Mesh | 场景本来接触/相交，TSDF无物体实例身份 | 可能是正常几何连通；悬空膜片、跨空隙桥接与颜色越界才需进一步定位 |

**表的含义：**当前完成的是结构、输出语义和节拍优化，质量问题并未全部解决。3cm/12cm尺度不能单独解释墙间约40cm空隙全部被误填，遇到这种现象应区分真实地面、自体残影与错误几何。

颜色数据流为“RGB投影→从TSDF生成合成深度→遮挡/表面门限→颜色插值→历史ColorVoxel平均→Mesh顶点取色”。当前颜色合成深度射线降采样默认4，边界处存在串色风险；该成员与普通view-calculator的raycast_subsampling_factor不同。并且本地Mapper参数设置未同步向颜色积分器设置截断距离，当前两者默认都4格，但未来仅缩小TSDF截断不能假设颜色带自动同步缩小。

这些是代码机制分析，不是已完成的误差归因实验。不能由51ms输入年龄直接认定有51ms位姿错配，也不能仅凭URDF中的noise/stddev=0.02认定实际深度噪声标准差必为2cm。

完整问题分析：[表面模糊与串色](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/RECONSTRUCTION_BLUR_COLOR_BLEED_ANALYSIS_20261105.md)。

## 7. 已完成、未完成与接下来的优先级

| 类别 | 当前状态 |
| --- | --- |
| workspaces示例主体接入与分层 | 已完成；默认官方组件，诊断可选；单D435/机械臂TF/原场景 |
| 原生TSDF/Color/3D ESDF/Mesh及地图服务 | 已接入并在仿真中运行；没有把显示点冒充Mesh |
| 输入640×480、独立内参/光学frame核对 | 仿真已验证；不等同于实机标定验收 |
| 融合门限与ESDF/显示门限优化 | 已修改并做短时对照；未修改官方shouldProcess算法 |
| 距离配色与专用三维入口 | 已完成；Mesh真实颜色问题未因此修复 |
| 上方转动、扩大范围、高位左右轨迹 | 已执行仿真；高位左右路径通过URDF/FCL约束检查 |
| 自体过滤、残影和错误颜色治理 | 尚未完成 |
| 新观测有效ESDF更新率、GPU完成率、屏幕端到端延迟 | 证据不完整；不能以消息Hz替代 |
| 真实D435/真实机械臂 | 有入口配置；本批测试没有实机验收 |
| 所有区域unknown消除或完整重建 | 未实现，也不应无观测强行补齐 |

建议后续顺序：先做URDF自体过滤与图像时间/投影核验，区分静止逐点融合和运动融合；再改颜色边界/遮挡置信、核对TSDF和颜色表面带；随后在相同区域真值上比较体素细化与覆盖。实时性方面则另行跟踪输入版本/变化块和GPU完成，避免只把调用/导出频率推高。

## 8. 实际截图

原始结构对照图是说明性图，不是实测截图；此处仅放已有真实运行窗口。

### 当前高位左右扫描的ESDF

![高位左右ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_stopped.png)

### 同一轮Mesh重建

![高位左右Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/mesh_after.png)

Mesh显示关闭了URDF机器人外观模型，但地图里重建出的机器人自体仍保留，不能将它误当作只是RobotModel叠加。图中局部缺失、不平整和自体污染仍是当前问题；RViz右下角约31FPS不属于ESDF测量结果。

### 同一轮相机输入

![高位左右RGB](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_wrist_motion_rgb.png)

截图视角可能受交互影响，跨轮不做逐像素精度比较。

## 9. 最短查看/核对指令

本次未执行下面指令。先停止同域旧场景与Mapper，以下两种启动互选，不要同时执行。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
# 官方主体、默认看Mesh
./run.sh gazebo_gui:=true
# 或：项目三维ESDF诊断点云与显示
# ./run_esdf.sh gazebo_gui:=true
```

监测终端：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
ros2 topic info /nvblox_node/esdf_points --verbose
ros2 topic hz /nvblox_node/esdf_points --wall-time
# Ctrl+C后，诊断后端可查看状态
ros2 topic echo /nvblox_node/metrics --once
```

esdf_points与metrics仅diagnostic后端存在；不要因为默认官方组件没有它们就另起第二个Mapper。完整启动、保存统计与运动复现分别见下方操作文档。路径为机械臂TF旁路，没有单独的slam_path话题，不能仅改名宣称完成SLAM。

## 10. 测试口径、来源与边界

本汇总仅从既有报告和JSON读取数值，未重新测试：

1. 核心融合/ESDF率主要来自原有官方timer累计样本数差除墙钟区间；不新增生产统计，不等于CUDA完成率。
2. 点云/Mesh率按消息接收时间；订阅会产生发布负载，窗口开关和跨轮实时仿真存在差异。
3. 官方滚动率、区间率、输入年龄P50/P95、滚动均值中位数是不同统计，本文不将它们混为同一延迟或频率。
4. 有界ESDF查询、地图保存与最终Mesh截图安排在频率采样之外；初始Mesh保存的短间隔包含在部分核心差分区间中。
5. 20项Python及既有C++策略检查在此前相关构建轮通过；独立GPU导出检查在早期重构验收中运行，最近仅参数/显示/运动实验没有每次重跑。不能写成本汇总重新做过全部测试。
6. 当前旧包源码已经改过，描述原基线以保存快照为准；不能用工作区当前node.py否认或替代原始固定查询逻辑。

主要阅读与复现索引：

- [原仓库结构、基线查询开销和已有优点](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ORIGINAL_PERCEPTION_AND_WEEKLY_CHANGES.md)
- [workspaces示例主体重构说明](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/WORKSPACES_EXAMPLE_REFACTOR_20261104.md)
- [1280×720初测](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/REFACTORED_EXAMPLE_TEST_20261104.md)
- [640×480复测](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/INPUT640_3D_ESDF_TEST_20261104.md)
- [融合限频30→40的同轮对照](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/FUSION_RATE_MARGIN_OPTIMIZATION_20261105.md)
- [ESDF33.4Hz、显示门限和重复关联时间戳](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)
- [发布入口及密集距离色带](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)
- [上方±2.3°](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/OVERHEAD_RECONSTRUCTION_TEST_20261105.md)、[±4.6°](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/OVERHEAD_ROTATION_RECONSTRUCTION_V2_20261105.md)、[±9.2°](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/OVERHEAD_WIDER_SCAN_V3_20261105.md)
- [抬高0.82m并左右16cm扫描](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/RAISED_LATERAL_RECONSTRUCTION_TEST_20261105.md)
- [几何模糊/串色原因分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/RECONSTRUCTION_BLUR_COLOR_BLEED_ANALYSIS_20261105.md)
- [逐终端操作指南](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_COMMAND_TEST_GUIDE_20261105.md)、[参数话题契约](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/PARAMETERS_AND_TOPICS.md)

历史单项报告的“当时默认值”和“当时未测试”保留历史意义；当前参数以实际启动合并后的runtime dump与代码为准，不以旧报告中的30Hz/5Hz值覆盖今天的33.4Hz门限。
