# 从编译开始的Mesh/3D ESDF仿真测试与融合质量说明

日期：2026-11-09。对象：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

## 1. 本轮做了什么，实际运行哪套代码

已执行当前新工程的`./build.sh`，两个功能包ur7e_d435_gazebo和nvblox_in_hand构建成功，耗时约0.58秒。本次是已有缓存上的Release增量构建，不是重新编译全部官方CUDA核心；build.sh关闭BUILD_TESTING，本次没有运行官方全套单元测试。

之后实际运行Gazebo与RViz：每档重建一张新地图，先静止观察20秒，再执行已验证的高位世界Y方向小幅左右移动20秒并回中。包含接近/回中后，移动阶段统计窗口约27秒。两档为0.03m（当前默认）和0.02m（质量对照），输入均为640×480，RGB/depth目标30Hz，融合限频40Hz，ESDF更新/导出门限33.4Hz，Mesh/层发布5Hz，融合最大距离3m。两档的物理截断带分别12cm、8cm，因而不是所有米制算法尺度相同的纯分辨率实验。

**本轮没有修改生产感知源码或默认参数。** 新增的是独立测试、分析、截图和说明；0.02m通过现有CLI覆盖。没有重启旧感知包，没有恢复已清理的父工程build/install，也没有修改workspaces。每档一次、各两个阶段，属于短时功能与质量检查，不能替代多轮长期压力测试。

### 官方与项目代码的边界

| 部分 | 本轮实际来源/角色 |
| --- | --- |
| 深度TSDF、颜色体素融合、ESDF计算、Marching Cubes Mesh | workspaces中官方nvblox的本机ROS迁移版本；链接其已安装库 |
| ROS主处理 | 官方NvbloxNode的processDepthImage/processColorImage/processEsdf/tick等主链 |
| 测试运行节点 | 项目InHandMapper派生节点：调用上述父类，增加统计和原生3D ESDF导出 |
| /nvblox_node/mesh | 官方Mesh生成、发布及官方RViz插件显示 |
| /nvblox_node/esdf_points | 项目CUDA导出器，遍历原生已观测ESDF体素、压缩输出；不是固定ROI点阵查询 |
| 机械臂、Gazebo场景、D435传感器及图像时刻TF | perception保留下来并适配到新工程的代码 |
| 采样、机械臂小幅扫描、截图、离线统计 | 本项目测试脚本；**不是NVIDIA官方benchmark或官方单元测试** |

不能把“主体算法调用官方库”描述成“整个项目及测试都是未经修改的官方示例”。本机ROS迁移版本、机械臂输入、分辨率和参数，与官方文档图片也不是完全相同条件。

默认`./run.sh mapper_backend:=official`直接加载官方组件；本轮用diagnostic是为了同时量测3D ESDF。每次只有一个Mapper；不是官方、诊断各建一张地图。`run_esdf.sh`显式选择diagnostic，默认`run.sh`则选择official。

## 2. 本轮实际结果

“静止”为高位观察，“左右移动”包含进入小幅运动和回中。统计按墙钟接收时间计算；不是RViz右下角渲染FPS。

| 体素 | 阶段 | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | esdf_points发布Hz | 新输入关联输出Hz | Mesh发布Hz | ESDF点数范围 | 输入年龄P50/P95 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 0.03m | 静止 | 29.19 | 29.75 | 32.79 | **28.77** | 19.48 | 4.82 | 24,184～24,184 | 55/84ms |
| 0.03m | 左右移动及回中 | 29.34 | 29.59 | 32.77 | **28.89** | 19.18 | 4.74 | 24,184～32,489 | 56/85ms |
| 0.02m | 静止 | 29.60 | 29.73 | 32.71 | **29.10** | 19.58 | 4.83 | 71,057～71,057 | 57/92ms |
| 0.02m | 左右移动及回中 | 29.47 | 29.69 | 32.48 | **29.18** | 19.66 | 4.73 | 71,057～95,742 | 57/93ms |

深度输入29.52～29.80Hz，颜色输入29.59～30.11Hz，仿真实时因子约0.998～1.002。没有再次出现稳定输入30Hz、融合只有18Hz的情况。40Hz只是输入时间戳门限，不能产生40Hz传感器数据。

### 指标口径与局限

- **ESDF调用**：调用原有processEsdf的次数；没有测独立CUDA事件完成时间，不能当成GPU完成Hz。
- **esdf_points发布**：实际收到的PointCloud2消息数/墙钟跨度，包含重复关联同一深度时间戳的消息。本轮相邻重复比例约32.3%～33.6%。
- **新输入关联输出**：相邻点云header严格递增次数/接收时间跨度，约19～20Hz。它不是几何变化率，也不能反推深度融合只有19Hz。
- **输入年龄**：接收时最近/clock减点云header，包含时钟采样误差、排队、计算与传输的影响；不是纯计算耗时，也不是屏幕端到端延迟。
- **融合统计**：静止阶段去除开头3秒回调积压后，对已有累计计数作差；运动阶段使用官方timing计数的前后快照作差，服务请求/响应中点作为墙钟区间。后者区间约27.1～27.2秒，与点云统计窗口并非逐毫秒相同。
- 原始PHASE日志中诊断滚动Hz受参数dump等阻塞后排队消息影响，可短时高于输入；本文没有直接平均这些滚动数值，使用analysis.json记录的上述差分口径。
- 3D ESDF没有输出二维static_esdf_pointcloud是当前官方端口模式的行为；不要拿该二维话题没有数据认定3D地图没更新。

本次实际轨迹相机高度约0.81998～0.81999m，X约0.4000～0.4012m，Y约−0.0193～0.1212m，横向跨度约14cm。路径命令上限±8cm，但20秒正弦未走满完整周期，不能宣称实际完成16cm全幅扫描；没有环绕。

### 网格规模与局部检查

| 体素 | 阶段 | 顶点数 | 三角形数 |
| --- | --- | ---: | ---: |
| 0.03m | 静止 | 3,006 | 4,306 |
| 0.03m | 左右扫描后 | 4,212 | 6,013 |
| 0.02m | 静止 | 7,214 | 10,660 |
| 0.02m | 左右扫描后 | 9,745 | 14,697 |

扫描后两档网格规模均增加；0.02m保留更密的几何采样，但仍有缺面、边缘不完整和颜色污染，不能仅凭三角形更多宣称质量验收通过。

对已知墙1顶部内部固定ROI（world X∈(0.28,0.52)、|Y−0.2|<0.025、|Z−0.5|<0.06m）中的Mesh顶点，静止/扫描后的高度RMSE：0.03m为约0.50/0.61mm，0.02m为约0.61/0.61mm，样本分别15/18和28/28点。这只是限定ROI内已存在顶点的平面高度检查，排除了棱角，未对未重建区域计罚；不是全场景毫米级精度证明。墙2同ROI在静止时两档均没有顶点，扫描后才有4/13点，反映可见区域扩展；不能由此给出整墙覆盖率。

因此“Mesh看着模糊”与“所有平面位置都误差很大”不是同一个结论：本轮平面内部可比较平整，但棱角、可见覆盖、低分辨率颜色和活动自体污染仍需改进。

## 3. 真实截图与统一视角的原始网格对比

下面是独立从四份保存PLY绘制的同一world ROI、坐标范围和视角，三角形使用顶点RGB均值，无额外几何平滑；它是离线网格图，**不是RViz截图**，用于减少交互改变视角带来的误判。机器人URDF可视模型没有叠加在这张图中，融合进地图的自体片段仍保留。

![保存Mesh统一视角对比](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/saved_mesh_comparison.png)

### 0.03m静止实际RViz

![0.03m静止实际RViz](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/voxel0p03_r1_static.png)

### 0.03m左右扫描中的实际RViz

![0.03m左右扫描中的实际RViz](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/voxel0p03_r1_measure.png)

### 0.02m左右扫描中的实际RViz

![0.02m左右扫描中的实际RViz](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/voxel0p02_r1_measure.png)

### 0.03m扫描后3D ESDF实际RViz

![0.03m扫描后3D ESDF实际RViz](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/voxel0p03_r1_esdf.png)

实际RViz截图包含RobotModel，其机械臂模型不等于已重建机械臂。窗口在测试中可能被交互旋转/缩放，截图用于确认真实显示和伪影，不能逐像素比较两档精度。RViz日志的fallback rendering来自Humble材质兼容处理，不表示GPU融合换成CPU，也不能凭该日志解释所有模糊。

## 4. 融合策略到底是什么

```text
深度图(16UC1) + 深度CameraInfo + 图像时刻机械臂TF
  → 官方GPU ProjectiveTsdfIntegrator → TSDF体素距离与权重
RGB(rgb8) + 颜色CameraInfo + RGB图像时刻机械臂TF
  → 从当前TSDF进行sphere tracing得到彩色视角合成深度
  → 表面带/遮挡检查 + RGB双线性采样 → ColorVoxel加权平均
TSDF → Marching Cubes零交叉插值 → 三角形Mesh
ColorVoxel → 近邻体素给Mesh顶点着色 → 官方Mesh话题/插件
TSDF → ESDF → 项目导出原生observed体素中心 → esdf_points
```

深度与颜色分别排队、分别按各自时间戳查TF；各自图像与CameraInfo配对，没有强制RGB与深度逐帧硬同步。这是官方支持的异步几何/颜色路径；当内容时刻与TF匹配、环境静态时可以工作，不能把“没有强配对”直接判成错误。

单体素深度更新的主要关系为：

```text
d_obs = measured_depth − voxel_depth
D_new = clip((w_old × D_old + w_obs × d_obs)/(w_old + w_obs), −μ, +μ)
w_new = min(w_old + w_obs, 5)
```

其中μ=4×voxel_size，观测权重使用inverse_square_tsdf_distance_penalty；测量位于负截断带后方则不更新，无效深度不参与正常距离融合。这是沿相机射线的投影距离，不是每条深度观测直接给出全空间精确欧氏距离。颜色也采用历史/新观测加权平均，最大权重5并不意味着饱和后冻结；并没有按物体身份分别建模。

颜色可见性使用TSDF合成深度，不是每张RGB强制绑定同一时刻实测深度。Mesh是体素颜色的顶点着色，不是高分辨率RGB纹理贴图。ESDF彩虹表示距离，与物体RGB颜色是两个含义。

## 5. Mesh模糊、颜色混合的原因与证据边界

| 因素 | 代码/配置事实 | 本轮如何理解 |
| --- | --- | --- |
| 地图空间分辨率 | 3cm体素；10cm厚墙仅约3.3格，2cm时约5格 | 输入640×480并不等于Mesh有640×480个独立细节；粗体素、Marching Cubes插值让棱角变钝 |
| 截断带尺度 | TSDF为4vox，3cm时12cm、2cm时8cm | 是更新/截断尺度，不是12cm高斯模糊；会影响薄物体、遮挡边缘及矛盾观测，不能单独解释所有假连接 |
| 体素颜色融合 | 历史颜色平均，Mesh近邻取色，三角形顶点间渲染过渡 | 灰地面/紫墙/机器人颜色可能混合，不能期待照片纹理清晰度 |
| 颜色边缘可见性 | 合成深度射线默认降采样4，再对合成深度与RGB双线性插值 | 边缘有串色风险；raycast_subsampling_factor与颜色sphere-tracing成员不是同一个参数 |
| 静态地图吸收活动自体 | static_tsdf；没有机器人自体深度遮罩 | 输入RGB明确能看到连杆；扫描时活动自体也会进入地图。不是只删除人物识别就能省略自体过滤 |
| 历史修正不充分 | decay=0、clearing=0，invalid_depth_decay_factor=−1 | 错误/遮挡位置不会自动快速遗忘；后续有效射线仍可能修正，不等于TSDF永久冻结 |
| 少量/边缘观测 | mesh_min_weight=0.1，depth预处理关闭 | 易出现稀疏片段和缺口；单纯提高门限也会增大缺面 |
| 位姿/时间对应 | 官方按图像stamp查TF；插件回调用world SimTime标记图像 | 仍需检查渲染帧内容时刻与stamp；本次没有测得曝光/渲染时间偏差，不能认定已发现外参或时间错配 |
| 仿真传感器噪声 | URDF配置gaussian stddev=0.02 | 只是配置存在，未做噪声注入有效性/逐像素方差专项测量，不能直接断言实测深度噪声就是2cm |
| 显示刷新与几何质量 | Mesh约4.7～4.8Hz，ESDF点约29Hz | Mesh降频影响显示新鲜程度；不能恢复体素分辨率，也不是颜色平均的根本原因 |

两墙之间存在真实地面，不能把地面连接两墙理解为两个障碍物错误合并；当前是一张环境地图，本来不输出分物体网格。unknown区域也不会因提高频率而自动成为已知自由空间。

### 后续优化优先级（本轮未实施）

1. 先做URDF/关节状态驱动的机器人自体遮罩，并保持颜色可见性一致；对比静止地图与活动机器人地图，确认污染来源。
2. 保持同输入、同真实轨迹和范围，以体素大小、米制TSDF/颜色带分别作单变量实验。当前0.02m更密但不是完整解决方案，不建议只继续提高消息Hz。
3. 检查图像内容时刻、header、关节/TF时刻和相机外参；高位静止与移动对照，使用边界重投影误差而非点云年龄替代时序误差。
4. 单独核对颜色sphere tracing降采样、边缘插值、遮挡阈值和颜色权重；当前Mapper参数设置代码未把TSDF truncation参数同样传给颜色积分器，后续修改需显式核对，不能假设一个YAML改变两者。
5. 在有自体过滤后评估衰减/清图及边缘深度过滤；衰减过强可能丢失静态地图，不能直接全局开启当作修复。
6. 固定视图/光照、用不带RGB的Mesh对照彩色Mesh，并建立棱角偏差、表面距离、可见区域完整率和颜色重投影指标；将质量验收与消息发布Hz分开。

## 6. 从编译开始的完整手动运行指令

以下使用本轮隔离域134与端口11563。各终端必须一致；不要在同域同端口同时启动旧Mapper。使用新终端并source本工程env.sh，不要source已删除的父工程install。默认依赖仍在workspaces/install；不需要重新生成旧根目录build_native。

### 终端0：编译

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
```

成功应显示两个包完成；本次build.log是实际成功证据。它只构建新工程src，链接workspaces已安装库；`--from-source`另有独立输出路径，不属于本轮验收。

### 终端1：先启动纯仿真场景

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
source ./env.sh
ros2 launch ur7e_d435_gazebo scene.launch.py gazebo_gui:=false
```

本轮Gazebo GUI关闭、RViz开启。若需要Gazebo窗口，将false改为true；图形负载会不同，频率不保证与表格完全相同。DISPLAY以实际桌面为准。出现端口占用时先确认占用者，不要直接结束其他任务场景。

### 终端2：把机械臂移到本轮高位观察姿态

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
source ./env.sh
/usr/bin/python3 artifacts/mesh_esdf_validation_20261109/view_sweep_isolated.py --check-only
/usr/bin/python3 artifacts/mesh_esdf_validation_20261109/view_sweep_isolated.py --seconds 0
./check_sim.sh
```

这是仿真专用、依赖当前机器人初始姿态与已检查路径的动作；不能直接用于实机。先接近高位再启动Mapper，避免把接近过程中的自体运动写入待评估地图。若当前关节不符合已检查初始姿态或高位中心，脚本会拒绝，不应绕过检查。

### 终端3：启动本轮同参数的Mapper与Mesh RViz

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
source ./env.sh
./run.sh   start_scene:=false run_rviz:=true   mapper_backend:=diagnostic esdf_mode:=3d enable_esdf_view:=true   rviz_mode:=mesh profile:=realtime   voxel_size:=0.03 depth_hz:=40 color_hz:=40   esdf_hz:=33.4 esdf_view_hz:=33.4 mesh_hz:=5
```

第二档先Ctrl+C停止**这个Mapper**，保留场景，回到中心，将voxel_size改为0.02再启动，得到新地图；不要在已有地图上动态修改体素大小。其他参数不变。

### 仅查看官方组件生成的Mesh（可选，不与上一Mapper同时启动）

停止诊断Mapper后，可使用相同场景和参数、直接加载本机官方ROS组件：

```bash
./run.sh start_scene:=false mapper_backend:=official rviz_mode:=mesh esdf_mode:=3d voxel_size:=0.03 depth_hz:=40 color_hz:=40 esdf_hz:=33.4 mesh_hz:=5
```

该入口不发布项目的esdf_points和metrics；官方ESDF仍在内部更新。本轮新采样使用的是前述diagnostic派生节点，没有将这条可选命令描述成另一次已完成的官方组件性能对照。

### 终端4：监测实际频率和配置

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
source ./env.sh
ros2 param get /nvblox_node voxel_size
ros2 topic list -t
ros2 topic echo /nvblox_node/metrics --once
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

最后一条持续运行；Ctrl+C后分别运行其他频率检查。ESDF导出按订阅需求执行：只看Mesh且无人订阅esdf_points时，ESDF导出点数/频率可能保持旧值或不更新；本轮采样器始终订阅该话题。

```bash
ros2 topic hz /camera/depth/image_rect_raw --wall-time
# Ctrl+C后
ros2 topic hz /camera/color/image_raw --wall-time
# Ctrl+C后
ros2 topic hz /nvblox_node/mesh --wall-time
```

单独topic hz不能得到融合率或新输入关联率；完整口径请使用下一节自动测试。监测多个大话题会增加订阅负载。

### 静止20秒后：终端2执行小幅左右扫描

```bash
/usr/bin/python3 artifacts/mesh_esdf_validation_20261109/view_sweep_isolated.py --seconds 20 --amplitude 0.08
```

指令沿已检查的world Y轨迹，高度约0.82m，不环绕，不做前后往复，结束后回中。保留足够时间等待动作结束。

### 查看Mesh、3D ESDF与保存结果

Mesh RViz中Fixed Frame为base_link，勾选Official incremental RGB Mesh。Mesh是官方插件显示，RGB图像可用于对比原始输入。鼠标在3D区域左键旋转、中键平移、滚轮缩放，Views选择Orbit。

ESDF可在同一RViz添加PointCloud2，话题/nvblox_node/esdf_points，Style=Points，Size(Pixels)=3，Color Transformer=Intensity，Channel=intensity，范围−0.09～0.24m，启用rainbow。也可单独运行`./rviz.sh diagnostic`，但额外窗口会增加负载；不要把ESDF彩虹当成Mesh的真实RGB颜色。

保存目录与服务命令（在已加载环境的终端4执行）：

```bash
mkdir -p /home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/manual_mesh_test_20261109
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/manual_mesh_test_20261109/mesh.ply'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/manual_mesh_test_20261109/timings.txt'}"
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/manual_mesh_test_20261109/rates.txt'}"
```

save_ply会触发完整网格保存，耗时应排除在正式实时性采样窗口之外。结束时先停止运动命令、Mapper/RViz，再Ctrl+C停止场景；本次自动测试结束已退出自己启动的进程。

## 7. 一条入口复现自动测试并保留旧证据

先停止手动启动的本域场景和Mapper。以下会运行两档、四阶段并自动退出，创建新的retest目录，不覆盖本次结果：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
DISPLAY=:1 ./artifacts/mesh_esdf_validation_20261109/run_retest.sh
```

run_retest.sh准备新证据目录后调用本轮已实际执行过的run_measurement.py，再执行离线统计及统一视角PLY绘图；封装脚本完成bash语法检查，未再重复启动一整轮实验。依赖DISPLAY、同一GPU/ROS安装、空闲端口和当前工程场景。若失败，查看新目录measurement.log，不要把不完整采样当作成功结果。

## 8. 代码定位、证据和本轮边界

- [启动后端与参数覆盖](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/nvblox.launch.py:58)
- [项目派生节点](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:36)
- [原生ESDF导出](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu:1)
- [官方深度/RGB主处理](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:966)
- [TSDF加权融合](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_tsdf_integrator.cu:29)
- [颜色融合与合成深度](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_color_integrator.cu:49)
- [Mesh近邻颜色](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mesh/mesh_integrator_color.cu:52)
- [Mesh按订阅计算发布](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:666)

- [实际编译日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/build.log)
- [命令、参数验证、发布器、原生点格式与执行状态](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/results.json)
- [本轮统计与条件ROI检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/analysis.json)
- [可直接导入表格的频率结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/test_results.csv)
- [仿真与机器人状态检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/robot_state_audit.json)
- [每条ESDF的header、到达时间、点数与年龄](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/cloud_records.json)
- [输入、Mesh、累计计数和时钟记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/samples.json)
- [实测相机轨迹](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/mesh_esdf_validation_20261109/camera_trajectory.json)

所有PLY、原始RGB截图、深度npy、运行时参数dump、ROS日志和窗口截图均保存在同一artifact目录。source_manifest.json记录本轮项目源码、相关官方源文件及实际Mapper二进制的SHA256。

本轮证明编译/启动链可用，并给出静止/左右观察下的真实频率和局部几何证据；没有证明整场景完整重建，没有完成噪声/时间标记/机器人遮罩的因果隔离，也没有给出实机D435精度或长期大地图性能保证。
