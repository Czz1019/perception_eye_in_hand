# 重建效果、体素大小与表面平整度差异分析及优化方向

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

日期：2026-10-27。对象：perception_eye_in_hand 原生 RGB/ESDF 接口 v3，与本机 workspaces 中官方 ROS 移植实现。

本轮仅分析源码、配置和显示逻辑，生成本文及证据索引，没有修改感知/控制程序，没有启动仿真、发送运动或测量当前场景。用户描述的“体素大、不平整”没有对应本轮采集的图像、运行参数 dump 和地图，因此下面明确区分确定的配置差异与需要实验确认的原因。

## 1. 结论与排查顺序

当前效果与 workspaces 不同是有具体原因可查的，不能归结为“已经使用官方库，因此效果必然相同”。目前的差异不仅在导出速度，也在体素分辨率、显示对象、融合权重、ESDF 置信门槛、相机输入和采集运动。

优先检查顺序：

1. **显示对象是否相同。** Mesh、颜色体素、ESDF inside 体素、距离带不能当作同一种表面比较。
2. **实际 voxel_size 是否相同。** 当前项目源码默认 3 cm，workspaces 官方 ROS 基础配置默认 2 cm；先前人为统一到 3 cm 的性能对照，不代表 workspaces 默认就是 3 cm。
3. **TSDF 加权参数是否相同。** 当前没有采用参考基础配置的 TSDF 距离惩罚加权。
4. **ESDF 权重门槛是否过低。** 当前 0.0001，参考 0.1，相差 1000 倍；这主要影响 ESDF 的可信观测范围，并不直接改变 Mesh 的权重门槛。
5. **静止时是否平整、运动后是否变厚。** 这能分离原始深度/融合问题与图像位姿、机器人自体、运动采样问题。
6. **最后再评估体素缩小、过滤和更复杂的重建策略。** 先保持 3 cm 和现有输入做算法对照，避免通过改精度掩盖逻辑问题。

## 2. 比较的是哪一份 workspaces 实现

主要参考是 `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox` 中的官方 ROS 移植库，当前本地 Git HEAD 为 `bc181942202b24fd47b2f3db8f77c3c82d484566`。本地文件存在定制修改，不能把它的所有参数称为 NVIDIA 上游原始默认值。

核对路径包括：

- `nvblox_examples_bringup/config/nvblox/nvblox_base.yaml`：基础参数，当前安装目录的对应文件解析到该源码。
- `launch/perception/nvblox.launch.py`：按基础配置、模式配置、相机配置和启动参数顺序叠加。
- `config/nvblox/specializations/nvblox_realsense.yaml`：RealSense/仿真桥接使用的相机配置。
- `launch/gazebo_realsense_example.launch.py`、`launch/sensors/gazebo_realsense.launch.py`：固定 Gazebo 相机和原图 relay 路径。
- `nvblox_ros` 和内部 `nvblox_core`：实际 TSDF、ESDF、Mesh 与显示实现。

另有 `gazebo_d435_nvblox.launch.py` 简化入口和 `nvblox_realsenseD435/sim_nvblox` Python 单次快照实现；它们不是同一个运行配置。简化入口中的旧 remap 写法也与当前库订阅名不同，不能仅看它的数值就认定用户过去实际运行了该节点。本文参数表以当前官方 ROS 基础配置为主，固定场景输入分析另有明确来源。

这些是**源码条件差异**，不是当前正在运行的节点参数实测。自定义 mapper_config、命令行覆盖、另一套已安装包都可能改变实际值，需按第 8 节确认。

## 3. 已确定的差异

### 3.1 参数对照

| 项目 | perception 当前配置 | workspaces ROS 基础配置 | 可能影响 |
| --- | --- | --- | --- |
| voxel_size | 0.03 m | 0.02 m | 当前空间格点更粗 |
| TSDF 加权 | inverse_square | inverse_square_tsdf_distance_penalty | 多视角融合的权重分配不同 |
| 截断带 | 4 vox = 0.12 m | 4 vox = 0.08 m | 物理融合带不同，不能只比较参数中的数字 4 |
| TSDF 最大权重 | 5 | 5 | 此项一致 |
| TSDF 最大积分距离 | 7 m，但公共深度有效范围为 0.2–4 m | 基础配置 5 m，部分示例单独设置 3 m | 地图可见范围和背景处理不同，不能只看积分距离参数 |
| ESDF 最低 TSDF 权重 | 0.0001 | 0.1 | 当前允许更多低权重体素参与 ESDF |
| ESDF site 距离阈值 | 1 voxel = 0.03 m | 2 voxels = 0.04 m | 距离种子集合和近表面零距离范围不同 |
| Mesh 最低权重 | 0.1 | 0.1 | 此项一致；不能用 ESDF 的门槛直接解释 Mesh 毛刺 |
| Mesh weld_vertices | true | true | 此项一致；焊接不等于几何平滑 |
| depth preprocessing | false | false | 都没有启用该内置预处理 |
| raycast_subsampling_factor | 4 | 4 | 此项一致，用于视锥块选择，不等于融合图像缩成 1/4 |
| 深度积分目标 | 1000 Hz | 30 Hz | 当前意图不额外限制相机；实际不可能超过有效输入 |
| ESDF 更新目标 | 30 Hz | 5 Hz | 当前传播计算负担可能更高；不直接提高 Mesh 精度 |
| 颜色积分目标 | 5 Hz，已启用 | 5 Hz，但基础 use_color=false | 显示内容与负载不同 |
| 图层发布 | 5 Hz | 5 Hz | 目标一致，实际频率需测量 |
| 输入队列上限 | 2 | 10 | 低延迟与等待 TF/丢帧的取舍不同 |
| TSDF 衰减 | 关闭 | 基础配置 2 Hz | 历史弱观测保留策略不同；参考特定 launch 也可能关闭 |
| 地图清理半径 | -1，关闭 | 基础配置 7 m | 当前导出 ROI 不限制后台整个地图分配 |

证据：[当前参数](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo.yaml:6)、[参考基础参数](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml:6)。完整键值比较与文件 SHA-256 见 [源码比较记录](/home/czz/perception_eye_in_hand/artifacts/esdf_surface_analysis_20261027/source_comparison.json)。

### 3.2 “体素看起来大”的直接解释

3 cm 与 2 cm 相比，格点边长增加 50%。一个体素代表的空间体积为原来的 3.375 倍。这解释格点更粗，但不是实测运算量比值；表面、分配块和截断区域的复杂度不同。

官方 `color_layer_marker` 使用 `CUBE_LIST`，实际显示边长是 `voxel_size - 0.001 m`。因此当前显示约 **2.9 cm** 的小立方体，参考 2 cm 配置显示约 **1.9 cm** 的立方体。立方体沿斜面排布，天然会形成台阶，不能期待它像连续 Mesh 一样平整。证据：[官方 Marker 尺寸逻辑](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:310)。

同时要区分：

| 显示 | 真正画出的内容 | 能否据此评价表面平整度 |
| --- | --- | --- |
| esdf_occupied | observed && is_inside 的体素中心 | 不能替代连续表面评价；它是格点集合 |
| esdf_distance_band | 表面附近正负距离带，默认 ±15 cm | 会包含已知自由空间，整体有厚度不等于墙体变厚 |
| color_layer_marker | TSDF distance≤0 且满足权重筛选的彩色体素 | 不限于 TSDF 零面，立方体和内部带会显得厚、粗 |
| mesh_markers / 原生 mesh | TSDF 零交叉提取的三角形表面 | 适合检查重建连续性、几何平整度与薄结构 |

参考 `realsense_example.rviz` 默认启用的是 NVIDIA Mesh，颜色体素默认关闭；当前项目的默认配置也已改为标准 Mesh，但用户保存的 RViz 状态或额外勾选项可能覆盖默认效果。应先比较同样的 Mesh，再比较同样的体素。

PointCloud2 当前默认是 3 像素 Points，其屏幕大小不代表地图体素边长；用户把 Style 改成 Boxes 后，Size (m) 也会影响观感。Marker 立方体、PointCloud2 点大小和真实 voxel_size 是三个不同概念。

### 3.3 输入和视场也没有完全对齐

当前场景：原始深度 1280×720，通过 stride=2 的纯抽点变为 640×360；原时间戳与 optical frame 保留，内参按比例缩放。

workspaces 的固定 Gazebo D435 示例 SDF：原始深度 **640×480**；官方仿真桥接使用 topic_tools relay，所查路径没有在 relay 中增加相同的 stride=2 抽点。

两套传感器都配置水平 FOV=1.57 rad。按当前插件针孔内参计算，两者送入融合的水平焦距都约 320.25 px，但当前垂直视场约 **58.68°**，固定参考 SDF 约 **73.70°**。因此不能只说“宽都是 640，所以视场相同”，也不能简单说当前水平分辨率一定更差。它们的垂直覆盖和实际目标在图像中的位置不同。

纯抽点不做边缘保持去噪，会保留被选像素的噪声，丢掉未采样的细线/薄边。代码中的 `raw[::stride, ::stride]` 与内参除以 stride 对应，未发现这里必然存在缩放内参错误；更换为像素平均/插值后才需要重新核对采样位置和遮挡边界。证据：[深度解码](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/depth.py:21)。

## 4. 表面“不平整”的原因分析

### 4.1 TSDF 加权没有完整对齐：高优先级

官方当前参考实现中：

- inverse_square 的基础权重主要由体素在相机坐标系中的深度 z 决定，约为 1/z²。
- inverse_square_tsdf_distance_penalty 在此基础上，对绝对投影 TSDF 距离不小于截断带的测量再乘 0.1。

这不是普遍意义的空间平滑，也不是把所有远处点一律乘 0.1；它改变的是融合时截断带外测量的影响。源码说明该策略用于减弱视角变化敏感的远表面测量，对地面孔洞可能有帮助。当前配置未保留这一策略，是可确认的实现条件差异，值得先在同精度条件下做单变量对照，但不能仅凭代码承诺一定消除当前毛刺。证据：[实际加权函数](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/integrators/internal/impl/weighting_function_impl.h:98)。

TSDF 融合是加权更新并截断，Mesh 从其零交叉生成。更宽截断带不等于把实体强制膨胀 12 cm，但在多视角、遮挡边界和噪声条件下，会改变进入融合的区域和结果。应联合核对物理带宽与加权，不能把“不平整”简单归结为 ESDF 查询。

### 4.2 ESDF 置信门槛偏低：影响 ESDF 可信度，不直接修复 Mesh

当前 ESDF 的 min_weight=0.0001，比参考 0.1 低 1000 倍。它会更早接受权重较低的 TSDF 体素，可能让边缘、初始单帧或弱观测区域更快进入 ESDF，表现为额外散点、边界波动或不稳定距离。

但两边 Mesh min_weight 都为 0.1。若用户当前看到的是 RGB Mesh，修改 ESDF min_weight 并不会直接改变 Mesh 的 TSDF 零面。必须先区分是哪一层毛糙。

0.1 也不是“至少积累若干帧”的可靠替代：当前 inverse_square 在近距离的一帧权重就可能超过它。若需要多帧一致性，还需独立统计观测次数或稳定性，不能只把 weight 当作帧数。

当前 max_site_distance_vox=1 与参考=2 改变 ESDF 种子选择。该实现将满足条件的 inside 近表面体素作为 site；增大阈值可能扩展零距离种子层，不是通用的高精度修复。应按米核对、保留安全距离语义，再做距离误差验证。证据：[TSDF→ESDF 的 observed/inside/site 判断](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/esdf_integrator.cu:113)。

### 4.3 相机运动后的时序和几何一致性：需要实测

参考固定场景的相机位姿不变，基本没有图像时刻位姿偏差累积的问题；当前机械臂在移动。即使使用同一 TSDF 算法，运动输入也更容易暴露时间与位姿误差。

当前官方处理确实按深度图像时间戳查询 TF，查不到则等待/重试，并非拿“最新 TF”直接套在历史深度上。此前“最新 TF 查询导致表面错位”的假设，不能直接作为当前代码已证实的问题。

仍需检查两件事：

1. Gazebo 图像插件在回调中读取 `world->SimTime()` 作为图像 stamp；该时刻与渲染深度真正对应的仿真采样时刻是否一致，需要实验核对，不能仅因为 TF lookup 使用了 stamp 就认定无误差。
2. 图像 stamp、joint_states、TF 是否在运动时充分对齐；实际相机传感器模型的位姿是否与相应 optical frame 一致。仅证明相机 pose 话题等于同一 TF，不是对传感器真实位姿的独立验证。

一阶近似可用 `位置误差 ≈ 平移速度×时间偏差 + 目标距离×角速度×时间偏差` 评估量级。例如假设目标距离 1 m、相机角速度 0.2 rad/s、时间偏差 50 ms，仅旋转项约为 1 cm。这是量级示例，不是本机实测偏差。

若静止融合平整、开始转动后出现双层墙或整体变厚，应优先检查时序/位姿，而不是继续降低权重门槛或增加显示点数。证据：[插件深度 stamp](/home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp:234)。

### 4.4 采集轨迹覆盖不足：已确定动作范围有限

新增视角动作仅围绕已有观察姿态摆动 wrist_1，默认 ±0.06 rad，约 ±3.44°；它不自动围绕目标进行球面/环绕采集，也没有保证目标持续居中。

这有助于测试实时更新，但不等于完整重建轨迹。侧面、背面、底部、掠射角表面可能长期没有足够观测；单侧观察墙面或遮挡边缘时可能出现孔洞、缺面和不均匀的局部质量。证据：[摆腕动作](/home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/scripts/view_sweep.py:134)。

### 4.5 机器人自体进入图像和静态地图残影：较高价值的检查项

当前应用没有在配置中接入机器人自体深度 mask，使用 static_tsdf，关闭 TSDF 衰减和地图清理。机械臂/夹具若进入相机视场，会被当作普通几何融合；移动后可能留下弱残影、浮动表面或局部厚层。静态场景下无机器人遮挡的参考结果更容易干净。

不能据此认定每个突出物都是机械臂残影：应在 RGB、原始深度和机器人模型投影中定位同一块区域。视角在腕部小范围转动，也不意味着机械臂表面相对 base_link 始终静止。

优化应优先进行有标定依据的机器人模型投影与深度一致性过滤，保留前景障碍；不应仅按某个大球/大盒删除邻近真实物体。衰减可以清弱残影，但也可能清掉后来被遮挡的真实障碍，需要与规划安全语义联合验收。

### 4.6 深度噪声、边缘和低权重观测：需以输入实测区分

当前 URDF 的深度 camera 节点含 gaussian/stddev=0.02，所查固定参考 SDF 中也有相同数值。因此不能把“当前额外配置了 2 cm 深度噪声”作为两者差异结论。

还需要核对 Gazebo 相应噪声模型究竟作用于哪些渲染输出，并通过静态平面原始深度的时间方差验证实际深度噪声。没有这一步，不应把该 stddev 直接当作测得的深度标准差。边界的飞点、无效深度和掠射角误差同样需要从原图确认。

当前关闭内置 depth preprocessing，参考基础配置也关闭；该实现的预处理主要是无效区膨胀，不是已经提供完整去噪。盲目打开可能扩大空洞，不能直接当作平面平滑开关。

两边 max_weight=5 一致，累计权重达到上限后仍会接受新测量，不是“融合几帧后地图完全冻结”。提高最大权重可以增强静态累计稳定性，也会降低错误几何纠正和动态变化适应速度，应单独实验。

### 4.7 RGB 纹理与几何的错觉：分开检查

颜色融合为 5 Hz，而深度可能更快。颜色在边界处不连续、部分三角形暂时灰色或 RGB 投影偏移，都可能让人感觉表面凹凸，但不必然说明顶点位置错误。

应把同一 Mesh 导出后用统一颜色显示，与真实 RGB 版本比较：若只有 RGB 图像看起来不平整，优先检查颜色覆盖、独立颜色内参、颜色图像时间/外参。增加颜色积分频率不能直接提高 TSDF 的几何分辨率。

## 5. 显示桥接是否引入了几何粗化

本次核对没有发现 `mesh_visualization.h` 对顶点进行体素中心取整、重新采样或额外添加 block 原点。它使用官方 Mesh 的全局顶点坐标，按 triangle indices 展开为 TRIANGLE_LIST，scale=1。这条路径本身不会把 2 cm 的 Mesh 自动量化成另一个更粗网格。证据：[桥接实现](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/mesh_visualization.h:29)。

当前稀疏 ESDF 导出器也没有回写 TSDF 或修改 Mesh 顶点。若真正的表面 Mesh 不平整，不能直接归因于导出器“查询点太少”；查询/导出开销可能通过争用处理时间间接减少有效输入，这与改变几何的算法错误需要分开验证。

标准 MarkerArray 会展开共享顶点，消息更大，可能增加 CPU、DDS 和 RViz 绘制开销；这主要影响更新与显示负担，不直接改变三角形几何。Mesh 以块为单位增量更新，单次消息并非完整地图。新订阅者的快照、清除和后续增量若有漏块/覆盖顺序问题，可能表现为闪烁或缺块，应做块缓存一致性检查，不能仅凭屏幕截图判断 TSDF 损坏。

原生 Mesh 插件在本参考版本中使用 `BaseWhiteNoLighting`，没有在该渲染路径逐顶点设置法线；官方 ROS Mesh 转换也未填充 normals 字段。因此目前没有依据说“workspaces 通过平滑法线，而我们丢失法线”是主要原因。证据：[官方 Mesh 渲染](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_rviz_plugin/src/nvblox_plugin_visual.cpp:252)、[Mesh 转换](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/conversions/mesh_conversions.cpp:62)。

`mesh_integrator_weld_vertices=true` 主要合并重复顶点，不是平面拟合或曲面平滑；不能通过再次打开同一开关期待墙面变平。三维 ESDF 格点量化也不能通过提高导出频率消除。

## 6. 优化方向与优先级

| 优先级 | 建议方向 | 解决什么问题 | 预期与代价 |
| --- | --- | --- | --- |
| P0 | 锁定运行参数、相机输入、视图、轨迹和参考版本 | 避免比较不同体素/不同对象/不同视场 | 建立可解释基线，不直接改变地图 |
| P0 | 先以 3 cm、现有深度输入、同一 bag 对照原生官方节点与当前应用 | 区分官方地图参数问题与自定义导出/调度问题 | 保持精度公平；需串行回放或避免双进程 GPU 争用 |
| P1 | 同 3 cm 下，单独比较 TSDF distance penalty 加权 | 多视角融合权重差异、可能的地面孔洞 | 预期改善部分视角敏感区域；不能承诺所有表面更好 |
| P1 | ESDF min_weight 从 0.0001 与参考 0.1 做对照 | 低可信体素过早进入 ESDF | 可能减少弱散点；覆盖变少/建立更慢需监测 |
| P1 | 静止→慢转→正常转的时序诊断，记录图像/TF 与相机真实位姿 | 双层表面、运动后变厚 | 若找到错误应修正时间来源/外参；降低速度只是诊断，不是修复 |
| P1 | 机器人自体 mask、深度一致性过滤 | 机械臂/夹具造成的静态地图残影 | 增加投影与过滤开销；必须避免误删近处障碍 |
| P1 | 目标中心约束的多观察位姿，停稳采集与连续运动分开比较 | 单腕摆动覆盖不足、掠射角质量差 | 增加采集时间；轨迹需姿态、碰撞与可达性规划 |
| P2 | 边缘保持深度滤波/稳健降采样，对无效区和薄边单独处理 | 深度噪声、飞点、纯抽点混叠 | 不能跨越前景背景平均；不能把孔洞补成自由空间 |
| P2 | ESDF 更新与 Mesh 显示预算调整，减少同步等待/重复序列化 | 不必要计算挤占深度融合和显示 | 保持真实地图精度，可能降低部分发布节奏；需要端到端测量 |
| P2 | 明确融合边界、有限局部地图/块清理策略 | 当前 ROI 只限制导出，地图长期增长 | 降低块数与内存；必须保留规划所需空间并处理删除 |
| P3 | 精度独立实验：2 cm 或更细地图 | 明确 3 cm 本身的离散化上限 | 同体积稠密格点 3→2 cm 理论增加 3.375 倍，实际负载另测；不能冒充同精度提速 |
| P3 | 仅供显示的法线、着色或保边平滑 | 视觉观感、三角形片面感 | 不应回写碰撞 ESDF；必须同时保留真实几何视图 |

### 6.1 第一阶段建议的参数实验

保持 voxel_size=0.03、深度 640×360、输入序列、ROI、最大距离和显示消费者不变，先分开做：

1. 原配置基线。
2. 仅将 weighting_mode 对齐到 inverse_square_tsdf_distance_penalty。
3. 回到基线，仅将 ESDF min_weight 对齐到 0.1。
4. 再观察组合是否互相影响；Mesh 和 ESDF 分别验收。

这不是本轮已经执行的参数修改。保持同体素时 truncation=4 vox 对应 12 cm，不能误称已经与参考 2 cm 的 8 cm 物理带完全相同。若要验证物理带宽，再作为独立实验显式定义 8 cm，不与精度改变混为一项。

max_site_distance_vox 不作为首轮“画面平滑”参数。它影响距离场几何语义，必须用真实障碍距离检查后再决定。

### 6.2 固定精度下改善实时性和质量的正确关系

当前 depth/ESDF 阶段有 CUDA 同步等待，块导出按选择块分批读取，标准 Mesh 桥接展开三角形；首次重发还会全层更新 Mesh。这些操作可能争用处理时间。当前 ESDF 目标 30 Hz 也比参考基础的 5 Hz 高。

若资源竞争导致有效融合帧减少、相机快速运动时观察覆盖不足，就会间接影响重建完整度。先测 depth_integrated、处理耗时和队列丢帧，再决定是否减轻 ESDF/显示任务。不能通过重复发布旧地图提升 topic hz，并宣称质量和实时性都得到改善。

如果需要不同分辨率的可视化地图与规划 ESDF，当前单个 Mapper 的 voxel_size 是共享层分辨率，不能只加两个独立参数就宣称支持。独立 Mapper/重采样/双地图方案需要另外评估一致性、内存和计算预算，不是本轮的简单配置优化。

## 7. 如何判断优化是否真的有效

建议以固定墙面、箱体边界与薄结构作为三个目标，测量：

| 指标 | 测量方式 | 防止什么误判 |
| --- | --- | --- |
| 平面残差 P50/P95、RMSE | 对同一物理墙面 ROI 的 Mesh 做面积均匀采样后拟合平面 | 点多不等于更平整，顶点密度不均会偏置统计 |
| 绝对位置偏差、法向误差 | 对比 Gazebo 真值平面/箱体与地图，统一 frame | 一个错误偏移的平面也可能残差很小 |
| 双层厚度、运动前后差异 | 同一墙面法向上的分布、静止与运动的差分 | 平均图掩盖重影 |
| 覆盖率、孔洞、离群比例 | 在相同实际可见区域统计，保留未观测标签 | 简单删掉坏点可能让 RMSE 变好却丢失障碍 |
| ESDF 距离误差与未知率 | 在已知自由区域查询真值距离，单独统计未知 | 不能把未知当自由，不能只看 ESDF 颜色 |
| 图像→地图延迟 P50/P95/P99 | 明确计时起点，使用不同源时间戳配对 | topic hz 不能证明低延迟 |
| 有效融合/ESDF/层显示频率 | 统计真实完成或提交的对应阶段，区分 ROS 与墙钟 | 源仿真变慢时不能只比较配置频率 |
| 分配块数、Mesh 三角形数、消息字节和 CPU/GPU 负载 | 同样消费者条件下采集 | 更细或显示更多带来的开销需显式记录 |

原始深度→TSDF Mesh→ESDF 是数据依赖链。先定位最早出现误差的一层：原始深度就粗糙则查输入；深度和位姿正常但 Mesh 变形则查融合；Mesh 正常但 ESDF 异常则查 site/权重/导出；导出地图正常但 RViz 异常则查缓存/显示。

建议实验顺序：A 静止单帧、B 静止多帧、C 慢速小幅转动、D 原有摆腕、E 有覆盖规划的多视角。每轮使用新地图或一致的初始化，保持同一数据回放与显示视角。单帧无法完整重建目标，A 的目的只是定位输入和单帧融合的几何问题。

## 8. 后续只读核对命令

本节供用户在当前应用运行时执行。本轮没有执行。新终端先设置与当前场景一致的 ROS_DOMAIN_ID、RMW 和 Cyclone DDS 配置，通常为：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
source /opt/ros/humble/setup.bash
ros2 node list
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox static_mapper.projective_integrator_weighting_mode
ros2 param get /ur7e_nvblox static_mapper.projective_integrator_truncation_distance_vox
ros2 param get /ur7e_nvblox static_mapper.esdf_integrator_min_weight
ros2 param get /ur7e_nvblox static_mapper.esdf_integrator_max_site_distance_vox
ros2 param get /ur7e_nvblox static_mapper.mesh_integrator_min_weight
ros2 topic echo /ur7e_nvblox/stats --once
ros2 topic echo /ur7e_nvblox/topic_info --once --qos-durability transient_local
```

完整快照可用 `ros2 param dump /ur7e_nvblox > /tmp/perception_quality_params.yaml`。对 workspaces 参考实例，在它实际运行的 ROS 域中查询 `ros2 param dump /nvblox_node`，以 `node list` 中实际名字为准。不要为了查看参数另启动第二个 mapper，与当前实例争用同一输入和 GPU 后再比较性能。

默认 RViz 中先只启用 Surface Mesh，关闭 ESDF 和 Color Surface Voxels；随后分别比较各视图。记录同一观察角度、同一缩放、同一可见目标的图像。若怀疑只是颜色问题，可保存几何用于独立查看：

```bash
# 自定义服务类型需要官方安装前缀；仅 source，不会启动额外节点。
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ros2 service type /ur7e_nvblox/save_ply
ros2 service call /ur7e_nvblox/save_ply nvblox_msgs/srv/FilePath '{file_path: /tmp/ur7e_quality_mesh.ply}'
```

这项保存会请求地图线程更新/导出 Mesh，可能产生短时停顿；不要把保存时段混入正常延迟基线。不要在正在运动的不同时间点保存两份文件后当作同一几何对照。运行和视角操作仍参见 [当前完整操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_COLOR_AND_TOPICS_GUIDE.md)。

## 9. 本轮可以确认与不能确认的事项

**可以确认：** 3 cm/2 cm 源码配置不同；TSDF 加权、ESDF 权重/site 阈值不同；体素立方体显示约 2.9 cm/1.9 cm；固定参考 SDF 与当前场景的输入尺寸/垂直视场不同；当前摆腕不是完整环绕采集；当前未接入自体深度过滤；标准 Mesh 桥接没有主动把顶点量化成体素中心。

**不能在本轮确认：** 用户当前实际加载哪份参数；正在看的显示层；当前平面误差大小；深度真实噪声标准差；渲染 stamp 与图像采样是否存在偏差；当前是否出现机器人残影、漏块或颜色错位；哪项优化带来多少 Hz 或多少毫米提升。

本轮结论是：先对齐显示与测量条件，再优先验证 TSDF 加权、ESDF 置信门槛、运动时序和自体过滤。体素细化是独立精度选择，不能替代逻辑修正，也不应把单纯观感变平滑作为规划地图更准确的证据。

## 10. 交付与证据

- 本文：原因分层、参数比较、代码判断、优化优先级、验收指标和后续核对命令。
- [源码参数与证据索引](/home/czz/perception_eye_in_hand/artifacts/esdf_surface_analysis_20261027/source_comparison.json)：文件路径、关键行、SHA-256、静态参数差异及推导数值。
- 本轮仅新增分析文档/证据及文档索引链接；没有改变体素、加权、滤波、运动、显示配置或可执行程序。
