# 三套 nvBlox 感知实现的融合策略、质量问题与对比分析

日期：2026-11-09。范围：只读分析代码、既有实机统计和已导出 Mesh；不修改源代码、不调整运行参数、不执行新的运动或对照测试。

## 1. 结论与证据边界

`ur7e_nvblox_in_hand` 与 `nvblox_in_hand` 不是相同的 ROS 接口，也不保证使用相同版本的 nvBlox 核心；但二者都把深度观测及相机位姿送入 nvBlox C++/CUDA 的投影 TSDF 融合。因此，输入深度噪声、安装标定、图像与机械臂时间偏差、体素尺度和静态建图假设等共性问题，会导致相似的模糊、飞边和重影。换成 C++ ROS 接口不会自动修正这些误差。

`realtime_nvblox` 同样使用 nvBlox C++/CUDA，并非另一种天然更精确的重建算法。其优势可能来自直接 SDK 接入、配套内外参、不同的位姿来源、较轻的输出负载或不同的显示采样。但本次没有旧版同场景、同输入的原始记录，不能证明旧版在相同条件下几何精度更高。

重要限制：此前提及的 `/home/czz/realtime_nvblox_ws.zip` 当前路径不存在；本次对照的是仓库 `src/realtime_nvblox`，补充核对 `src/realtime_nvblox_2`。这些代码的当前默认值不能直接当作用户过去运行时的实际配置。两个目录包含同名 Python/ROS 包，实际运行时还应记录包路径，避免拿错版本。

## 2. 三套实现的结构与差异

| 项目 | realtime_nvblox | ur7e_nvblox_in_hand | nvblox_in_hand |
|---|---|---|---|
| 入口与组织 | 自有采集、位姿缓存、建图和查询工作线程 | ROS rclpy 接入、单 GPU 所有者工作线程、独立输出 | ROS C++ 适配节点继承官方 NvbloxNode，使用其队列、融合及发布流程 |
| 图像来源 | pyrealsense2 直接采集 | ROS Image + CameraInfo | ROS/NITROS 图像 + CameraInfo |
| 核心 | 安装环境中的 nvblox_torch wheel C++/CUDA | nvblox_torch wheel C++/CUDA，检查固定提交，与导出桥头文件匹配 | workspaces 已安装的 nvblox_ros 及其 nvBlox C++/CUDA 核心 |
| 位姿 | 默认 cuVSLAM；机械臂配置可改为 TF/位姿话题 | 每路图像采集时间查询全局到该 optical frame 的 TF | 官方 Transformer 按传感器 frame 和图像时间查询 TF |
| 内外参 | 直接读取实际 SDK stream profile 与 get_extrinsics_to | 从 CameraInfo.K 构造相机，外参来自 TF | 本地官方转换函数使用 CameraInfo.K，外参来自 TF |
| 深度单位 | 读取设备 depth_scale_m | 16UC1 按毫米乘 0.001；32FC1 按米 | 官方 ROS 图像转换约定；需与消息编码/单位一致 |
| 参数方式 | 构造 Mapper 时主要指定体素和 TSDF 类型，其余多为 wheel 默认 | 显式设置 MapperParams 中的权重、截断、ESDF、Mesh 等参数 | base→相机→机械臂→profile→实机→CLI 多层配置 |
| ESDF 显示 | 在自定义规则网格上查询距离 | 导出原生 observed ESDF 体素 | GPU 压缩导出原生 observed ESDF 体素 |
| Mesh 输出 | Mesh 数组/保存，RLock 串行保护 | 完整 MarkerArray 替换 | 官方增量分块 Mesh 消息与插件 |

核心版本口径：工程已有依赖审计记录 Python wheel 为 `0.0.10.dev1+cu12`、提交 `e4a2b00f45ecf989682d72a33a2c29defe39fd2d`，本地 ROS 路线记录为 ROS 3.2.5 移植、core 0.0.7。不能据此推导两者二进制行为逐项相同；本次没有重新完成所有库的版本审计。

## 3. 为什么 realtime 可能看起来更好

### 3.1 输入、内外参和位姿路径不同

`realsense_source.py` 从同一设备的 stream profile 读取内参、深度比例和传感器间外参，外参旋转矩阵按 librealsense 列主序转换。`RobotPoseProvider.configure_calibration()` 将手眼标定换算到 rig，再在每路图像时间组合 depth/color 位姿。

新两套使用外部 ROS 驱动及 TF。理论上可以完全一致，但必须确保 TF 外参来自当前设备与当前挂载，CameraInfo 对应实际分辨率，时间戳属于同一时间基准。它们不会自动验证历史安装参数是否依旧准确。

发现一项具体不一致：`src/realtime_nvblox/config/robot_eye_in_hand.yaml` 中 tool0→color 平移为 `[-0.017628,-0.059898,-0.011736]`，而当前历史标定文件记录 `[-0.029901376,-0.069185919,-0.012263443]`，旋转也不同。两组平移相差约 1.54 cm。不能直接互换，也不能断言其中哪组正确；必须根据真实安装与标定记录核对。若当时 realtime 实际使用 cuVSLAM，其位姿误差来源又与机械臂方案不同。

### 3.2 默认运行任务和显示密度不同

检查到 realtime 默认配置：原生体素 0.05 m、颜色关闭、Mesh 关闭、Mesh 输出上限 1 Hz、自定义 ESDF 查询间距 0.15 m。机械臂覆盖配置仍关闭颜色和 Mesh，并将 depth_hz=0 表示连续处理。

当前新两套测试启用颜色与 Mesh，体素 0.03 m，Mesh/层调度 5 Hz，输出原生 observed ESDF。旧版更稀疏的点显示可能更干净，较轻负载也可能改善响应；但它不能作为“旧版 Mesh 更精细”的证明。必须找到旧版当时启用 Mesh/颜色的真实配置与导出文件。

规则网格查询是合法的距离场查询方式，但查询间距不等于地图体素尺寸，完整格点也不等于完整观测。当前默认 publish_full_grid=true；unknown 应结合发布代码和标志解释，不能按外观当作真实自由空间。

### 3.3 没有发现旧版存在已启用的特殊深度滤波链

在此次检查的 realtime 两份 Python 源码中，没有发现 spatial_filter、temporal_filter、hole_filling_filter 或 decimation_filter 的显式调用。不能将其优势归因于“旧版自动使用了 SDK 时空滤波”。默认 VSLAM 分支确实有投射器亮灭分流，机械臂配置则关闭闪烁；需要区分运行分支。

## 4. 官方实际使用的融合优化策略

以下结合官方概念文档和当前本地官方 core 源码；网页 latest 与本地旧 core 的默认值可能不同，具体行为以本地源码及实际参数为准。

### 4.1 投影 TSDF 与带权多帧融合

本地 `projective_tsdf_integrator.cu` 将体素投影到深度图，使用测量深度与体素相机深度的差作为投影距离。在允许的截断范围内按如下方式融合：

```text
D_new = clamp((W_old * D_old + w * d) / (W_old + w), -mu, mu)
W_new = min(W_old + w, W_max)
```

这是一种局部带权累积，不是自动进行跨帧 ICP、手眼再标定或位姿图优化。若位姿系统性错误，平均会把错误写入表面；增加融合次数无法消除系统误差。权重达到上限后仍继续融合，并非地图冻结。

### 4.2 距离权重及截断外惩罚

本地 `weighting_function_impl.h` 支持 constant、constant_dropoff、inverse_square、inverse_square_dropoff、inverse_square_tsdf_distance_penalty 等模式。

当前新两套显式配置的模式为 inverse_square_tsdf_distance_penalty。该本地实现的基础权重按体素的相机深度计算 `1/z_voxel²`（接近零时特殊处理），不是简单写成所有情况下都按测量深度倒数。超出表面后方截断距离不贡献；TSDF 距离达到截断尺度的观测会附加较低系数，减少视角变化影响。

realtime 构造 Mapper 没有显式传入上述整组参数，不能凭代码文本认定它一定采用 constant，也不能认定它与新两套权重完全一致。需要读取其实际 wheel 默认参数才能公平比较。

### 4.3 无效深度处理与几何筛选

官方支持无效区域膨胀预处理；它不是通用高斯平滑，也不是填洞。当前 C++ 实机参数 do_depth_preprocessing=false，配置的膨胀次数并未生效。Python 新版在 CPU 端剔除非有限值、过近和超远深度，但检查到的接口没有另行组织同等的无效区域膨胀步骤。

边缘错误匹配、反光或遮挡区的错误有效值，不一定会被“非零且在量程内”的检查发现。扩大无效区域可能减少边缘污染，也会损失有效细节；不能直接宣称开关打开后必然提高完整性。

### 4.4 颜色投影融合与 Mesh

颜色使用自己的内参和图像时间位姿，对已重建几何进行投影融合，并非把同下标的 RGB 像素直接贴到未对齐深度像素。深度与颜色使用不同 optical frame 是正常设计，关键是两者外参和时间正确。

Mesh 从 TSDF 零等值面提取，min_weight 决定哪些低置信体素可参与，weld_vertices 合并相同顶点以改善网格连接；顶点焊接不能修复错位观测或恢复小于体素的孔洞。

### 4.5 增量 GPU 处理和地图维护

稀疏体素块、GPU 并行投影融合、更新块跟踪、ESDF 增量传播、独立频率调度用于控制开销。raycast_subsampling_factor 影响可见块计算，不应直接解释成“每四个 ESDF 点只保留一个”。增量 Mesh 消息减少传输，不直接决定几何分辨率。

衰减与地图半径清理可管理旧观测和资源，但会丢失历史信息。它们不是手眼或时间误差校正。当前 C++ 实机的 decay_tsdf_rate_hz 与半径清理频率均为零；手、机械臂和其他运动物体进入静态重建时，旧错误观测可能保留。

移除人体识别不改变静态 TSDF 的基本公式，但同时移除动态对象掩码后，不能期待原来的人体排除能力仍然存在。机械臂相机运动不等于场景物体动态，位姿正确时静态融合本应处理相机运动。

## 5. 本轮已经确认的现象

| 证据 | 可以得出的结论 | 不能得出的结论 |
|---|---|---|
| C++ 深度累计29340，当前约29.97 Hz；颜色累计1，当前0 Hz | 颜色成功计数未持续推进，需优先排查颜色链路 | 不能说明所有 Python 版也有同样颜色故障 |
| 导出 PLY 为1736顶点、2014三角面；1509顶点RGB为127/127/127，约87% | 大面积灰色已写在数据中，无法靠RViz恢复纹理 | 仅凭灰色不能确定某个TF必然错误 |
| CloudCompare 中几何仍不理想（用户观察） | 至少部分问题在地图数据中 | 不能完全排除RViz另有增量显示问题 |
| 当前 v=0.03、截断4voxel、Mesh阈值0.1、Wmax=5 | 网格与融合参数确实生效；截断尺度0.12m | 0.12m不是Mesh误差；0.03m不是3cm精度保证 |
| 已存在可查询的TF链 | 安装与机械臂链已连通 | 不证明历史标定值及图像时刻位姿精确 |
| RGB CameraInfo 为640×480，D全零 | 当前没有非零D被忽略的直接证据 | 不等于整个相机投影模型已经校验正确 |

上述计数来自此前只读快照，不是本次三套重新执行同条件对照实验的结果。ESDF调用Hz、点云发布Hz、不同输入推动的地图版本率和端到端延迟是四个不同指标。

## 6. 为什么两套新程序会出现类似问题

共同依赖链为：真实深度质量→相机模型→图像时刻位姿→TSDF→Mesh/ESDF。两套接口差异主要在输入组织、参数、调度与发布；它们都不能凭接口类型恢复缺失几何、纠正标定或保证细节。

当前小孔、灯珠、薄片等对象可能小于一个或几个3cm体素；多帧投影融合会使边缘变圆。若再叠加位姿误差、时间偏移或边缘离群点，前景与背景可能融合成厚表面。1°角度误差在1m处约产生1.7cm横向偏差，已接近该网格尺度。这是敏感性示例，不是本轮实测标定误差。

ESDF observed 模式还会显示自由空间中的有效距离体素，不能按“所有彩点都应在障碍物表面”评价重建。三个程序的 ESDF 点数、稀疏程度和颜色映射不统一时，视觉对照会混淆距离场展示与几何精度。

## 7. 优劣与适用性

| 实现 | 优势 | 当前限制/风险 |
|---|---|---|
| realtime | SDK校准信息集中；可读设备深度比例；直接采集少一层ROS输入配对；线程及缓存复用 | 依赖自有采集和时钟管理；默认颜色/Mesh关闭；默认规则查询网格与原生体素不同；底层参数多隐式；不自动拥有高级滤波 |
| ur7e Python | 独立工作区；显式MapperParams；原生体素导出；GPU操作单所有者，降低并发风险 | ROS消息/CPU-GPU传输和输出读回成本；自行承担图像模型/队列/位姿接入；未等价覆盖所有官方预处理能力 |
| ROS C++ | 官方ROS主流程、分块Mesh及服务、参数层次完整；主计算在CUDA | 当前依赖workspaces部署；颜色融合异常待定位；配置覆盖与NITROS配对增加排查维度；最新官方网页不等于已安装旧版行为 |

C++路线的包装开销可能较小，但性能由GPU工作量、地图规模、读回、订阅和输入调度共同决定。Python接口也调用原生CUDA，不能按语言名称给出实时性或质量排名。

## 8. 后续验证与优化优先级（尚未执行）

1. 先定位C++颜色融合仅一次：检查图像与CameraInfo配对频率、各自时间戳、采集时刻TF、图像转换日志及原生rates。不要以RViz能看到RGB代替融合成功。
2. 核对两组手眼标定及SDK内部外参；保持原color标定语义，组合得到depth位姿。不要用旧版的不同标定盲目覆盖当前值。
3. 同一静止场景分别导出原始反投影深度、单视角TSDF Mesh、多视角Mesh。先确认错误出现在原始深度还是累积融合。
4. 同一套安装先静止10秒，再缓慢运动；分别建新地图。静止好而移动差时优先时间/位姿；静止也差时优先量程、深度质量和体素尺度。
5. 录制同一份输入和TF，统一640×480、体素、范围、权重、截断、颜色/Mesh开关与输出频率后比较。realtime当前直接SDK输入不直接支持同一ROS bag回放，需要后续适配或设计固定输入驱动；本次没有实现。
6. 统一输出语义：用同一表面PLY评价几何，用同一距离范围/选择模式评价ESDF，不以点多、稀疏或颜色丰富判精度。
7. 基线正确后逐项评估无效边缘抑制、细体素、权重/截断与低置信Mesh阈值；不能同时改多项后归因。细体素增加内存及计算量，局部/分层地图属于后续设计，不是当前已经实现的功能。

建议结果表同时记录：原始输入Hz、成功深度/颜色融合Hz、ESDF调用Hz、点云发布及推进时间戳Hz、数据年龄P50/P95、地图规模、有效观测比例、表面点到参考平面误差、边缘厚度、颜色覆盖率及运动前后几何差异。未经参考真值，不把“更平滑”写成“更准确”。

## 9. 代码与官方资料索引

代码（仓库根目录为 `/home/czz/perception_eye_in_hand`）：

- `src/realtime_nvblox/realtime_nvblox/{realsense_source,robot_pose,runtime,nvblox_backend}.py`
- `src/realtime_nvblox/config/{default,robot_eye_in_hand}.yaml`
- `src/realtime_nvblox_2/realtime_nvblox/nvblox_backend.py`：同名副本，文件内容与主目录不同，本次不混为同一版本。
- `ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/{node,backend}.py`
- `nvblox_in_hand/src/nvblox_in_hand/src/{mapper_node.cpp,esdf_pointcloud_exporter.cu}`
- `nvblox_in_hand/src/nvblox_in_hand/{launch/perception/nvblox.launch.py,config/nvblox/nvblox_base.yaml}`
- `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp`
- 同一官方源码树 `nvblox_core/nvblox/src/integrators/projective_tsdf_integrator.cu`
- 同一官方源码树 `nvblox_core/nvblox/include/nvblox/integrators/internal/impl/weighting_function_impl.h`

官方资料（在线latest仅解释概念与接口，不能用其默认值替代本地旧版）：

- [nvblox技术原理、静态/动态重建与RealSense分流](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)
- [官方参数：预处理、Mesh、ESDF与地图维护](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)
- [官方RealSense排障](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_realsense.html)

本轮交付只有这份分析文档，没有修改程序、配置、依赖或机器人状态。
