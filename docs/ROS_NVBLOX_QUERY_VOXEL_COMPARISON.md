# 官方 ROS 移植版与 ur7e_nvblox：查询、体素尺寸和实时性差异

分析日期：2026-10-26。本文只分析源码和已有实验记录，没有修改实现、参数或 RViz 配置，没有启动仿真、GPU 基准或机械臂。

本文是用户授权修改之前的静态分析快照，下文“当前代码”及行号均指修改前版本。后续实际修改和仿真结果见[实施与实测说明](ESDF_ROS_ALIGNMENT_IMPLEMENTATION.md)，不要将本文的旧默认值当成实施后的配置。

## 1. 结论与对比对象

**当前查询与显示的组织方式确实存在不必要的耦合和开销，但现有证据不能支持“大部分时间都耗在查询中”。** 已能直接从代码确认的差异是：

1. `ur7e_nvblox` 默认在每次 ESDF 更新后，查询固定三维盒中的所有采样位置并读回 CPU；即使没有点云订阅者，也执行这一步。
2. 该盒的采样间距是 `grid_resolution_m`，与内部地图的 `voxel_size_m` 独立。只改后者，不会改变默认查询网格的形状、总点数和间距。
3. 官方 ROS 移植版把增量地图更新、地图显示和三维服务导出分开。三维服务按 **ESDF 原生体素尺寸** 导出，配套显示节点也按服务返回的体素尺寸生成坐标，所以改变体素尺寸会改变输出网格及显示点间距。
4. 官方 ROS 的 Mesh、体素块、二维 ESDF 切片、三维 ESDF 服务是不同输出。它们的计算量和视觉含义不能混为一谈；流畅的 Mesh 画面不能证明全域三维距离导出也达到同样频率。
5. workspaces 中还存在一个目标 30 Hz 的三维查询 launch，但它读取已有 ESDF，不强制更新地图，且只允许一个请求在途。目标查询频率、实际响应频率和新地图频率必须分别测量。

用户已明确排除 `/home/czz/nvblox_ws/src` 下的 `isaac_style_nvblox`、`my_nvblox`、`realtime_nvblox_ros`，因此**这些实现不作为本文的正常版本基线**。实际依据为：

| 对象 | 本次读取的位置 | 用途 |
| --- | --- | --- |
| 当前手动采集感知 | `/home/czz/perception_eye_in_hand/src/ur7e_nvblox` | 当前问题实现 |
| 当前共享 Python 后端 | `/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox` | 网格构造、原生调用、点云打包 |
| 官方 ROS 移植工作区 | `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox` | 官方 ROS 节点和内置 C++ 核心 |
| 配套三维查询节点 | `/home/czz/workspaces/isaac_ros-dev/src/live_esdf_query` | 服务请求与三维点云显示 |

ROS 库 HEAD 为 `bc181942202b24fd47b2f3db8f77c3c82d484566`，但本地 `nvblox_base.yaml` 和 `realsense_emitter_flashing.yaml` 有修改，本文参数以**本地文件**为准，不能当成该提交的上游默认值。取证文件哈希见 [本次证据清单](esdf_analysis/ros_comparison_evidence.json)。

本次可见的项目 `src/` 仍为原来的 13 个目录，没有发现上述 ROS 库在其中的另一份副本。因此本文直接读取 workspaces 中的实现，不把用户提到的“已加入项目”擅自对应到其他包，也不据此推定当前运行的二进制必然就是这些源码编译出的版本。

## 2. 先区分“计算 ESDF”和“查询 ESDF”

两条链路都使用 nvBlox 原生地图能力。当前共享后端的 `update_esdf()` 调用 `mapper.update_esdf(0)`，并没有在 Python 中用全部网格点重新计算一遍最近障碍距离。

```text
深度 + 图像时刻相机位姿
             ↓
       原生 TSDF 融合
             ↓
       原生 ESDF 更新          ← 地图内容在这里更新
             ↓
  查询/导出 → CPU → ROS → RViz  ← 读取与展示，不等于重新生成 ESDF
```

ROS 内置核心 `Mapper::updateEsdf()` 在 `mapper.cpp:495–520` 获取待更新块，交给 `EsdfIntegrator::integrateBlocks()`，再标记完成。传播影响可以扩展到邻块，因此“增量”也不等于只访问本帧命中体素。

当前代码的问题主要位于更新后的查询、读回与展示调度。**查询未知位置不会自动在原生地图里分配一个 ESDF 体素。** RViz 中完整网格的点数不能用来推算原生地图分配了多少块。

ESDF 中正距离的已观测自由空间是避障所需净空信息；未知空间才是没有可靠距离的区域。不能通过删除所有自由体素解决实时性问题，否则会破坏距离场的用途。

## 3. ur7e_nvblox 当前究竟执行了什么

关键源码：

- [仿真配置](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/config/gazebo.yaml)
- [融合和查询线程](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py:152)
- [查询缓冲复用](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/esdf.py:12)
- [规则网格构造](/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox/nvblox_backend.py:153)

默认仿真参数为：内部体素 `0.03 m`，查询间距 `0.05 m`，查询范围 `2 × 2 × 1.5 m`，ESDF 目标更新 `15 Hz`，输出模式 `dense`。

`TfMapper._worker()` 在同一融合工作线程中依次进行：

1. 等待最新深度与相机信息，查询该图像时刻的 TF，检查帧龄。
2. 解码、深度单位转换、裁剪与降采样，准备相机位姿。
3. 调用深度融合；达到 ESDF 更新时刻后调用 `update_esdf()`。
4. 调用 `query_prepared()`，等 GPU 结果可读并复制到 CPU。
5. 统计已知样本数，将完整结果交给另一个发布线程。

`query_prepared()` 已经复用 Nx4 GPU 查询坐标及输出缓冲，不是逐点 Python 查询，也不是每帧重新上传整个查询坐标。当前 dense 分支也没有额外 GPU clone。把这些已做的优化再作为“需要新增的优化”不准确。

但它仍存在以下应用层开销：

| 位置 | 当前行为 | 影响 |
| --- | --- | --- |
| `node.py:224–230` | 每次 ESDF 更新后完整查询并同步读回 | 下一帧融合必须等这一步结束 |
| 同一位置 | 查询前不检查点云是否有人订阅 | 关闭 RViz 可减少打包发布，但不会取消固定网格查询 |
| `node.py:268–290` | 发布端才按订阅需求打包完整/已观测点云 | 过滤发生在读回之后，不能减少已执行的查询量 |
| 规则网格 | 对所有盒内位置查询，包括地图之外和未知区域 | 产生可避免的枚举、读回和完整点云负担 |
| 两个点云均被订阅 | 分别构造完整点云和已知距离带点云 | 增加打包、复制、DDS 与 RViz 工作量 |

这里的串行依赖确实值得调整，但还不能据此判断它是当前现场的最大瓶颈。TF 等待、图像处理、输入频率、GPU 调度、发布和 RViz 都需要独立计时。

## 4. 为什么 voxel_size 改了，ur7e_nvblox 的点数似乎没变

### 4.1 当前存在三个独立的尺度

| 尺度 | 当前参数或配置 | 控制内容 |
| --- | --- | --- |
| 原生地图尺度 | `voxel_size_m = 0.03` | TSDF/ESDF 离散精度、块大小、内部存储和计算规模 |
| 查询/导出尺度 | `grid_resolution_m = 0.05` | dense 输出采样坐标及总样本数 |
| 屏幕点大小 | `esdf.rviz` 的 `Style: Points`、`Size (Pixels): 3` | RViz 中每个点占多少屏幕像素 |

改变第一项，不会自动改变后两项。这是代码显式解耦的结果，**不是证据表明 voxel_size 没传入 Mapper 或原生 ESDF 没工作**。它确实传入了共享后端构造 Mapper 的位置。

当前网格每轴样本数是 `floor(尺寸 / 查询间距) + 1`，因此：

```text
N = 41 × 41 × 31 = 52,111
```

只把地图体素从 3 cm 改成 2 cm、5 cm 或其他值，只要查询间距与查询盒不变，完整 dense 输出仍是这 52,111 个位置。内部距离数值、已知覆盖、近表面细节和运行代价仍可能改变；已观测距离带的最终点数也可能变化，但不是按原生体素数直接输出。

比如保持地图体素 3 cm，只把查询间距改为 10 cm，规则样本数便变成 `21 × 21 × 16 = 7,056`。此例只是解释参数关系，**本次未修改配置**。

### 4.2 官方 ROS 三维服务与配套显示的尺度链路不同

官方 ROS 的 [ESDF 服务转换器](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu:50) 明确执行：

```text
response.voxel_size_m = esdf_layer.voxel_size()
原生 ESDF 层 + 请求 AABB → 原生体素对齐的 dense grid
```

底层 `layer_to_3d_grid_impl.cuh:119–150` 用 `1 / layer.voxel_size()` 将 AABB 转成整数体素索引，输出维度为 `max_index - min_index + 1`。它没有额外独立的 `grid_resolution_m`。

配套 [global_esdf_query_node.cpp](/home/czz/workspaces/isaac_ros-dev/src/live_esdf_query/src/global_esdf_query_node.cpp:361) 读取服务返回的体素尺寸，并用：

```text
x = origin.x + (ix + 0.5) × voxel_size
y = origin.y + (iy + 0.5) × voxel_size
z = origin.z + (iz + 0.5) × voxel_size
```

计算显示坐标。显示循环又按 `stride` 跳格，因此其显示点间距约为 `stride × voxel_size`。**这是你观察到“原 ROS 版本的点数和效果随 voxel_size 变化”的直接代码原因。** 不需要假定两边使用了不同 ESDF 数学算法。

在范围和有效覆盖近似相同时，三维体积采样规模约随 `voxel_size^-3` 变化，二维切片约随 `voxel_size^-2` 变化；表面 Mesh 则还受几何、截断带和三角化影响。过滤后的实际点数不会严格遵循固定倍数。

当前包还有 `sparse` 分支：它从正 TSDF 权重筛选原生体素中心候选，再查询原生 ESDF 并剔除未知；这条分支的点数和间距会受原生体素尺寸影响。它不是默认 dense 行为，也不是完整原生 ESDF 层遍历接口，不能根据未输出某位置就判断该位置自由。

## 5. 官方 ROS 三维导出为什么有更适合规则场的访问方式

ROS 服务的 `voxelLayerToDenseVoxelGridInAABBAsync()` 按块发射 CUDA 工作：每个体素块对应一个线程块，由一个线程查询块哈希指针，其他线程共享该指针并读取块内连续体素；块不存在则写未知默认值。转换器使用 `observed` 判断有效性，并通过 `sqrt(squared_distance_vox) × voxel_size` 与 `is_inside` 恢复有符号米制距离。

这与当前应用的 Nx4 任意位置查询接口用途不同：后者保留任意坐标/球半径查询的通用性，前者针对原生对齐的规则场批量导出，能利用块内访问结构。**在需要完整规则场时，原生块式导出是值得采用的实现方向。**

但不能据此直接给出“快多少倍”：当前 Python wheel 与 ROS 内置核心并非已证明同一构建，坐标数、分辨率、AABB、GPU、同步与 ROS 传输也不同，必须统一输入实测。

尤其不能说“官方没有 dense 查询和读回开销”。该服务同样构造包含未知单元的 dense grid，复制 GPU→CPU pinned grid，再转为 ROS `Float32MultiArray`，最后同步 CUDA stream。它也会因查询体积和分辨率变得很慢。

本地该转换器实际输出三个空间维度的一路距离值；不能仅根据 `EsdfAndGradients` 名字就假定响应一定包含额外梯度通道。配套客户端有通道数兼容处理。

## 6. 官方 ROS 的“实时效果”必须按输出路径解释

### 6.1 地图更新和显示不必同频

本地 [nvblox_base.yaml](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml) 设置：

| 工作 | 本地目标配置 |
| --- | --- |
| 深度融合 | 30 Hz |
| ESDF 更新 | 5 Hz |
| 地图层发布 | 5 Hz |
| ESDF 模式 | `3d` |
| 原生体素尺寸 | 0.02 m |
| 三维服务是否强制刷新 | 由请求的 `update_esdf` 决定 |

`NvbloxNode` 的 tick 分别检查深度队列、ESDF 更新和可视化时机；三维导出请求通过任务队列执行。它仍有串行和同步依赖，不能夸大成“所有任务完全并行”。

对比当前 ur7e 仿真 15 Hz ESDF 目标，ROS 这份配置每秒原生 ESDF 更新目标只有三分之一。不同工作频率本身就会影响资源占用；这些数值是源码配置目标，不是本次实测频率。

### 6.2 workspaces 的三维查询 launch 并不只请求低频小地图

必须区分 C++ 参数默认值和 launch 覆盖值。[实际 launch](/home/czz/workspaces/isaac_ros-dev/src/live_esdf_query/launch/global_esdf_query.launch.py) 设置：

```text
AABB = 6 × 6 × 2.5 m
rate_hz = 30
stride = 2
update_esdf = false
距离显示过滤：0.03 ～ 1.0 m（默认取绝对距离）
```

虽然注释写了“低频即可”，**实际 rate_hz 是 30，不能依据注释写成 0.5 Hz**。0.5 Hz 是 C++ 节点未被覆盖时的默认值。

`timerCallback()` 在已有请求等待响应时直接返回；收到响应并处理点云后才允许下一次请求。因此 30 Hz 是触发目标，并不保证每秒得到 30 次响应，更不保证每秒有 30 张不同的 ESDF。

`update_esdf=false` 读取当前地图，地图若仍按 5 Hz 更新，多个响应可以来自同一次地图更新。客户端点云重新打当前时间戳，也不能证明距离场内容刚刚更新。

按本地 2 cm 体素和该 AABB，整数边界公式得到 `301 × 301 × 126 = 11,415,726` 个服务单元，单通道 float 距离数据约 **45.66 MB/次**，未计消息开销。`stride=2` 的显示候选约 143.65 万个，之后才按距离过滤。

**stride 只减少客户端遍历和显示点数，不减少本次服务返回的完整 dense 数据量。** 如果真的持续完成 30 次/秒，仅距离数组原始数据就约 1.37 GB/s。这说明这条 ROS 查询链路也不能凭源码认定低开销。运行时可能用了其他参数、实际响应远低于目标，或者用户观察的是 Mesh/另一话题；目前没有这次运行日志，不能替用户选定其中一个解释。

### 6.3 其他显示路径为什么不会画出同样的灰色三维格点

| 输出 | 实际内容 | 与当前完整 dense 点云的区别 |
| --- | --- | --- |
| 官方 Mesh | TSDF 等值面三角网格 | 显示表面，不把整个自由/未知体积画成点 |
| TSDF/颜色体素块 | 有权重及 TSDF 距离过滤后的原生块内体素 | 位置和物理显示尺寸直接来自原生体素 |
| 二维 ESDF 切片 | 选定平面上的距离图，GPU 转点云时跳过未知 | 输出维度少，未知不进入最终点云 |
| 配套三维查询点云 | 服务返回的原生 dense 场，经 stride 和距离带过滤 | 不保留全部未知点，采样跟随原生体素尺寸 |
| ur7e 完整点云 | 所有固定查询位置，包括未知 | 很容易形成灰色立体格点罩住场景 |

官方 `layer_publishing.cpp` 按订阅选择要序列化的层；Mesh 只有需要流式发布时才更新。核心 `serializeSelectedLayers()` 使用待发布块追踪，RViz 插件保留各块并更新/移除对应对象。传统 Marker 路径则有整条消息重建的成本，不能把所有输出都称为块增量消息。

本地 `processEsdf()` **仅在 `esdf_mode == k2D` 时调用二维切片点云发布**；当前 YAML 是 3D。因此该切片路径用于解释官方支持的不同输出，不能声称它就是当前 3D 配置下实际发布的点云。也不能把二维投影距离场替代机械臂需要的完整三维距离场。

代码中虽定义了 `EsdfVoxelFilter(is_inside && observed)`，但本地 `LayerPublisher` 的实际创建和发布路径不能仅凭这一定义就推断“默认发布三维 ESDF 占据体素”。应以所订阅的话题和实际调用为准。

## 7. 灰色点阵、自由空间与实际计算量

当前共享 `esdf_cloud()` 在 `full_grid=True` 时保留全部样本，并把未知颜色设为 `[55,55,55]`；已观测输出才剔除未知并保留 `abs(distance) <= max_visualized_distance_m` 的距离带。0.5 m 距离带本身也可能包含大量有效自由空间点，并不只是一层障碍表面。

默认完整点云按 `x,y,z,rgb,distance` 每点 20 字节，52,111 点约 1.04 MB/次，15 Hz 约 15.63 MB/s 原始点数据，另有 DDS、CPU 复制和渲染开销。距离 CPU 读回本身则仅约 `52,111 × 4 = 208,444` 字节/次；二者不能混用。

所以需要分别统计：原生地图块数、内部观测体素数、查询样本数、查询有效数、发布点数及 RViz 帧率。完整查询点云的 52,111 个点既不是地图内存体素总数，也不是 52,111 个障碍物。

用户截图中显示项名称虽然为“Observed ESDF”，但话题被折叠，且窗口标题带未保存配置标记。灰色分布与完整网格未知点的代码行为相符，仍需实际话题确认；仅凭名称不能断言订阅了哪一路。RViz 的 31 fps 也不等于 ESDF 更新 31 Hz。

## 8. “大部分时间花在查询中”目前有多少证据

### 8.1 已有记录并不支持普遍归因

| 历史证据 | 记录 | 能支持的判断 |
| --- | --- | --- |
| `artifacts/gazebo_perception/verification.json` 静止场景 | 包含 GPU 读回的 mapping P50 约 2.03 ms；接收到结果 P50 约 103.75 ms；发布约 4.99 Hz | 在该历史版本/工况中，地图与读回阶段不足以解释大部分端到端等待 |
| `src/ur7e_nvblox/PERFORMANCE.md` 同输入合成流 | dense 输出从 10.09 到 13.99 Hz；固定点数 52,111 | 查询缓冲和导出组织有实际优化价值，但改善不是当前现场性能证明 |
| 同报告 sparse 对比 | sparse 13.75 Hz、19,714 点，dense 13.99 Hz | 输出点更少也不必然更快，遍历候选和导出成本同样重要 |

上述是已有实验记录，本次没有重新运行。不同实验版本、统计窗口和输入不能相互拼接，也不能用两个 P50 直接相减推导某个阶段的精确占比。

### 8.2 当前计时还有一个容易误判的边界

`query_readback` 在 `update_esdf()` 返回后开始，包含查询提交和 `.cpu()` 读回；CUDA 异步执行时，读回可能等待前序同流任务完成。因此这个时间即便很大，也未必全是查询 kernel 的纯执行时间。`fusion_and_esdf` 则不覆盖之前的 TF 等待和全部发布开销。

`tf_timeout_sec=0.15` 是允许等待上限，不是每帧固定消耗 150 ms。只有统计每次 TF 等待/失败，才能判断它是否造成主要延迟。

当前 `esdf_published` 在发布处理完成时递增，即使没有点云订阅者也可能递增。它不能代替某个点云话题实际收到的频率，更不能表示更新内容的新鲜度。

### 8.3 要确定主瓶颈，应做的同输入测量

后续实现与测试应保持同一深度流、图像时刻位姿、体素尺寸和积分距离，分别记录：

1. 曝光/图像时间、接收时间、取帧等待、TF 等待、丢帧原因。
2. 解码/降采样、H2D、TSDF、ESDF 的 CPU 提交时间与 GPU 完成时间。
3. 查询 kernel、D2H、CPU 过滤与打包、ROS 发布、订阅接收时间。
4. 每次结果对应的最后融合图像时间及地图版本，区分重复发布与新地图。

分别对照“只更新地图”“地图+固定盒查询”“地图+切片/表面显示”“地图+完整点云显示”。ROS 服务还应单独统计请求往返、服务任务排队、实际响应 Hz；不能只看 30 Hz timer 配置。

GPU 分段使用 CUDA events，诊断性同步必须两边采用一致口径，避免把等待时间从 ESDF 挪进查询后就误判算法瓶颈。本轮仅列出验证方法，没有增加计时或执行实验。

## 9. 对后续修改方案的具体修正方向

已有[感知修改方案](ESDF_PERCEPTION_MODIFICATION_PLAN.md)可据本次确认的官方 ROS 路径细化为：

| 建议改动 | 要解决的问题 | 预期结果与边界 |
| --- | --- | --- |
| 保留原生三维增量 ESDF，把可视化导出从每次更新后的强制步骤中拆开 | 读回阻塞下一帧融合；没有订阅仍查询 | 融合无需等待每次显示导出；收益需分段测量 |
| 默认显示 Mesh/近表面原生体素或明确的 ESDF 切片，完整未知网格改为调试选择 | 灰色未知点阵和大范围自由点遮挡 | 画面更符合几何理解；内部仍保留有效自由空间 |
| 原生体素显示与 planner 固定采样接口分开 | 把固定采样格误认为原生体素，修改 voxel_size 看不到相应显示变化 | 显示间距随原生体素变化；已有 planner 坐标接口可保留兼容 |
| 真正需要完整原生规则场时采用 C++ 原生块式批量导出 | 任意点查询接口被用于完整原生体素场导出 | 更适合块访问；仍有 dense 内存、D2H 和传输成本 |
| 按订阅/消费者需求导出，限制 ROI，分别设置融合、ESDF、显示和 planner 查询频率 | 所有工作共用一个更新节奏，浪费查询未知和重复地图 | 更稳定的资源预算；不能把降低显示频率宣传为地图加速 |
| 为 ESDF 结果携带地图版本、最后融合图像时间及导出时间 | 重复旧图被当作实时新图，发布频率误导 | 可以验证真实新鲜度和端到端延迟 |
| 保留严格图像时刻 TF、`base_link` 参考系及深度 optical frame 配套内参 | 直接照抄旧 ROS 的 odom/话题导致位姿与投影错误 | 保持手动移动眼在手上的几何一致性 |

不应直接复制旧 launch 的 6×6×2.5 m、2 cm、30 Hz 全域服务请求；不应只提高频率或缩小体素；也不应为了显示清晰将完整三维场改为二维投影场。原生体素越小，计算与导出负担一般越大；固定接口采样越细，查询负担也越大，两者需要分别控制。

## 10. 关键源码定位索引

以下行号对应本次磁盘源码，后续修改需重新核对。外部路径均为实际读取位置。

| 文件 | 位置与核对内容 |
| --- | --- |
| `src/ur7e_nvblox/ur7e_nvblox/node.py` | 177：图像时刻 TF；224–230：更新后强制查询与读回；268–290：订阅判断在发布阶段 |
| `src/ur7e_nvblox/ur7e_nvblox/esdf.py` | 12–34：查询缓冲、同步读回、固定网格；36 起：原生候选稀疏导出 |
| `src/realtime_nvblox/realtime_nvblox/nvblox_backend.py` | 86：体素传入 Mapper；110：原生 ESDF 更新；153 起：独立分辨率网格 |
| `src/realtime_nvblox/realtime_nvblox/ros_publishers.py` | 99：未知灰色；109 起：完整/已知距离带选择 |
| `src/ur7e_nvblox/rviz/esdf.rviz` | 26–27：固定像素点显示 |
| ROS `nvblox_ros/src/lib/nvblox_node.cpp` | 627：ESDF 调度；635：层发布调度；767：更新；783：仅二维模式输出切片；1354：层发布；1664 起：三维服务 |
| ROS `nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu` | 28–44：observed/符号/米制转换；50–125：原生分辨率 dense 导出及 CPU 读回 |
| ROS `nvblox_ros/nvblox_core/nvblox/include/nvblox/map/internal/cuda/impl/layer_to_3d_grid_impl.cuh` | 28–68：块共享哈希指针；119–150：原生体素边界和块式发射 |
| ROS `nvblox_ros/src/lib/layer_publishing.cpp` | 168：TSDF 过滤；270：Marker；403：块插件；638 起：订阅选择；673：所选层序列化 |
| ROS `nvblox_ros/nvblox_core/nvblox/src/mapper/mapper.cpp` | 495：增量 ESDF；875：待更新块流式导出 |
| `live_esdf_query/src/global_esdf_query_node.cpp` | 220：单请求在途；322：响应完成；361：返回体素尺寸；435 起：stride/距离过滤；459：体素中心坐标 |
| `live_esdf_query/launch/global_esdf_query.launch.py` | 实际覆盖为 30 Hz、stride 2、不强制 ESDF 更新，优先于 C++ 默认值 |

这里的 ROS 相对路径均位于 `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/`，配套查询路径位于 `/home/czz/workspaces/isaac_ros-dev/src/`。
