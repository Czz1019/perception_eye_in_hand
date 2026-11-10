# workspaces与两个in-hand功能包的融合方法及数据流程对比

日期：2026-11-10。依据本机源码、参数文件及已有颜色开关对照结果，未修改源代码、未执行新的仿真。本报告关注实际调用关系，不以Python/C++语言名称代替算法判断。

## 1. 结论先行

- workspaces主线与nvblox_in_hand C++版复用同一官方ROS处理类及底层库；C++版主要替换应用位姿来源、精简传感器/识别任务、调整参数，并增加ESDF导出与计数。
- ur7e_nvblox_in_hand Python版重新实现ROS输入、队列、TF查询及发布，通过nvblox_torch调用另一版本的nvBlox C++/CUDA，核心仍是投影TSDF融合、颜色层积分、Mesh提取与增量ESDF。
- 三者不是“官方会融合、两个简化包只堆点”的差别；点云直接反投影与体素融合本来就是不同表示。真正不同集中在输入合同、接受帧集合、位姿来源、参数、核心版本、调度与输出。
- 当前四组开关颜色测试说明：该静止仿真场景的Mesh几何缺失不是颜色模块直接造成。它尚不能证明所有实机条件正常，也未包括一个独立的workspaces原入口同录包运行组。

## 2. workspaces实际组织与数据流

主体目录：`/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox`。

```text
nvblox_examples/nvblox_examples_bringup/realsense应用入口
  ├─ sensors/realsense.launch.py：驱动、相机配置、可选splitter
  ├─ perception/vslam.launch.py：红外/IMU位姿估计
  ├─ perception/nvblox.launch.py：模式、重映射、配置层次
  └─ visualization/...：显示
              ↓
RGB/Depth + 各自CameraInfo + 时间戳 + 相机位姿
              ↓
nvblox_ros::NvbloxNode
  ExactTime配对 → 队列 → 图像时刻TF就绪检查 → 限频 → 图像转换
              ↓
MultiMapper / Mapper / CUDA integrators
  Depth → TSDF      RGB → ColorLayer（读取已有TSDF）
                ├─ Mesh
                └─ ESDF
              ↓
分块地图/Mesh消息、切片、服务及RViz插件
```

官方双IR流主要提供VSLAM位姿，Depth/Color进入nvBlox。splitter按投射器状态选择帧，不是用于对Mesh做几何平滑的模块。用机械臂位姿代替VSLAM后，不需要为了几何融合再强制订阅infra1/2，但仍必须提供准确时间对应的相机位姿。

### 本地workspaces并非一组未经修改的官方参数

读取到的本地base.yaml为v=0.02、深度30Hz、颜色5Hz、Mesh/ESDF/层5Hz、TSDF衰减2Hz、半径清理1Hz，并且use_color=false。实际launch可能覆盖，不能把这些文件值直接当作之前测试的最终运行值。

本地realsense_emitter_flashing.yaml活动项为depth_profile=640x480x30、infra_profile=640x480x60、emitter_on_off=true。它与官方文档常见的depth/IR 60Hz分流示意不完全相同；如果亮灭交替筛选作用于深度，真正送入建图的频率要实测，不能自动认为30Hz配置就有30Hz有效亮帧。

本地Gazebo适配还存在depth CameraInfo转发到color CameraInfo，以及camera_link到base_link单位别名。这些是固定相机示例兼容处理，不适合直接复制进UR机械臂实机。因此“参考workspaces”应区分核心官方模块与本地场景适配。

## 3. C++版nvblox_in_hand：哪些相同，哪些改变

应用launch沿用层次化配置：base→D435→UR7e→profile→仿真/实机→可选自定义→CLI。它映射camera_0输入、启用static_tsdf、禁用雷达，主要位姿来自机械臂TF。

默认official后端加载官方组件；diagnostic后端继承NvbloxNode，其processDepthImage/processColorImage/tick/processEsdf继续调用基类。增加的是成功时间戳计数、原生observed ESDF体素导出和监测，不重新编写TSDF积分公式。

因此以下部分与本地workspaces主体相同：

- Image与CameraInfo ExactTime配对。
- NITROS图像接入和图像编码转换。
- 官方消息队列、图像时刻TF查询及shouldProcess门限。
- MultiMapper、TSDF/Color/Mesh/ESDF积分器。
- 官方分块Mesh与保存服务。

主要应用差异：

- 位姿由机械臂而非VSLAM提供；深度/颜色各自使用光学frame。
- 当前realtime profile为v=0.03、深度颜色40Hz上限、ESDF33.4Hz、Mesh/层5Hz、关闭衰减及半径清理。
- 人体/动态任务没有作为当前静态建图流程运行，不再自动拥有这些掩码剔除能力。
- 额外完整ESDF点云导出会增加扫描、GPU读回、ROS传输与显示成本。

容器组合与独立节点的组织不同，不能保证端到端传输成本完全相同；也不能把NITROS标签直接理解成所有路径绝对零复制。

## 4. Python版ur7e_nvblox_in_hand：重新组织的环节

```text
ROS Image ───────────────→ 各自deque（容量3）
ROS CameraInfo ──────────→ 最近有效内参缓存
                         ↓
尺寸/frame检查、时间戳检查、0.5s图像年龄、融合限频
                         ↓
图像时间的base_link←optical TF
                         ↓
CPU按encoding/step解析 → 米制深度/RGB → CUDA Tensor
                         ↓
单GPU所有者线程调用nvblox_torch Mapper
                         ↓
TSDF变化 → ESDF更新；Mesh/ESDF读回 → 输出线程
                         ↓
完整MarkerArray Mesh / 原生ESDF PointCloud2
```

它没有ROS C++主节点的ExactTime图像/内参配对、NITROS接入和相同队列实现。相机模型及预处理合同由项目自己维护。wheel和ROS核心版本不同，算法同类不意味着所有默认参数、实现细节或内存路径相同。

## 5. 输入与处理策略逐项对照

| 环节 | workspaces ROS主线 | nvBlox C++ in-hand | Python in-hand |
|---|---|---|---|
| 深度来源 | RealSense原始流/可选splitter；也有本地仿真relay | 当前直接订阅独立D435或仿真Depth | 订阅ROS Depth |
| 红外流 | 主要供VSLAM | 当前机械臂TF，不使用VSLAM | 当前机械臂TF，不使用VSLAM |
| CameraInfo | 与本路图像精确时间配对 | 同官方 | 缓存最近内参，核对尺寸/frame |
| 内参模型 | 本地cameraFromMessage读取K构造针孔模型 | 同官方 | K构造无畸变Sensor；未实现注释提到的P选择 |
| 位姿 | 示例VSLAM/TF，具体模式可变 | 机械臂TF，use_tf_transforms=true | 每帧查询相同方向TF |
| 图像等待队列 | 官方list，当前最大长度10，满时丢最旧 | 同官方 | deque容量3，满时覆盖最旧 |
| TF就绪处理 | 每次tick扫描队列，收集所有就绪项处理 | 同官方 | 检查队首；未就绪时等待，可能阻挡后面的帧 |
| 过期策略 | 当前检查的processQueue无Python同等0.5s年龄剔除；有队长/TF/时间门限约束 | 同官方 | 明确最大输入年龄0.5s |
| 深度转换 | 官方ROS/NITROS编码转换 | 同官方 | 显式处理字节序、行跨度，16UC1乘0.001、32FC1米 |
| 范围剔除 | 底层积分距离及驱动有效值合同 | 当前最大3m | CPU显式剔除<0.1和>3m，再交底层 |
| 无效区域预处理 | 核心支持可选膨胀，本地base关闭 | 当前关闭 | 没有项目层同等膨胀链；只做基础有效值过滤 |
| 调度 | ROS tick组织各任务 | 同官方，追加诊断导出 | 自有单GPU工作循环与独立输出线程 |

重要细节：官方processQueue先扫描全部就绪项、解锁后逐项处理，Python每轮通常为深度/颜色各取一个候选。两者在突发输入、TF迟到、队列积压时接受的帧集合可能不同，这会影响调用频率和运动场景，但不能仅凭静止Mesh缺面确定是哪项造成。

## 6. 融合方法：共同的算法与具体参数

### 6.1 深度融合

共同采用投影TSDF：确定本帧可见块，对体素投影查询深度，得到投影距离，再按观测权重与历史权重累积。概念表达为：

```text
Dnew = clip((Wold*Dold + w*d) / (Wold+w), -mu, mu)
Wnew = min(Wold+w, Wmax)
```

它不是把深度点直接连成网格，也没有因改用官方接口就自动执行ICP、手眼修正或位姿再优化。增量跟踪变化块，Mesh和ESDF更新会复用已有地图。

本地workspaces及当前两个包主要使用inverse_square_tsdf_distance_penalty权重：距离权重降低远处观测影响，附加TSDF距离惩罚减少视角变化影响。当前C++核心的inverse_square以体素相机深度计算，不应泛化为所有版本都严格按测量深度计算。

| 参数 | workspaces本地base文件 | C++当前profile/应用 | Python当前配置 |
|---|---:|---:|---:|
| 体素 | 0.02m | 0.03m | 0.03m |
| 最大积分距离 | 5m | 3m | 3m |
| 截断距离 | 4voxel | 4voxel | 4voxel |
| 对应截断米值 | 0.08m | 0.12m | 0.12m |
| 最大TSDF权重 | 5 | 5 | 5 |
| Mesh最小权重 | 0.1 | 0.1 | 0.1 |
| 视图块raycast子采样 | 4 | 4 | 4 |
| 深度预处理膨胀 | 关闭 | 关闭 | 未单独组织等价流程 |
| 衰减/半径清图频率 | 2Hz/1Hz | 0/0 | 0/0 |

左列不是已验证的历史运行参数；必须考虑launch覆盖。2cm和3cm对小物体的细节与截断米值都不同，不能称为相同精度比较。raycast_subsampling用于可见块计算，不能当作点云直接四倍降采样。

### 6.2 颜色融合

workspaces/C++核心读取已有TSDF，从彩色相机视角sphere tracing出合成深度，再按可见性向ColorLayer融合RGB。Python调用wheel add_color_frame，仍是原生颜色积分，而非ROS端简单贴图。

颜色和深度分别使用对应CameraInfo、光学frame及时间；无需强制将原始深度改成color frame。两路图像之间不一定必须精确同时，但每路图像都需要自己的正确位姿，运动时还要保证场景一致。

颜色不直接修改TSDF几何。调用完成也不等于这帧实际给大量体素着色；这正是仅用color_integrated计数判断纹理质量的局限。

### 6.3 Mesh

workspaces与C++版在官方网格积分器中提取TSDF零等值面；Python通过wheel update_color_mesh/get_color_mesh读取对应网格。角点权重/邻块有效性条件会使部分深度支持区域没有三角面。

C++官方分块消息与Python完整MarkerArray是发布方式不同，不意味着其中一套是“点云假Mesh”。Python保存PLY通过wheel自身转换，绕过MarkerArray快速编码，已提供独立排查路径。

### 6.4 ESDF

三者使用nvBlox增量ESDF核心，而不是从显示出来的Mesh重新计算。C++官方tick按ROS时间调度调用ESDF，即使没有新输入也可能调用；Python仅在新深度将地图标记为dirty后调用。

C++新增与Python桥新增的3D点云导出，都可能遍历较大范围的原生ESDF有效体素；因此“内部增量更新”与“完整点云读回”必须分开评价。

## 7. 限频与实时性差异的准确解释

本地官方shouldProcess使用 `elapsed > 1/hz`。Python使用图像纳秒差与round(1e9/hz)比较，未达到间隔则跳过。两者都可能受名义30Hz时间戳量化/抖动影响，当前40Hz门限用于容纳30Hz输入，而不是生成40帧。

此外：官方调度主要使用ROS时钟，Python ESDF/Mesh due()使用墙钟monotonic。加速、暂停或低实时因子的仿真回放下，相同“Hz设置”不一定意味着相同墙钟调用频率。

已有固定录包输入深度约20.1Hz、RGB约16.4Hz；Python融合19.68/16.30Hz不能称作30Hz输入再次被丢成18Hz。C++较低融合比例需用配对、TF、限频、队列与转换的逐级计数定位。不能单凭表格判定底层融合算法更慢。

原生GPU积分、读回、ROS发布和显示是不同耗时。本轮接收大型Mesh的监测出现消息接收不完整迹象，不以监测器接收Hz冒充发布Hz。

## 8. 已有对照证据与目前能推导的结论

颜色开/关固定录包实验：两个后端四组均3006顶点、4306三角面、84.71%有效深度覆盖，相对输入深度P95约17.80mm。各后端ON/OFF轮廓IoU为1，差异只有少量亚毫米。

这支持：该静止场景两套包装层得到的几何相近，颜色模块不是缺面的主要直接原因，优先检查共同的TSDF→Mesh离散和有效性条件。

它不能支持：所有后端使用相同core版本、当前workspaces应用入口一定具有同样最终参数、或者直接运行原workspaces就必然更好。本次四组未单独重跑workspaces原入口；C++版调用官方主类是代码复用证据，不替代独立第五组运行证据。

## 9. 推荐的下一轮对比方式

1. 对同一固定录包增加workspaces原生NvbloxNode静态组，只替换输入重映射与全局TF，不启动第二套相机或VSLAM。
2. 导出最终实际参数，统一体素、截断米值、范围、权重和地图维护开关，而不只统一voxel_size。
3. 逐级统计图像到达→CameraInfo配对→TF就绪→限频→有效体素更新，不只统计API调用。
4. 对共同缺面区域导出TSDF权重、距离及Mesh角点拒绝原因，判断是融合覆盖丢失还是网格提取拒绝。
5. 单独测试无可视化、原生Mesh发布、完整ESDF点云导出三种负载；避免把发布开销归咎于TSDF核心。

以上是后续建议，本报告没有执行这些新增组，也未修改代码。

## 10. 代码和既有结果索引

- [workspaces应用编排与重映射](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py)
- [官方图像队列实现](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/include/nvblox_ros/impl/nvblox_node_impl.hpp:34)
- [官方tick与处理流程](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:578)
- [官方Mapper深度与颜色积分](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mapper/mapper.cpp:268)
- [C++包参数与启动适配](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/nvblox.launch.py)
- [C++诊断派生类](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp)
- [Python输入与调度](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py)
- [Python原生后端与参数映射](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py)
- [已有四组颜色开关测试](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/COLOR_ON_OFF_TWO_BACKENDS_TEST_20261110.md)

官方概念参考：[技术说明](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)、[nvblox论文](https://arxiv.org/html/2311.00626v2)。具体默认值及实现判断以本地读取版本为准。
