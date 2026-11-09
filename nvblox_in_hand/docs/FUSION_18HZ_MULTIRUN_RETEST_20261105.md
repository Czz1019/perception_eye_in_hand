# 约18Hz融合问题：30 / 33.4 / 40Hz融合门限九轮复测

后续已另做九轮全部开启三维导出的ESDF实时性对照，主要结果见[三维ESDF输出实时性报告](ESDF_REALTIME_OUTPUT_COMPARISON_20261105.md)。本文仍为原先无三维导出的融合实验，不能混用两轮数值。

日期：2026-11-05。工程：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。本次重新运行Gazebo、CUDA官方Mapper和RViz，不沿用旧轮数据作为新结果；下表全部来自本次九轮采集。

## 1. 本轮要回答的问题

输入深度和RGB都约30Hz，为何实际融合曾只有18Hz？将深度/颜色融合门限分别设置为30、33.4、40Hz后，结果是否重复出现？33.4Hz到底设置在哪个环节？

本轮**确实测试了“深度/颜色融合门限33.4Hz”**，与此前仅测试“ESDF更新/显示门限33.4Hz”不同。ESDF更新门限在本轮所有条件中固定33.4Hz，不随融合组变化。此前表格里的待实测项，以本次结果补齐。

## 2. 实验设计与控制条件

| 条件 | 本轮设置 |
| --- | --- |
| 实际后端 | 官方nvblox::NvbloxNode组件；每轮重新创建Mapper和地图 |
| 输入 | Gazebo D435深度16UC1/RGB rgb8，各640×480，目标30Hz |
| 标定与位姿 | 两路各自CameraInfo，图像时刻机械臂光学TF |
| 相机运动 | 固定在已验证约0.82m高位，本轮没有左右/前后晃动 |
| 体素/最大融合距离 | 0.03m / 3m |
| 地图类型 | static_tsdf；原生3D ESDF，RGB、Mesh和地图层保持开启 |
| 被比较变量 | 深度/颜色融合门限同时设30、33.4或40Hz |
| 固定ESDF更新门限 | 33.4Hz，tick约10ms |
| Mesh/地图层调度目标 | 5Hz，保持相同Mesh订阅与RViz启动配置 |
| 项目三维点云导出 | 本轮不启用；官方组件不提供esdf_points/metrics |
| 重复次数 | 每档3轮，共9轮 |
| 每轮过程 | 等待Mesh及RViz就绪、额外静止5秒、正式采样约20秒、保存Mesh后退出Mapper |
| 顺序 | 30→33.4→40→40→30→33.4→33.4→40→30 |
| 隔离环境 | ROS_DOMAIN_ID=129，Gazebo端口11459；同一个持续运行场景 |

先将机械臂移动到此前通过URDF/FCL检查的高位中心，之后九轮不发送扫描指令。这样把运动、自体位移和不同视角的影响从门限对照中尽量排除。

交错顺序用于减小运行先后影响，但不是随机化，也不是同一bag回放。图像时间戳抖动、系统负载和用户窗口交互仍可能不同；结论只针对本场景短时测试，不保证所有设备或长时间大地图都达到同样频率。

## 3. 问题代码与为什么出现18Hz

本地官方NvbloxNode::shouldProcess判断：

```cpp
rclcpp::Duration time_since_last = time_now - time_last;
return time_since_last.seconds() > (1.0 / desired_frequency_hz);
```

processDepthImage/processColorImage各自用**本张图像时间戳减上一次融合图像时间戳**调用它；不满足时直接返回true，表示消息已处理但没有真正积分。因此topic输入稳定不等于每张图参与融合。

| 融合门限 | 要求的严格最小间隔 | 对正常33ms输入的影响 |
| --- | ---: | --- |
| 30Hz | >33.333ms | 33ms不通过，可能到下一帧约66ms才融合 |
| 33.4Hz | >29.940ms | 正常33ms可通过，但短于/等于约29.94ms的抖动间隔仍可能跳过 |
| 40Hz | >25ms | 给约30Hz输入留更宽的时间抖动余量 |

出现18Hz而非固定15Hz，是因为真实相邻帧间隔并非全部相同，有的连续通过，有的隔帧通过。该限频问题与GPU能否计算30Hz不是同一件事。提高门限并没有生成新传感器图像，只是减少主动跳过正常输入。

源码：[融合门限判断](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:564)、[深度过滤](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:982)、[颜色过滤](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:1193)。

## 4. 九轮逐轮实测

频率为实际计数或消息接收结果，不是配置值。

| 顺序 | 融合门限 | 重复轮 | 深度输入Hz | RGB输入Hz | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | Mesh Hz |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 30Hz | 1 | 29.92 | 29.87 | 18.74 | 18.49 | 32.98 | 4.75 |
| 2 | 33.4Hz | 1 | 29.92 | 29.97 | 29.64 | 29.09 | 32.75 | 4.75 |
| 3 | 40Hz | 1 | 29.86 | 29.96 | 29.87 | 29.87 | 32.94 | 4.75 |
| 4 | 40Hz | 2 | 29.86 | 29.62 | 29.62 | 29.82 | 32.73 | 4.74 |
| 5 | 30Hz | 2 | 29.96 | 29.96 | 18.44 | 18.59 | 33.03 | 4.75 |
| 6 | 33.4Hz | 2 | 29.92 | 29.97 | 29.72 | 29.72 | 32.93 | 4.75 |
| 7 | 33.4Hz | 3 | 29.92 | 29.92 | 29.37 | 29.03 | 33.08 | 4.76 |
| 8 | 40Hz | 3 | 29.97 | 29.97 | 29.84 | 29.89 | 32.96 | 4.74 |
| 9 | 30Hz | 3 | 29.92 | 29.97 | 18.36 | 18.46 | 32.86 | 4.74 |

### 各档三轮汇总

“均值±标准差”使用三次测量的样本标准差，不是置信区间；括号为最小～最大。短时三次重复不足以证明长期最坏情况。

| 融合门限 | 深度融合Hz：均值±标准差（范围） | 颜色融合Hz：均值±标准差（范围） | ESDF调用Hz均值 | Mesh Hz均值 |
| --- | --- | --- | ---: | ---: |
| 30Hz | 18.52±0.20（18.36～18.74） | 18.52±0.07（18.46～18.59） | 32.96 | 4.75 |
| 33.4Hz | 29.58±0.18（29.37～29.72） | 29.28±0.38（29.03～29.72） | 32.92 | 4.75 |
| 40Hz | 29.78±0.14（29.62～29.87） | 29.86±0.04（29.82～29.89） | 32.88 | 4.74 |

### 从回调到实际积分的累计计数（三轮相加）

| 融合门限 | 深度回调→积分 | 深度积分/回调比 | 颜色回调→积分 | 颜色积分/回调比 |
| --- | --- | ---: | --- | ---: |
| 30Hz | 1813→1123 | 61.94% | 1817→1123 | 61.81% |
| 33.4Hz | 1816→1794 | 98.79% | 1815→1776 | 97.85% |
| 40Hz | 1808→1806 | 99.89% | 1812→1811 | 99.94% |

计数来自原有ros/depth_image_callback、ros/color_image_callback与ros/depth/integrate、ros/color/integrate差分；快照可能截在处理链不同位置，约1帧差异不等于严格漏帧或零丢帧证明。

### 输入时间戳与门限关系

下表对各轮收到的图像时间戳做离线门限解释，区间模型独立于Mapper自身状态，不能代替上面的真实融合计数。

| 融合门限 | 深度相邻正间隔中位数（三轮） | 深度相邻间隔≤门限比例（三轮） | 颜色相邻间隔≤门限比例（三轮） |
| --- | --- | --- | --- |
| 30Hz | 33.0ms/33.0ms/33.0ms | 55.6%/56.4%/59.9% | 55.7%/56.9%/59.1% |
| 33.4Hz | 33.0ms/33.0ms/33.0ms | 1.0%/0.8%/1.5% | 2.6%/0.7%/2.8% |
| 40Hz | 33.0ms/33.0ms/33.0ms | 0.2%/0.0%/0.0% | 0.2%/0.0%/0.0% |

这里按相邻输入间隔统计，实际门限是距“上次成功融合”的间隔；被跳过后会累积到下一帧，所以不能简单把这个比例直接当实际丢帧比例。分析同时保存了仅模拟门限的预测准入率，用于判断趋势。

### 本轮结论

1. **约18Hz不是图像输入只有18Hz。** 九轮输入均接近30Hz，30Hz融合门限组三轮深度均值18.52Hz、颜色18.52Hz，重复出现低融合率。
2. **融合门限33.4Hz已实际测试。** 三轮深度均值29.58Hz、颜色29.28Hz；该数值是融合参数实验结果，不是借用ESDF33.4的结果填表。
3. **当前40Hz修正得到重复支持。** 深度均值29.78Hz、颜色29.86Hz。余量比33.4更宽，本轮保留默认40，不再改成33.4。实际吞吐依然受30Hz传感器限制。
4. 结合源码主动过滤路径和改变门限后的计数提升，证据支持时间门限为本场景低融合率的主要原因；这不是对全部GPU开销的完整剖析，不据此断言所有环境都没有计算瓶颈。
5. 本轮ESDF门限始终33.4Hz，入口调用次数不是融合成功数，也不保证每次有新观测块。不能把它作为30Hz新地图证据。

![九轮真实数据汇总图](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105/fusion_repeats_summary.png)

图中柱为三轮均值、误差线为样本标准差、黑点为各轮值；灰线为30Hz相机目标。ESDF是调用次数，不能与“不同输入产生的地图数”画等号。


## 4.1 补充：融合后的3D ESDF频率与实时性

此节补查同一九轮测试已有官方日志，没有重新运行仿真。ESDF模式为3d，所有组ESDF门限均为33.4Hz。

| 深度/颜色融合门限 | 3D ESDF调用均值 | 约每次调用间隔 | ESDF输入年龄滚动均值范围 | 这些窗口均值的中位数 |
| --- | ---: | ---: | ---: | ---: |
| 30Hz | 32.96Hz | 30.34ms | 40～50ms | 45ms |
| 33.4Hz | 32.92Hz | 30.37ms | 33～37ms | 35ms |
| 40Hz | 32.88Hz | 30.42ms | 31～41ms | 35ms |

实时性提升主要体现为：融合恢复接近30Hz后，ESDF使用的最新深度更及时，日志窗口均值的中位数从45ms降到35ms；ESDF调用频率则基本不变，因为本轮固定同一个ESDF门限。

年龄来自官方ros/esdf_integration：updateEsdf调用后取节点当前时间，减最新融合深度时间戳。只汇总每个日志窗口样本数为100的滚动均值，每组24个记录窗口；这些窗口相互重叠，且取该轮全部可用稳态日志，**不是正式measure区间逐帧延迟，也不是P50/P95或GPU完成耗时**。约30.4ms是调用次数倒数，不是每次ESDF计算耗时。不能把45→35ms理解为屏幕端到端延迟降低10ms。

**本轮没有启用/nvblox_node/esdf_points，因此没有本轮3D点云发布Hz或显示延迟实测。** 上表证明周期调用与输入新鲜度，不能证明每次生成了不同的新地图，也不能证明所有距离点已在GPU完成并显示。

此前高位左右扫描的diagnostic后端中，融合40/ESDF33.4/显示33.4时，三维点云发布29.22Hz、关联输入年龄P50/P95约51/79ms。这是另一轮、带额外导出负载的历史结果，仅供参照；不能填入本轮融合33.4Hz组，或与上表的滚动均值直接比较。

原始补充统计：[esdf_realtime_analysis.json](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105/esdf_realtime_analysis.json)。当前尚缺同轮诊断点云的新输入版本推进、GPU完成与屏幕端到端关联测量，不能宣称“完整3D ESDF实时性已经全部验证”。

## 5. 修改了什么，哪些没有修改

| 位置 | 已有修正 / 本轮操作 | 目的 |
| --- | --- | --- |
| config/profiles/realtime.yaml | 此前已将integrate_depth_rate_hz和integrate_color_rate_hz从30改40；当前保持40 | 为30Hz图像留25ms最小间隔余量 |
| 同一profile的update_esdf_rate_hz | 此前已设33.4；本轮九组固定同值 | ESDF节拍与融合门限分离，避免混淆 |
| launch/perception/nvblox.launch.py | 复用现有depth_hz/color_hz CLI→官方参数映射 | 各轮显式覆盖30/33.4/40，运行后dump断言实际值 |
| tests/test_launch_contract.py | 此前已有默认融合40、ESDF33.4契约；本轮未改 | 保持当前默认行为，不把实验档永久写回 |
| artifacts/fusion_gate_repeats_20261105 | 新建九轮采集、统计分析、参数快照、截图和报告证据 | 重复验证18Hz原因，并首次补齐融合33.4Hz实测 |
| 官方workspaces C++/CUDA主体 | 未修改 | 不是重写积分核函数，也不是修改图像时间戳伪造吞吐 |

当前默认仍为融合40Hz、ESDF33.4Hz；并没有把深度/颜色默认改成33.4。本轮没有新增调度算法、逐帧版本跟踪或GPU优化，没有安装新依赖，也没有因改参数而重新编译官方核心。

## 6. 与之前“33.4Hz”测试的区别

| 项目 | 之前ESDF节拍优化 | 本轮融合门限复测 |
| --- | --- | --- |
| 深度/颜色门限 | 固定40Hz | 对比30、33.4、40Hz |
| ESDF门限 | 对比30与33.4Hz | 全部固定33.4Hz |
| 观察对象 | ESDF调用和三维显示节拍 | 输入帧是否进入深度/颜色融合 |
| 诊断点云 | 另测显示门限30/33.4/40 | 不启用项目三维点云，官方主体一致 |
| 33.4Hz的含义 | ESDF更新或显示参数 | 本轮中间组确实是深度/颜色融合参数 |

因此本轮30Hz融合组即使仍只有约18Hz融合，也可能有约33Hz的ESDF调用，因为ESDF门限在本轮已固定33.4；这不与历史“融合18、ESDF24.4”矛盾。调用次数不是新输入更新次数，不能据此判断每次有新地图。

本轮没有测/nvblox_node/esdf_points发布率，因为默认官方组件不提供该项目导出。要查看诊断点云用run_esdf.sh，但不要把另一后端的发布频率混入这张同条件官方融合对照表。

## 7. 测量口径与限制

- 核心率：原有save_timings累计样本数差/两次服务请求响应墙钟中点之差；同时保存最短/最长可能区间，服务实际快照时刻仍有小误差。
- 输入与Mesh率：消息接收时间；输入统计窗口为正式measure阶段，与服务快照区间有少量边界差。
- 计数表是进入积分函数，不是CUDA事件测得的GPU完成帧率，也不保证所有像素有效或每次改变地图。
- 静止相机排除了本轮运动轨迹影响，但不是实机D435验证，也不是长期大地图压力测试。
- 截图仅为真实窗口证据，不用于判断30Hz或量化几何精度。配色、边界模糊和自体污染没有在本轮修复。
- 没有通过删点、降低体素精度、减少范围、降低输入尺寸或提高相机帧率获取本轮融合提升。
- 本轮脚本语法与运行时组件/参数/输入检查通过；未修改感知生产源码，因此未重新构建或重复之前的离线单元/GPU测试。

## 8. 实际窗口截图

以下为每档首轮正式采样阶段的实际官方Mesh窗口，输入仍在左上显示。它们不是ESDF点云截图；不同窗口交互/视角不做像素级质量对比。

### 融合门限30Hz

![30Hz融合组实际窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105/gate30p0_r1_measure.png)

### 融合门限33.4Hz

![33.4Hz融合组实际窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105/gate33p4_r1_measure.png)

### 融合门限40Hz

![40Hz融合组实际窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105/gate40p0_r1_measure.png)

## 9. 手动复现指令

先停止同域同端口的旧场景/Mapper。以下每条是一轮独立启动，不能同时执行。它们的ESDF固定33.4Hz，区别仅融合门限。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
# 第一组：复现30Hz融合门限
./run.sh gazebo_gui:=true depth_hz:=30 color_hz:=30 esdf_hz:=33.4
```

停止上一组后，分别运行：

```bash
# 第二组：本轮新验证的33.4Hz融合门限
./run.sh gazebo_gui:=true depth_hz:=33.4 color_hz:=33.4 esdf_hz:=33.4
```

```bash
# 第三组：当前默认修正
./run.sh gazebo_gui:=true depth_hz:=40 color_hz:=40 esdf_hz:=33.4
```

新监测终端加载相同环境：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
ros2 param get /nvblox_node integrate_depth_rate_hz
ros2 param get /nvblox_node integrate_color_rate_hz
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 topic hz /camera/depth/image_rect_raw --wall-time
# Ctrl+C后
ros2 topic hz /camera/color/image_raw --wall-time
```

topic hz只测输入，不能证明融合。静止预热后保存官方统计，在明确墙钟采样区间前后分别保存timings，再取ros/depth/integrate和ros/color/integrate样本数差；完整步骤见[逐终端测试指南](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_COMMAND_TEST_GUIDE_20261105.md)。

### 自动复现本轮九次固定高位测试

停止手动场景后执行；脚本会自己启动隔离Gazebo和Mapper。先备份本次artifact目录，脚本会覆盖同名数据。DISPLAY按当前桌面设置，本次为:1。脚本包含进入高位的机械臂命令，只用于本项目仿真，不用于实机。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/fusion_gate_repeats_20261105/run_measurement.py
/usr/bin/python3 artifacts/fusion_gate_repeats_20261105/analyze_repeats.py
```

静态对照是本轮的控制条件；不需要额外执行任何左右/前后摆动脚本。自动脚本已包含经过检查的高位计划与初始状态保护。

## 10. 原始证据

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_gate_repeats_20261105`。

| 文件 | 内容 |
| --- | --- |
| results.json | 真实启动顺序、阶段、组件/输入检查、每轮参数及服务快照墙钟区间 |
| samples.json | 全部输入图像时间戳与消息接收记录 |
| analysis.json | 九轮频率、三轮均值/波动、计数和输入门限解释模型 |
| *_runtime_parameters.yaml | 各轮实际生效配置，避免只看源码默认值 |
| *_static_timings.txt、*_measure_timings.txt | 核心融合计数的原始依据 |
| *_rates.txt、mapping_*.log | 原有官方滚动率、队列/延迟日志 |
| *_measure.png、*_rgb.png | 真实RViz与RGB证据 |
| mesh_*.ply | 各轮最终网格，不作为本轮几何精度验收 |
| motion_plan.json、camera_trajectory.json | 高位进入计划和实际静止TF记录 |
| source_manifest.json | 脚本、当前配置和相关官方源码哈希 |

原始历史说明：[融合限频第一次对照](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/FUSION_RATE_MARGIN_OPTIMIZATION_20261105.md)、[ESDF33.4与显示门限](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)。历史值与本轮不同，保留各自当时条件，不能直接互换。
