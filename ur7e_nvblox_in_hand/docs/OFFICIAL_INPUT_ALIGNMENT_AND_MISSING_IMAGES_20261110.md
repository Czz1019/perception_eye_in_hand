# 官方输入同步、空间对齐与图像缺帧分析

日期：2026-11-10。对象：本机 workspaces 中 Isaac ROS Nvblox 普通静态建图路径，以及 perception_eye_in_hand 中 ur7e_nvblox_in_hand。本文只分析，不修改代码、不启动仿真或硬件。

## 1. 结论

“对齐”至少包含三种不同操作：同一相机的 Image 与 CameraInfo 时间配对、深度与彩色的空间投影对应、图像与相机位姿的时间对应。它们不能互相替代，也不能修复已丢失的图像。

本机 workspaces 的 NvbloxNode 在不启用分割时，分别精确同步深度图与深度内参、彩色图与彩色内参；两路图像各自排队、限频，按各自 frame_id 和时间戳获取相机位姿，再分别融合进共享地图。没有把深度图和彩色图放在同一个 RGB-D 精确同步器里。

当前 Python 包采用最近 CameraInfo 缓存，且按图像自己的时间戳查 TF。因此其底层融合接口与上面有共同思路，但 ROS 输入配对方式并不完全等价。不能说它完整复现了官方 ROS 前端。

## 2. 官方资料与本地版本的关系

官方接口文档明确要求深度图配套 CameraInfo，彩色图可选且需要配套 CameraInfo；TF 模式下不必额外提供 pose/transform 话题。[NVIDIA 官方接口文档](https://nvidia-isaac-ros.github.io/v/release-4.0/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)

网页说明用于核对接口含义，具体同步、排队行为以本机源码为准。本机 workspaces 已有修改，不应视为未经修改的上游发行版；也不能把 release-4.0 文档当成本机所有依赖的版本证明。

本地关键文件：

- [NvbloxNode 实现](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:240)
- [同步策略定义](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/include/nvblox_ros/nvblox_node.hpp:348)
- [位姿查询实现](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/transformer.cpp:42)

## 3. Image 与 CameraInfo：时间配对

### 3.1 workspaces 中的实现

非分割路径使用 message_filters::sync_policies::ExactTime：

```text
Depth Image(tD) + Depth CameraInfo(tD)
             → depthImageCallback → 深度输入队列

Color Image(tC) + Color CameraInfo(tC)
             → colorImageCallback → 彩色输入队列
```

ExactTime 比较消息 header 中的时间戳；两条消息不必同时到达，但需要在同步器保存期间找到相同时间戳。ROS message_filters 官方说明区分 ExactTime 与 ApproximateTime，前者要求时间相同，后者按时间接近程度进行配对。[官方 message_filters 文档源码](https://github.com/ros2/message_filters/blob/rolling/doc/index.rst)

这里 tD 不必等于 tC。分割路径另有多路同步策略，不能把这些策略概括为普通建图对所有输入使用近似同步。

如果只有 CameraInfo(t)，没有 Image(t)，同步器不会凭内参生成图像。图像已丢失时，改为 ApproximateTime 也不能恢复像素。

### 3.2 当前仿真插件如何生成时间戳

插件的 cameraInfo(image, ...) 直接执行：

```cpp
info_msg.header = image.header;
```

深度回调先生成深度图，再用它构造 CameraInfo，调用 CameraPublisher 发布。彩色路径同样由图像 header 构造 CameraInfo。因此正常执行时，同一帧两条消息的时间戳相同。

但 CameraPublisher 对外仍是独立图像与内参话题；提交同一对消息不等于接收端必然同时、完整收到。具体图像传输还受订阅、transport 和 QoS 影响。

[仿真插件源码](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp:234)

### 3.3 当前 Python 前端的区别

camera_info() 保存每一路最近收到的有效 CameraInfo。candidate() 使用该缓存，检查图像与 CameraInfo 的 width、height、frame_id 一致，但没有要求 header.stamp 相同，也没有逐帧比较所有内参内容。

固定内参、固定分辨率、同一光学坐标系时，复用内参有合理性；但分辨率切换、裁剪、重标定或缓存更新滞后时，存在使用旧内参的风险。这种缓存策略需要明确的标定生命周期管理，不能仅凭宽高一致就证明内参正确。

本轮缺失的 CameraInfo/图像配对数量，不会在该 Python 前端按 ExactTime 规则直接变成相同数量的同步拒绝；它可能复用已缓存内参。真正缺失的图像仍然无法融合。

[Python 前端](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:138)

## 4. RGB 与 Depth：时间同步和空间对齐不同

RealSense 官方 README 对两个参数的定义不同：enable_sync 让 librealsense 将相关传感器帧组成同步 frameset；align_depth.enable 将深度对齐到 RGB 图像。前者不等于空间投影，后者也不保证 ROS 消息无损送达。[RealSense 官方说明](https://github.com/realsenseai/realsense-ros#rgbd-topic)

### 4.1 预先把深度对齐到彩色

空间对齐的几何思路是：深度像素通过深度内参反投影到三维，再通过深度到彩色的标定外参转换，最后用彩色内参投影到彩色图像平面。

```text
深度像素(uD,vD,zD)
 → 深度光学系三维点
 → 深度至彩色外参
 → 彩色光学系三维点
 → 彩色像素(uC,vC)
```

这是针孔几何的解释，并非对 librealsense 每一步实现细节的逐行复述；实际驱动还处理像素覆盖、遮挡和无效值。

开启该路径后，要使用 aligned_depth_to_color 的图像及其对应 CameraInfo，并核对实际 header.frame_id。不能只把原始深度图的 frame_id 改成 color，也不能把原始深度图直接配上彩色内参。

双相机视点不同，重投影会存在遮挡和无覆盖区域，空间对齐不能保证填满所有空洞。

### 4.2 Nvblox 独立融合两路观测

本机 workspaces 普通建图路径不要求先把深度变成彩色像素网格。processDepthImage() 使用深度自己的内参及位姿，processColorImage() 使用彩色自己的内参及位姿；深度更新几何，彩色基于已有几何更新颜色。

这允许两路图像独立到达、独立限频。成立条件是两路相机内参、光学坐标系、外参和时间对应都正确，且场景与位姿误差满足融合要求；独立融合并不消除运动或动态场景造成的误差。

本机 RealSense 启动映射也保留原始 depth/image_rect_raw 和 color/image_raw，部分配置通过 splitter 选择深度，不应把 splitter 等同于 RGB-D 空间对齐。

[本地启动映射](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py:82)

## 5. 图像与机械臂位姿如何对应

workspaces 在融合深度时，用深度图 header.frame_id 和深度图 header.stamp 查询 global_frame 到该光学系的变换；彩色使用自己的 frame_id 与 stamp。TF 模式不是用处理时刻的最新位姿代替图像时刻的位姿。

记 T_A_B 为把 B 系坐标变换到 A 系，则机械臂手眼场景可表示为：

```text
T_base_depth(t) = T_base_tool0(t) × T_tool0_color × T_color_depth
T_base_color(t) = T_base_tool0(t) × T_tool0_color
```

公式假定手眼结果确实是 tool0 到 color optical 的变换。实际 TF 树可经 camera_link 等中间节点组合；每条边的方向与标定语义必须一致。不能把原先对 color 标定的数值直接作为 tool0→camera_link 发布，除非已做坐标变换。

原始深度使用 depth optical frame 是正常的；手眼标定在 color optical frame 也正常，两者通过相机内部外参连接。depth 与 color 应保留各自语义，不能为了名字统一而强行合并。

TF 查询可在有足够历史样本时插值。机械臂 TF 不必和图像同频，但需要覆盖图像时刻；缺少未来样本、时间基准不一致或延迟过大时，可能等待或丢弃观测。

另一个版本细节：本机 transformer.cpp 的 pose/transform 话题队列路径使用 lower_bound 查询并检查时间容差，代码带有“尚未实现插值”的 TODO。它不是严格搜索前后两侧最近点，不能把它与 TF 插值路径混为一谈。

Python 包也按图像时间戳查询 TF；查询失败会留在有限队列中重试，超过有效年龄后丢弃，不直接替换成最新 TF。

## 6. 本轮为什么出现有内参、无图像

已有录制的共同时间范围中：

| 核对项目 | 深度 | 彩色 |
|---|---:|---:|
| 有 CameraInfo 时间戳、没有相同时间戳图像 | 315 | 503 |
| 有图像时间戳、没有相同时间戳 CameraInfo | 1 | 0 |

这些是时间戳集合匹配数量，不是 DDS 网络包丢失计数。多数已录到的图像都能匹配内参，更支持图像链路缺帧，而非普遍时间戳错配。

插件使用 SensorData QoS。ROS 官方对传感器数据策略采用 Best Effort 和较小队列，以及时性为优先；它不承诺每条消息送达。Reliable 订阅也不能将 Best Effort 发布端变成可靠传输，且不兼容的可靠性组合可能无法通信。[ROS Humble QoS 官方文档源码](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

需排查的环节：插件图像发布/transport、DDS 分片及缓冲、订阅器接收调度、rosbag 缓存与写入。现有数据没有记录各环节完整计数，不能指定某一个为已证实根因。插件彩色与红外共用 image_msg_ 是另一个需核实回调并发性的风险，尚无证据证明本轮发生了竞争，且不能直接解释深度缺帧。

仿真时间与墙钟比例约 0.999，排除了整体仿真持续明显降速这一主要解释；不排除传感器渲染线程局部延迟。录制阶段没有运行 mapper，因此输入 bag 中的缺帧不能归因于此次 ESDF 计算。

原始依据：[输入时间戳分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/stride23_replay_20261110/input_timing_diagnosis.json)。

## 7. 后续验证与改进方向

1. 在传感器回调、插件发布、独立接收器、rosbag 保存四处按同一 header.stamp 计数；记录 frame_id、内参版本和消息字节数，避免仅凭 topic hz 定位。
2. 分别测试仅深度、深度加彩色、加入录包；同时测仿真/墙钟比例与传感器回调间隔。
3. 对普通官方路径验证每一路 Image/CameraInfo 的精确匹配比例，不额外强制 RGB 与深度逐帧 ExactTime。
4. 对 Python 包明确选择精确配对，或受控的固定内参缓存。后者需处理分辨率、ROI、内参变更和缓存失效。
5. 保留图像自己的光学系与时间戳，检查 TF 历史覆盖；不要把最新位姿、改写 frame_id 或改写时间戳作为补帧措施。
6. 依据测量调整兼容 QoS、队列和接收缓冲；观察丢帧与延迟的权衡，不能承诺改为 Reliable 就获得完整 30 Hz。
7. 若选择 aligned depth，整套切换图像、CameraInfo 与光学系；它解决空间对应，不能解决录制丢帧。

## 8. 对当前问题的实际意义

官方同步器负责让已收到的正确数据对应起来，不能凭 CameraInfo 恢复丢失图像。当前需要先确定图像缺在哪一段，再验证位姿与相机模型；时间同步正确也不自动证明 Mesh 精度合格。本文没有新增测试结果，也没有更改运行参数或任何源代码。
