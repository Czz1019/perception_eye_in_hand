# 实时性测试对应代码与后续优化路线

日期：2026-11-03。本文解释 artifacts/realtime_test_20261103 的实测结果，提出下一阶段分析方案，不表示以下优化已实现。

## 1. 本次测的是哪条路径

```text
nvblox_in_hand/run.sh
 → launch/bringup.launch.py
 → launch/perception/mapping.launch.py，mapper_backend=observed_view
 → install/nvblox_in_hand/lib/nvblox_in_hand/mapper_node
 → InHandMapper : 官方 NvbloxNode
 → 官方 Mapper：深度 TSDF 融合、变化块 ESDF 更新
 → 可选 serialize 全部分配块、CPU 筛选、Marker 发布
```

项目源码位于 `/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand`。默认构建复用 `/home/czz/workspaces/isaac_ros-dev/install` 官方库；测试安装目标依赖 libnvblox_ros_lib.so 和 libnvblox_lib.so。upstream 是对应代码阅读/可选源码构建快照，不能把默认测试称为已经重新编译了整套 upstream 核心。

没有运行父工程 ur7e_nvblox 的 Python TfMapper、固定 ROI 距离查询或旧 /ur7e_nvblox/esdf_3d 发布者。场景来自新工作区的 ur7e_d435_gazebo 副本。

测试先预热，再运行无 ESDF 显示订阅 25 秒、有 ESDF Marker 订阅 20 秒。机械臂保持初始姿态，无 RViz 和 Mesh 订阅。Marker 订阅用于触发真实显示导出，不是伪造地图数据。

## 2. 为什么能接近 5 Hz

mapping.launch.py 设置 update_esdf_rate_hz=5。官方 NvbloxNode::tick 按 ROS 时间调度 processEsdf，随后调用 multi_mapper_->updateEsdf。核心 Mapper::updateEsdf 取变化块并调用 EsdfIntegrator，而不是每次重新查询固定大 ROI 的全部点。

InHandMapper::processEsdf 先调用基类，再增加周期计数；测量脚本用两个累计计数差除以墙钟时间，得到约 5.09 / 4.76 Hz。这个指标表明更新方法被调用的节奏，不证明每周期都有新变化块，也不等于 GPU 完成或用户看到新画面的频率。

## 3. 本次结果支持哪些判断

| 结果 | 合理解释 | 还不能断言 |
| --- | --- | --- |
| 无显示约 5.09 Hz | 静止场景中 ESDF 调度接近目标 | 运动和大地图也能稳定达到 |
| 订阅后约 4.76 Hz | 导出负载下周期略有下降 | 全部下降都来自 ESDF 算法本体 |
| 深度 16.69 → 11.83 Hz | 与导出开启存在明显负载相关性 | 一次顺序实验已严格量化唯一因果贡献 |
| 附加显示样本均值 106.40 ms | 同步导出路径开销明显 | 106 ms 全是 PCIe 拷贝，或全是 CPU 计算 |
| 原生 ESDF 主机调用约 1 ms | 方法返回的主机耗时较短 | GPU 任务完整执行仅需约 1 ms |
| Marker 0.79 Hz | 实际接收低于配置 1 Hz 上限 | ESDF 仅计算了 0.79 Hz |
| 仿真实时率约 1 | 本次未出现明显仿真降速 | 开 GUI/RViz 后仍不竞争资源 |

106 ms 的计时从 serializer 调用前开始，到发布返回后结束，包含 GPU 流同步等待、序列化与可访问数据准备、CPU 逐体素筛选、消息构造和发布。serializer 内部明确调用 cuda_stream.synchronize；此前队列中尚未完成的 GPU 工作也可能在这里被等待，不能把这段都归入纯显示拷贝时间。

## 4. 第一步：把时间和工作量测清楚

增加诊断时，优先保留原参数和算法，避免边测边改变输入负载。

| 环节 | 应记录内容 | 目的 |
| --- | --- | --- |
| 图像输入 | 帧号、原始时间戳、分辨率、接收墙钟时刻、有效深度比例 | 确认真正输入速率与质量 |
| 限频与队列 | 因限频跳过、TF 缺失、过期、队列溢出的数量及帧龄 | 区分主动跳帧与计算过慢 |
| 深度融合 | 主机提交耗时、GPU 事件耗时、触及/分配块数 | 定位实际融合工作量 |
| ESDF | 每周期变化块数、更新状态、GPU 事件耗时 | 区分空调用、局部更新和大范围传播 |
| 导出 | 块枚举、序列化/同步、CPU 筛选、消息构造/发布分段耗时 | 拆解当前约 106 ms |
| 数据规模 | TSDF/ESDF 总块数、序列化体素数/字节、输出点数/字节 | 判断随地图增长的复杂度 |
| 消费者 | 消息接收时刻、实际地图版本、屏幕显示时刻 | 区分发布频率和最终新鲜度 |

GPU 事件应在所属 CUDA 流记录，延后读取已完成事件，避免每帧新增全局同步而改变被测性能。CPU 时长、GPU 时长和排队等待可能重叠，不能简单相加。ROS 时间用于图像/位姿配对，单调墙钟用于实际耗时。

为地图状态提供关联的版本或输入图像时间，避免只用 now() 给 Marker 打戳却误认为地图就是当前时刻的数据。频率稳定也可能在处理积压旧帧。

## 5. 第二步：调查深度为什么只有约 16.7 Hz

本地官方快照 NvbloxNode::shouldProcess 使用严格 `时间间隔 > 1/目标频率`。processDepthImage 对不满足条件的图像返回已处理但不融合。

源图若恰为 30 Hz、目标也为 30 Hz，时间戳间隔等于或略小于阈值就可能被跳过。理想等间隔时可能出现每两帧接受一次的行为；实际值取决于时间戳抖动、源频率和精度。因此先统计原始间隔与限频跳过比例，不能把 16.7 Hz 直接当作 GPU 吞吐上限。

可短暂以高 depth_hz 解除限频作诊断，或对严格比较、容差/截止时间方案做对照。这样的实验改变了调度条件，结果必须独立标注，不能直接冒充原 30 Hz 配置下的优化成绩。最终修复应使用清晰的速率控制语义，防止无意接收超额负载。

## 6. 第三步：优先优化导出路径

当前代码调用 getAllBlockIndices，再同步 serialize，之后才筛选 observed 和 12 cm 距离带。图上点少并不能减少前面的全层读取。

建议按复杂度逐步尝试：

1. 保持完整性语义，拆分导出耗时，确认主要成本是同步、数据准备还是 CPU 消息构造。
2. 在 GPU 上筛选并紧凑输出有效距离带，减少传回和 CPU 遍历量；先证明字节量下降且位置/符号/observed 一致。
3. 独立跟踪显示变化块，增量导出并保留块删除、地图重置和新订阅者全量补发语义。不要直接消耗核心 ESDF 的更新追踪器，导致核心漏更新。
4. 使用有界异步快照/双缓冲将显示与融合解耦，只保留最新待显示版本；明确 GPU 流事件依赖和地图块生命周期，避免后台读取正在修改或释放的内存。
5. 根据需要增加显示 ROI、点数预算和频率上限。ROI/抽样只影响展示，不能缩减规划所需地图后仍声称功能等同。

单纯把 serializer 放进另一线程不保证性能提升：共享流同步、地图锁和主机内存访问仍可能阻塞融合，并引入数据竞争。先设计一致快照和资源边界。

## 7. 第四步：同条件验证 workspaces 与项目

固定深度分辨率、2 cm 体素、3 m 范围、相机模型/噪声、TSDF 截断与权重、TF 和图像序列，优先用同一录制输入回放。确保回放时 TF static 保留、仿真时钟唯一，排除现场轨迹差异。

分别测官方节点、项目适配关闭显示、开启显示、开启 Mesh/RViz 四种组合。相同消费者才可比较纯节点开销；没有 RViz 时官方可能省去按需 Mesh 工作，应明确这一差异。

静止、已有腕部转动、地图逐渐增长三个场景分别重复至少三次；随机或交替运行顺序，从空地图开始。每项保存均值与 p50/p95/p99，并记录实际观察时长，短测试不宣称长期尾延迟结论。

## 8. 性能之外还要验收质量

墙角 unknown：统计有限 ROI 内真实 observed 比例，区分显示带外；针对剩余可见未知增加观察视角，而非填自由。

运动残影：验证传感器测量时间与 TF，一帧一帧比较仿真真值；检查手眼、光学坐标系和机器人自身入图。静态环境的相机运动重影先修输入一致性，动态障碍残留再评估局部清除/动态模式。

优化应同时报告表面误差与厚度、已观测覆盖率、伪障碍、残影清除时间、数据帧龄和更新频率。避免用更多跳帧、缩小地图、降低分辨率取得更高 Hz，却丢失原要求的完整性。

## 9. 代码导航和证据

- [启动与后端选择](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/mapping.launch.py:50)
- [C++ 薄适配与计数](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:26)
- [全层导出与显示计时](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:52)
- [官方调度、shouldProcess 与深度处理快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/nvblox_node.cpp:564)
- [核心 ESDF 变化块更新](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/nvblox_core/nvblox/src/mapper/mapper.cpp:495)
- [序列化同步位置](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/nvblox_core/nvblox/include/nvblox/serialization/internal/impl/layer_serializer_gpu_impl.h:23)
- [测试订阅和测量流程](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/measurement_script.py:73)
- [本次实测结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/RESULTS.md)
