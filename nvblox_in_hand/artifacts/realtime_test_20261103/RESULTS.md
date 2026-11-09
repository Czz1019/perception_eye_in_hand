# ESDF 简单实时性实测

日期：2026-11-03。本次为实际 Gazebo + 安装版本 mapper 测试，不是配置频率推算。

## 条件

- RTX 4060 Laptop GPU；2 cm 体素；最大融合距离 3 m；3d/static_tsdf。
- 使用现有 D435 仿真场景，深度配置 1280×720、30 Hz；没有额外订阅原始图像来核对传输频率。
- 深度调度上限 30 Hz，颜色 5 Hz，ESDF 5 Hz。
- 机械臂保持初始姿态，没有发送运动命令；未执行环绕或腕部转动。
- 无 Gazebo GUI、无 RViz、无 Mesh 消费者。第二阶段仅用 ROS Marker 订阅者触发三维 ESDF 距离带导出，不包含屏幕绘制。
- 同一进程预热后，先记录 25 秒基础阶段，再记录 20 秒订阅阶段。这是简单顺序对照，未随机化或重复三轮。

## 测量结果

| 指标 | 不订阅 ESDF 显示（25 s） | 订阅三维 ESDF 显示（20 s） |
| --- | ---: | ---: |
| ESDF 调用周期，墙钟 Hz | 5.09 | 4.76 |
| 深度实际融合，墙钟 Hz | 16.69 | 11.83 |
| 颜色实际融合，墙钟 Hz | 4.64 | 3.98 |
| ESDF 主机调用耗时采样均值 | 1.23 ms | 0.96 ms |
| ESDF 附加显示耗时采样均值 | 未执行 | 106.40 ms |
| ESDF 附加显示耗时采样范围 | 未执行 | 103.28–113.86 ms |
| 三维 ESDF Marker 接收频率 | 未订阅 | 0.79 Hz |
| 每条显示消息体素点数 | 不适用 | 34,696 |
| 仿真时间/墙钟时间 | 1.001 | 0.999 |

频率由接收到的 metrics 累计计数差除以墙钟间隔计算；Marker 频率由实际消息到达间隔计算。调用耗时是每条 metrics 所携带的最近一次耗时样本，其均值不等于全部 ESDF 调用的逐次加权统计。由于采样窗口和调度边界，5.09 Hz 不表示保证长期超过配置的 5 Hz。

本次控制台输出的 ESDF delay 滚动均值范围为 58–82 ms，深度融合 delay 滚动均值范围为 27–34 ms。它们是整段日志中各统计窗口的均值范围，不是逐帧最小/最大延迟。测量点使用 ROS 图像时间戳到主机统计位置，不代表 GPU 完成或屏幕端到端延迟。

## 结论与限制

此静止小场景中，内部 ESDF 调用周期接近 5 Hz，约每 0.2 秒调用一次；订阅三维显示后，附加导出读回约占 106 ms/次，深度融合频率较第一阶段降低约 29.2%。结合当前适配节点在 ESDF 路径同步序列化全部分配块的实现，显示导出是本次可确认的明显开销来源。

这不能证明动态场景、完整扫描后的大地图、同时显示 Mesh/RViz 时仍有相同性能；也不能将主机 ESDF 调用耗时解释为 GPU 完整计算耗时。ESDF 计数是方法调用周期，未逐次验证每周期变化块数量。本次未测运动残影清除时间或完整覆盖率。

建议优先优化全层同步读回与显示导出，再补充运动轨迹下的帧龄和 GPU 完成测量。不能把降低显示点数或修改体素精度当作本次同条件优化成果。

测试结束后已停止本次启动进程，并检查 gzserver、mapper_node 无残留、11457 无监听。关闭阶段日志有 SIGINT/后续 Gazebo 退出异常记录，发生在测量完成后的清理期间，未计入上述采样窗口。

## 原始证据

- [结构化结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/results.json)
- [原始 metrics、时钟、Marker 样本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/samples.json)
- [启动与统计日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/launch.log)
- [运行时参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/runtime_parameters.yaml)
- [本次测量脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/realtime_test_20261103/measurement_script.py)
