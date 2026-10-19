# 2026-09-20 MarkerArray QoS 修复

## 原因和改动

`/perception_planning/markers` 原发布端使用 Best Effort，RViz 默认请求 Reliable，
因此 DDS 拒绝建立这组收发关系。目标接收日志与该可视化订阅警告属于不同的数据通路。

修复包含两项：

1. markers 发布端改为 Reliable、Keep Last、depth=1，可同时服务 Reliable 和 Best Effort 订阅。
2. RViz MarkerArray 配置将错误的 `Marker Topic` 字段改为 `Topic`，明确设置 Reliable、Volatile。

字段依据是本机 ROS Humble 的 `rviz_common/ros_topic_display.hpp`：
`_RosTopicDisplay` 创建的属性名为 `Topic`；MarkerArrayDisplay 继承该显示类型。
ESDF、相机等其他话题的 QoS 没有改变。

## 实测

执行默认命令 `./run_ur7e_perception_planning.sh`，实际启动 Gazebo headless 和 RViz。
创建独立的 Reliable、Best Effort 测试订阅，再发布 base_link 目标 `[0.5, 0.5, 0.45]` m。

- 发布端与 RViz 订阅端的端点信息均为 RELIABLE。
- 两种测试订阅各收到 6 条 MarkerArray，其中各 3 条确认包含坐标完全一致的新目标标记。
- 消息包含 20 个机器人/相机/支架碰撞球。
- RViz 截图确认目标球和碰撞球实际显示，Global Status 正常。
- 运行日志中没有 `incompatible QoS` 或 `RELIABILITY` 不兼容警告。
- 测试时规划节点 `error=null`；本次没有验收该远目标的最终到达能力。

工作区证据位于 `log/perception_planning_markers_fix/`：

- `verification.json`、`verification.log`：六项检查全部通过。
- `endpoints.txt`：发布端、RViz 和两种独立订阅的 QoS。
- `validate_markers.py`：本次运行验证脚本。
- `rviz_markers.png`：实际显示截图。
- `launch.log`：启动、目标接收和退出日志。

测试结束后关闭了本次启动的仿真和 RViz。重新运行默认启动脚本即可加载修复。
目标 `[0.5, 0.5, 0.45]` 的绕障到达验证已列入 [ToDoList.md](../ToDoList.md)，
不能把标记成功显示当作已经实现目标可达。
