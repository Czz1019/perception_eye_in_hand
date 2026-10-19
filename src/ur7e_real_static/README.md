# ur7e_real_static

眼在手上真实场景：示教器带动预扫描 → 静态 ESDF → SAGE 离线规划 → 显式请求低速轨迹执行。
不会启动 Gazebo、连接机器人 IP、安装驱动、自动运动或继承仿真扫描先验。

- [实现与计划](ToDoList.md)
- [真实场景测试说明](REAL_TEST_GUIDE.md)
- [2026-09-27 真实扫描图文报告（21 个姿态与最终地图）](report/2026-09-27_real_scan_report.md)
- [2026-09-24 感知优化与对照测试](validation/REAL_STATIC_OPTIMIZATION.md)
- [2026-10-13 D435i 碰撞包络适配及旧扫描点复查](validation/camera_collision/REPORT.md)

已新增 `config/collision_spheres_d435i.yaml` 和本机专用 `config/real_d435i.yaml`：按实际固定外参将相机单大球改为 6 球包络，保留 5 mm 包络余量和原自碰撞阈值。
历史扫描结束姿态的自碰撞检查通过；旧自动扫描八点仍不满足 15 mm 余量，支架/工具/线缆仍缺实测几何。
新配置默认禁止执行，并使用新的输出目录；原预览文件和自动扫描入口没有切换到新模型。详见上述适配报告。

在工作区根目录构建 `./build_ur7e_real_static.sh`，填完 `config/real.yaml` 的副本后启动：
`./run_ur7e_real_static.sh config:=/absolute/path/my_real.yaml`。
应用运行于 whole_control；设备驱动在系统 ROS 环境独立运行。
默认 `execution_enabled: false`，发布目标只生成预览，不发送机械臂指令。

## 本次感知改进

复用 GPU ESDF 查询缓冲，使用局部裁剪的精确 CUDA 距离变换，将 RViz 点云打包/发布放入独立线程；无订阅时跳过打包，冻结后仍能向晚加入的 RViz 提供最后一帧。
默认 `capture_hz: 15.0` 是采集处理上限，`visualization_hz: 1.0` 是显示上限；实际完成率取决于输入、TF、停稳状态和计算负载。
`/ur7e_real_static/status` 的 `perception` 字段提供完成率、耗时、丢帧和等待原因。

每个提交的规划距离场均保持完整计算；未知空间阻止规划。默认流程仍为停稳预扫描、冻结后规划、显式低速执行。
已完成软件及模拟控制器测试；2026-09-27 真实 UR7e/D435i 完成 1233 帧/21 姿态扫描、静态 ESDF 冻结与 RViz 显示，见 [首次实机记录](validation/FIRST_REAL_SCAN.md)。当前起点观测覆盖和自碰撞球模型检查未通过，目标轨迹预览、实体精度、控制和停止效果仍待验证。
