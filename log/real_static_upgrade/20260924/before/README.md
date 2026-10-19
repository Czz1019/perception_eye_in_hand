# ur7e_real_static

眼在手上真实场景：示教器带动预扫描 → 静态 ESDF → SAGE 离线规划 → 显式请求低速轨迹执行。
不会启动 Gazebo、连接机器人 IP、安装驱动、自动运动或继承仿真扫描先验。

- [实现与计划](ToDoList.md)
- [真实场景测试说明](REAL_TEST_GUIDE.md)

在工作区根目录构建 `./build_ur7e_real_static.sh`，填完 `config/real.yaml` 的副本后启动：
`./run_ur7e_real_static.sh config:=/absolute/path/my_real.yaml`。
应用运行于 whole_control；设备驱动在系统 ROS 环境独立运行。
默认 `execution_enabled: false`，发布目标只生成预览，不发送机械臂指令。
