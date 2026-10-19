# 2026-09-20 默认启动与 SSL 报错修复

## 问题与处理

用户报告 `gzclient` 的 `libcurl (56) / OpenSSL SSL_read / unexpected eof`。
本机带 GUI 的复现日志也捕获到 `gzclient` 的 `libcurl (35) / unexpected eof`。
Gazebo 图形界面会异步请求在线模型列表，相关路径见
[Gazebo InsertModelWidget 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/gui/InsertModelWidget.cc)。
感知规划节点没有因为这条网络错误退出；它与相机/ESDF 的计算错误不同。

按照用户允许的 headless 方案，将 bringup 和 scene 两个入口的 `gazebo_gui` 默认值均改为 false。
默认不再启动发出该请求的 gzclient，保留 gzserver、相机渲染、TF、控制器和 RViz。
这是默认运行路径的修复，不是对外部服务器 TLS 问题的修复；显式 `gazebo_gui:=true`
仍可能出现网络错误。没有关闭证书校验，也没有修改用户的网络或系统 Gazebo 配置。

带 GUI 的复现中还记录到地图处理延迟和 STALE_MAP 停顿；新场景相机分辨率从
640×360 调整为 320×180，以减少渲染、图像传输和自过滤负载。CameraInfo 来自相机插件，
感知继续按消息内参和 depth_stride 计算；ESDF 网格分辨率与超时保护未改变。

退出测试另外发现 Humble 的订阅取消息与 SIGINT 关闭上下文的竞争异常。
入口只在 ROS 上下文已关闭时处理该 RuntimeError；上下文正常时仍抛出异常。
两项新增回归测试分别验证关闭期间正常清理、正常运行时异常不被吞掉，均通过。

## 最终两遍验证

两遍均从停止状态执行同一条不带参数的命令：

```bash
./run_ur7e_perception_planning.sh
```

每次启动后运行 `validate_sim.py --exercise --faults --duration 60`，随后发送 SIGINT 并等待退出。

| 项目 | 第一遍 | 第二遍 |
| --- | --- | --- |
| gzserver / RViz | 正常 | 正常 |
| gzclient | 未启动 | 未启动 |
| 两个 ros2_control 控制器 | active | active |
| 13 项闭环验收 | 全部通过 | 全部通过 |
| 位置目标误差范围 | 10.97～11.84 mm | 11.28～11.95 mm |
| 地图平均更新频率，含故障注入 | 2.35 Hz | 2.33 Hz |
| 更新间隔 P95 | 0.405 s | 0.404 s |
| libcurl / SSL_read / unexpected eof | 未出现 | 未出现 |
| Python Traceback / ERROR | 未出现 | 未出现 |
| 所有运行组件退出码 | 0 | 0 |

闭环检查包括在线地图更新、实际机械臂运动、FK/TF 对照、几何净空、三个位置目标到达、
墙内目标被阻止、运动中断流停动、恢复感知和地图重置。
结果摘要见 [headless_runs.json](headless_runs.json)。

原始日志位于工作区 `log/perception_planning_startup_fix/`：

- `reproduce.log`：原配置复现；`fixed_run1.log`：带 GUI 时捕获的 SSL 错误。
- `final_run1.log`、`final_run2.log`：最终两遍启动、运行和退出日志。
- `final_run1.json`、`final_run2.json`：完整验收数据。
- `final_run*_controllers.txt`、`final_run*_processes.txt`、`final_run*_windows.txt`：运行组件证据。
- `shutdown_tests.log`：新增退出行为回归测试。
- `original_integrity.log`：realtime_nvblox 原有 Python 源文件校验通过。

日志仍可能包含依赖库的非致命提示；第一遍退出时出现未 await 协程的 RuntimeWarning，
但全部节点退出码为 0，第二遍未出现该提示。本次没有屏蔽这些警告。
两遍测试结束后均已关闭测试进程，没有留下 Gazebo 服务端或客户端。
