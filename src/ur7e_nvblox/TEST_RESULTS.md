# 本机验证记录（2026-09-16）

环境：ROS 2 Humble / Gazebo Classic；感知进程位于
`/home/wqj/miniconda3/envs/whole_control/bin/python`，PyTorch 2.9.1+cu128，
NVIDIA GeForce RTX 5060 Laptop GPU。

## 已完成

- colcon 构建通过。
- 6 项 pytest 测试通过：毫米/米解码、大小端和行填充、无效深度、
  降采样光线一致性、相机到 base 坐标变换、真实 CUDA 已知平面 ESDF、
  时钟回退后的旧 DDS 图像防护（部分项目合并在同一测试中）。
- 使用实际 `ur7e_d435_gazebo` 场景，原场景验证脚本通过。
- 实际接入 1280×720、16UC1 深度图，降采样为 640×360；
  内部体素 0.03 m，查询网格 0.05 m、52111 个采样点。
- 连续 ESDF 消息包含有效距离与近表面样本，坐标系为 `base_link`。
  初始视角约有 2907 个已观测单元，其余保留未知标记。
- 输出位姿与图像时间戳 TF 的矩阵元素最大误差约 2.3e-8。
- shoulder_pan 平滑改变 0.08 rad，相机在 base 中平移约 0.04693 m；
  感知继续运行，随后恢复起始关节姿态，最大关节恢复误差约 2.9e-5 rad。
- `/ur7e_nvblox/reset_map` 后生成新地图并恢复有效 ESDF。
- 地图保存服务成功生成约 2.3 MB 的
  `~/nvblox_output/ur7e_sim/base_link_map.nvblox`。
- 保持 mapper 运行、重启 Gazebo 场景后，时钟回退触发新建图并恢复输出。
- 原 `src/realtime_nvblox` 的 58 个文件逐一 SHA256 比较一致，包括原有缓存文件。
  原 `ur7e_d435_gazebo` 源码也没有修改。

## 性能与边界

完整点云的稳定观测窗口接收约 3.09 Hz；另一负载窗口约 0.87 Hz。
**本次验证确认了功能正确性，没有达到或宣称稳定 10/30 Hz。**
频率受本机仿真输入、TF 到达、CPU 调度和大点云传输影响。
稳定窗口中融合加 ESDF 查询耗时中位数约 1.86 ms、p95 约 4.94 ms，
这不包含 TF 等待、输入传输和输出传输，不能当作端到端延迟。
10 Hz 配置是更新上限。

通过独立发布线程和 RELIABLE 完整点云降低输出对输入/TF 的干扰；
RViz 使用更小的 `esdf_observed` 点云。超时 TF、过期图像及积压帧会被丢弃并计数。
未知区域没有被视为自由空间，单个相机视角不保证覆盖所有障碍物。

本机直接调用 Gazebo `/reset_simulation` 后曾出现上游关节 TF 停止正常更新。
mapper 会等待有效 TF，重启整个场景可恢复；日常清图使用 mapper 的 reset_map 服务。

## 原始证据

工作空间 `log/ur7e_nvblox_validation/` 中保存：

- `unit_tests.log`、`build.log`
- `scene_check.json`、`esdf_check.json`、`post_motion_check.json`
- `motion_reset_check.json`、`scene_restart_check.json`
- `save_map.log`、`source_integrity.json`
- `exercise_scene.py`：本次运动测试脚本，只用于已启动的 Gazebo 测试实例；
  会发布关节指令并恢复初始姿态，记录默认写入 `/tmp/ur7e_esdf_validation/`。

测试期间只控制了本次启动的仿真机械臂，没有连接真实机器人。
