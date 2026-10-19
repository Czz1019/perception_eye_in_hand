# 软件验证记录

2026-09-23，ROS Humble、whole_control、原生 nvBlox/CUDA 和实际 storm_kit SAGE。
没有接入 UR7e 或 RealSense 硬件；测试使用隔离的 ROS_DOMAIN_ID 和 `/mock/...` 控制接口。

## 结果

- 构建成功，见 `build.log`。
- **15 项测试全部通过**（3.30 s）；3 条上游 PyTorch/SWIG 弃用警告，无测试失败。
- **连续两遍 ROS 集成测试通过**：分别冻结 5 / 6 帧、2 个视角；每遍 21,986 个实测射线自由体素、290,038 个测试先验体素；SAGE 到达规划执行 16 次优化。
- 合成障碍 GPU 测试验证名义关节插值会碰撞，但生成的目标轨迹满足整臂/自碰撞和速度检查。此项使用解析距离场，与深度融合测试分开验证。
- 实际根目录启动脚本在默认预览模式正常启动，SIGINT 后正常退出。
- 原感知包 17 个 Python 源文件 SHA256 与本次工作开始前一致。

## 已执行

- 单元与 GPU 测试日志：`tests.log`。
- ROS 集成记录：`mock_run_1.json`、`mock_run_2.json`，相应完整日志同目录。
- 命令行启动/退出记录：`launch_smoke.log`。
- 原感知包源码校验：`original_integrity.log`。

ROS 测试链路真实调用了深度解码、图像时间 TF、机器人自身过滤、nvBlox 融合及 ESDF 查询，冻结地图后调用实际 SAGE，发送真实 ROS FollowJointTrajectory Action 给模拟控制器。
集成测试同时检查无自动启动运动、无地图拒绝、场景未确认拒绝、静态地图超过原在线时限仍能使用、断流取消、失效清图、重载需重新确认。

**测试边界**：输入深度来自合成平面；模拟场景另含一个明确标注、独立定义的固定自由区，以覆盖相机不可见的测试机器人近场。
日志分别记录实际观测自由体素和先验体素。它证明软件链路可执行，不证明真实预扫描覆盖、测量精度或实际无接触。
模拟控制器理想跟踪五次插值，误差不能作为真机精度指标。


## 复现命令

在工作区根目录构建后执行。输出路径必须不存在；需要 whole_control 中已有的 CUDA/nvBlox/storm_kit。不会连接真实硬件。

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/setup.bash
cd /home/wqj/realtime_nvblox_ws
export OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/wqj/miniconda3/envs/whole_control/bin/python -m pytest -q src/ur7e_real_static/test
ROS_DOMAIN_ID=94 /home/wqj/miniconda3/envs/whole_control/bin/python \
  src/ur7e_real_static/test/run_mock_integration.py --output /tmp/ur7e_real_static_test_1
ROS_DOMAIN_ID=95 /home/wqj/miniconda3/envs/whole_control/bin/python \
  src/ur7e_real_static/test/run_mock_integration.py --output /tmp/ur7e_real_static_test_2
ROS_DOMAIN_ID=96 /home/wqj/miniconda3/envs/whole_control/bin/python \
  src/ur7e_real_static/test/run_launch_smoke.py --output /tmp/ur7e_real_static_launch_test
```

集成测试降低了最少帧数/视角/稳定等待时间以缩短测试，并明确启用测试自由区。生产配置仍是默认 15 帧、3 视角、空自由区、禁止执行。
此次没有做真实驱动联机、真实 UR 控制器插值/取消、实体碰撞、相机测量精度或实机绕墙验收，也没有重新启动 Gazebo。
