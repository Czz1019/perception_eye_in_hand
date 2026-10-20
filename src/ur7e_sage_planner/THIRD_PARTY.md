# 来源

2026-09-18 从 `/home/wqj/storm` 复制本包所需文件，原文件不做修改：

- clean_SAGE 的两个规划入口、joint_servo_executor.py 和 record_joint_smoothness.py。
- examples/SAGE_MPPI/deployment_refinement.py，保持算法和参数。
- examples/sim_gazebo 的障碍物工具，以及两个 reach_static 文件中的 ROS 接口、
  坐标变换、碰撞球可视化与子进程清理函数。未包含旧规划入口。
- 对应任务、机器人、世界、初始状态和碰撞球配置。
- content/assets/urdf/ur7e 的规划模型和网格，以及 ur7e_self_sdf.pt 权重。

迁移改动仅涉及包导入、资源定位、可写运行目录、conda 入口和场景障碍物生成开关。
storm_kit 继续使用 whole_control 环境中的安装，未复制或修改其源码。
STORM 和 UR 模型许可证保存在 licenses/ 下。
