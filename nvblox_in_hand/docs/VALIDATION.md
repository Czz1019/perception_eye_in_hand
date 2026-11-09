# 已完成工作与验证边界

2026-11-04重构提示：本文保留此前结构/运行记录。当前默认已改为官方NvbloxNode组件，入口、配置及显示选择见 [重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md) 和 [最新操作说明](OPERATIONS.md)。此前inside-only默认、20字节点云及旧频率不能当作新版接口或结果；本轮未运行测试。

日期：2026-10-28。

## 已完成

- 在父工程目录下新建独立 `nvblox_in_hand/`，保留场景、机器人、标定和官方 ROS/CUDA 模块结构。
- 提供默认官方核心依赖复用构建，以及独立目录中的官方源码快照和可选全量源码构建入口。
- 提供官方后端和薄适配后端、原生 Mesh/体素/ESDF/服务、图像时刻机械臂 TF 路径、D435 驱动入口、功能开关和可选扫描。
- 提供完整目录、参数、话题、仿真、实机输入、保存/重放和效果对比文档。

## 已通过的验证

| 验证 | 结果 | 证据 |
| --- | --- | --- |
| 默认构建和安装 | ur7e_d435_gazebo、nvblox_in_hand 两包成功 | artifacts/build_verified.log，退出码 0 |
| C++ 原生 ESDF 显示策略 | 未观测剔除、非法距离剔除、距离单位和内外符号通过 | test.sh 的 esdf_view_policy_test |
| Python launch 契约 | 4 项通过，含单 D435、原始输入、机器人 TF、开关及官方对照后端 | artifacts/offline_tests.log |
| FK/FCL 与扫描依赖检查 | 7 项通过 | 同上 |
| 启动接口 | --show-args 及实际 launch Action 构造通过；没有执行 Action 启动进程 | artifacts/launch_arguments.txt |
| 二进制依赖 | mapper ldd 无 not found | artifacts/mapper_ldd.txt |
| 来源校验 | 所有复制来源文件仍匹配复制前哈希 | provenance/local_changes.json |

默认构建使用主机环境完成。在工具沙盒中，colcon 曾等待子进程不返回；直接 CMake 编译/链接正常，主机环境下完整 colcon 构建成功。已清理本轮重复等待的构建进程。构建脚本限制并行度，并关闭桌面通知，不依赖扫描/仿真进程。

## 未做的验证

没有启动 Gazebo、打开 RViz、接通 D435、移动机械臂或执行真实扫描；没有新的画面、几何精度、表面覆盖率、实际频率、GPU 完成延迟或端到端显示延迟测量。

没有执行 `build.sh --from-source` 的整套官方 CUDA/NITROS 源码重编译。该入口及源码快照已经提供，但不能当作已验证的第二种构建结果。默认成功构建不需要改动 workspaces；运行仍依赖其兼容的已安装官方库。

以上状态同时记录在 artifacts/verification.json。历史父工程实验和离线可达性计划不替代本工程的运行验证。

## RViz 闪退修正补充（2026-10-28）

此次修正后默认构建再次成功。新增状态检查相关测试后，Python 离线测试为 13 项全部通过，另有 C++ ESDF 策略测试通过。新的记录位于 `artifacts/rviz_fix_20261028/`，与上文初次生成记录区分。

使用本目录解包的 Xvfb，在独立 ROS 域 129、虚拟显示和软件 OpenGL 中进行 Mesh 插件回归：接收 48 条测试 Mesh，存活约 10 秒，主动关闭后退出码 0，没有缺失 box.geom 异常。最初 Qt offscreen 路径因 GLX 窗口句柄不兼容失败，没有进入 Mesh 验证；成功结果来自后续虚拟 X 显示，不混淆两次尝试。

未修改系统安装，虚拟显示依赖仅解包至 `.test_runtime/`，不是运行仿真的必需依赖。未在用户桌面打开 RViz，未启动 Gazebo、执行机械臂运动或验收真实场景中的机器人状态。虚拟测试三角网格不是重建结果。

本轮完整启动与验收步骤见 [仿真运行手册](SIMULATION_RUNBOOK.md)。机械臂状态需在场景运行时执行 `./check_sim.sh` 核对；当前日志本身不能证明曾出现重复发布者或 TF/关节不一致。
