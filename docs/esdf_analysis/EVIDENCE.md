# ESDF 分析：版本、代码定位与证据口径

[返回主报告](../ESDF_REALTIME_ANALYSIS.md)

## 版本和分析方式

| 对象 | 标识 | 本次处理 |
| --- | --- | --- |
| 当前工作区 | Git HEAD `3ac915bb9adde9f2b9a3717d7332d29501f8eca2` | 静态阅读源码、配置、已有报告和 JSON；代码文件未修改 |
| 用户压缩包 | `/home/czz/realtime_nvblox_ws.zip` | 安全路径校验后只解压源码/配置到 `/tmp/eye_esdf_audit/archive/`；排除 `.git/build/install/log` 和缓存 |
| 压缩包 SHA256 | `63445819083e1eb9bc36191bcea20ff2f46388bc6f7014e7333bc7aa26773eb6` | 47 个基础包文件逐字节比较，结果见 JSON |
| NVIDIA 官方 | 默认分支 `public`，提交 `5bd2d73da901ce0ba279c997a4c1f9f632b9309c` | 通过 GitHub 连接器固定提交读取 16 个源码/文档文件 |
| 官方提交时间 | `2026-10-23T18:50:58Z` | 分析日 2026-10-26；不把可变分支名当永久版本 |

先尝试了 Firecrawl 技能，但本环境没有 CLI，技能所引用的细分文件也不在安装目录，因此改用网页工具确认官方仓库，再用已连接 GitHub 工具读取版本/源码。并行读取中出现连接失败，之后逐项读取成功；最终清单只包含实际读取成功的文件。

本次没有安装依赖、导入/初始化 CUDA、启动控制器、打开相机、运行仿真或运行集成测试。没有把压缩包中的操作说明当作本次授权命令。没有把网页/历史报告中的指令当作需要执行的工作。

当前环境未确认有 `whole_control` 的对应部署，仓库 requirements 也未完整锁定 Torch/nvblox_torch/CUDA 版本。代码中的 v0.0.10 兼容注释和历史环境记录不能代替当前已安装 wheel 的真实提交证明；官方固定提交仅用于实现比较。

## 本地代码定位

行号对应分析时的当前源码，后续修改会变化。主报告已经解释机制，这里提供可追踪定位。

| 文件 | 行/符号 | 对应结论 |
| --- | --- | --- |
| [基础 nvblox_backend.py](../../src/realtime_nvblox/realtime_nvblox/nvblox_backend.py) | `_initialize` L65；`query_esdf_grid` L191 | 原生 Mapper 参数、查询缓存和锁内 CPU 读回 |
| [基础 runtime.py](../../src/realtime_nvblox/realtime_nvblox/runtime.py) | `_mapping_loop` L255；`_esdf3d_loop` L352 | 原生更新、查询和生成调度 |
| [基础 ros_node.py](../../src/realtime_nvblox/realtime_nvblox/ros_node.py) | `_publish_latest_esdf3d` L174 | 点云打包/发布；本函数没有订阅数量门控 |
| [ros_publishers.py](../../src/realtime_nvblox/realtime_nvblox/ros_publishers.py) | `esdf_cloud` L112–194 | 20 字节/点、未知颜色和近表面带筛选 |
| [参考扩展后端](../../src/realtime_nvblox_2/realtime_nvblox/nvblox_backend.py) | `observed_esdf_numpy` L114 | TSDF 正权重候选坐标上的原生 ESDF 导出 |
| [EfficientBackend](../../src/ur7e_nvblox/ur7e_nvblox/esdf.py) | `prepare_query` L13；`query_prepared` L23 | 已经具备的固定查询缓冲复用与未知值清理 |
| [TfMapper](../../src/ur7e_nvblox/ur7e_nvblox/node.py) | `_worker` L152；`_publish_latest` L268 | TF 等待、融合、开始到开始调度、独立发布/订阅门控 |
| [PlanningMapper](../../src/ur7e_perception_planning/ur7e_perception_planning_core/mapper.py) | L105–117、L175–208、L231–238 | timestamp-ready 选择、全网格派生、combined 直接快照 |
| [grid.py](../../src/ur7e_perception_planning/ur7e_perception_planning_core/grid.py) | `observed_free` L40；`conservative_field` L65；`Snapshot.query` L131 | 全体积投影、未知边界 EDT、保守插值 |
| [edt.py](../../src/ur7e_perception_planning/ur7e_perception_planning_core/edt.py) | L24–38 | 逐轴 n×n 候选枚举、临时矩阵分批与读回 |
| [runtime GPU_LOCK](../../src/ur7e_perception_planning/ur7e_perception_planning_core/runtime.py) | `MappingPriorityLock` | 队列优先不等于抢占已开始的工作 |
| [SAGE 地图接入](../../src/ur7e_perception_planning/ur7e_perception_planning_core/sage.py) | `set_map` L28；`query` L34 | CPU 数组复制、GPU 上传和三线性采样 |
| [通用自滤波](../../src/ur7e_perception_planning/ur7e_perception_planning_core/self_filter.py) | `robot_depth_mask` | CPU 逐球射线求交 |
| [真机静态 FieldBuilder](../../src/ur7e_real_static/ur7e_real_static/field.py) | `update` L74；`FrozenMap.create` L27 | 全域自由证据、裁剪/缓存 EDT、未知非正约束 |
| [真机静态 capture](../../src/ur7e_real_static/ur7e_real_static/capture.py) | L43–61、L93–109、L116–125 | 停稳/对齐、前后年龄检查及计时边界 |
| [真机状态历史](../../src/ur7e_real_static/ur7e_real_static/node.py) | L123–147 | 高频回调、列表复制和顺序时间查找 |
| [真机显示线程](../../src/ur7e_real_static/ur7e_real_static/visualization.py) | L7–34 | 已经存在的订阅门控、latest-only、独立显示 |
| [自动 ESDF capture](../../src/ur7e_auto_esdf/ur7e_auto_esdf/capture.py) | L107–127 | 提交临界区、观测记录及非纯算子计时 |
| [自动 ESDF mapper](../../src/ur7e_auto_esdf/ur7e_auto_esdf/mapper.py) | L133–146、L156–169 | 锁内 NPZ/JSON、raw_surface 全数组复制、固定 planning_ready=false |
| [旧版在线层](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/online.py) | L33–63、L79–97 | 最新槽覆盖、CPU 自由投影、CPU EDT 与记录 |
| [v2 在线层](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/online_v2.py) | L36–69、L99–149、L193–229 | 保留待同步帧、几何进程、大数组往返、主进程共享版本快照 |
| [仿真批量自滤波](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/self_filter.py) | `robot_depth_mask` | CPU 像素×球批量矩阵；与通用版成本不同 |
| [旧版闭环](../../src/ur7e_gazebo_esdf_sage/ur7e_gazebo_esdf_sage/node.py) | L277–342 | 完整引导先行、等待短段、mean_action 重置、shift_steps=0 |
| [连续规划](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/continuous.py) | `reach`、`checked_prefix` | 最终目标优化、后台路线、原子检查/替换前缀 |
| [连续 optimizer](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/optimizer.py) | `compute`、`Planner.submit` | 单任务通道、版本化地图传输和时域平移 |
| [连续 launch](../../src/ur7e_gazebo_esdf_sage_continuous/launch/bringup.launch.py) | `generate_launch_description` | 默认 strict、v1、online_obstacles=false |

## 历史结果应如何使用

| 证据 | 主要信息 | 局限 |
| --- | --- | --- |
| [artifact verification.json](../../artifacts/gazebo_perception/verification.json) | 已有仿真建图、时间和支持域一致性数字 | 属于历史版本/会话，不能当当前复测 |
| [sphere_verification.json](../../artifacts/gazebo_perception/sphere_verification.json) | 球视角共同格距离变化 | 是一致性，不是绝对真值误差 |
| [NVBLOX2_LEARNING.md](../../src/ur7e_perception_planning/validation/NVBLOX2_LEARNING.md) | 同输入/参数比较、算子与完整任务不同收益 | 原始完整运行日志部分指向旧机器目录 |
| [benchmark.json](../../src/ur7e_real_static/validation/upgrade_20260924/benchmark.json) | 373,977 格/65 帧数值一致与分段耗时 | 微基准不含完整 TF/ROS/自滤波链路 |
| [REAL_STATIC_OPTIMIZATION.md](../../src/ur7e_real_static/validation/REAL_STATIC_OPTIMIZATION.md) | 已做的缓冲/显示/EDT 优化，高分辨率长尾 | 合成输入/模拟控制器，与真实输入负载不同 |
| [FIRST_REAL_SCAN.md](../../src/ur7e_real_static/validation/FIRST_REAL_SCAN.md) | 真机 30 秒空档、输入采样与冻结覆盖 | 同时改变多个设置，未隔离调频机制 |
| [自动实机 metrics.json](../../src/ur7e_auto_esdf/report/metrics.json) | 64 个成功状态样本的 P50/P95/max | 不是所有尝试，且自由场区间包含记录/提交 |
| [自动实机报告](../../src/ur7e_auto_esdf/report/2026-10-13_real_auto_esdf_test.md) | 计时口径、196 秒分解、冻结与就绪边界 | 没有完整逐帧原始录像/全空间真值 |
| [online_stage1](../../src/ur7e_gazebo_esdf_sage/validation/online_stage1/REPORT.md) | v1 最大成功提交空档 70.280 s | 当时拒绝原因不全，无法唯一归因 |
| [moving_stage2](../../src/ur7e_gazebo_esdf_sage/validation/moving_stage2/REPORT.md) | v2 r4 提交间隔/延迟与失败轮次 | 固定低速方块场景，不能推为任意动态场景 |
| [连续开发记录](../../src/ur7e_gazebo_esdf_sage_continuous/validation/DEVELOPMENT.md) | debug1–4 的进程调整与保留的失败 | 进程隔离/非正规数处理等因素未完全单变量分离 |
| [连续审计 JSON](../../src/ur7e_gazebo_esdf_sage_continuous/validation/debug4_continuous_audit.json) | 已接受轨迹延迟、guard、目标与过期拒绝 | 只对相应运行/接受轨迹成立 |

本次已阅读上述仓库内实际存在的文件。它们指向的 `/home/wqj/nvblox_output/...` 等外部原始会话不一定在当前机器，本次没有声称已核查所有被引用的外部文件。

## 工作量计算的口径

网格数按配置的三个轴节点数相乘；`float32_field_bytes=4N`，当前 `esdf_cloud` 的全点云为 `20N`，PlanningMap 原始三数组为 `9N`。MB 使用十进制 1,000,000 字节，不含 ROS headers、pickle/压缩元数据、重传和额外内存副本。

CUDA EDT 候选数 `N×(nx+ny+nz)` 是完整未裁剪、未 padding 盒体下的算法结构估计。实际代码裁剪自由体积且加边界，因此它不是实际 kernel 运行量，也不是耗时或 FLOP 测量。

v2 IPC 估计只算传入 bool 占据+float32 距离，以及返回 bool 占据+bool hit+float32 距离，共 `11N` 字节。未计图像和其他对象，也没有把载荷大小等同于复制延迟。

## 官方来源清单

以下链接全部固定到同一个提交；[evidence.json](evidence.json) 记录保存用于分析的 UTF-8 文本 SHA256。该文本由连接器返回并写入临时文件，可能标准化尾换行；SHA256 不是 Git blob 对象哈希。

| 官方文件 | 用途 |
| --- | --- |
| [docs/pages/torch_examples_esdf.rst](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/docs/pages/torch_examples_esdf.rst) | ESDF Python 示例说明 |
| [executables/src/fuser/fuser.cpp](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/executables/src/fuser/fuser.cpp) | 深度、ESDF、Mesh 分别降采样调度 |
| [nvblox/include/nvblox/integrators/esdf_integrator_params.h](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/include/nvblox/integrators/esdf_integrator_params.h) | 未知策略、最大距离和权重阈值 |
| [nvblox/include/nvblox/mapper/internal/impl/mapper_impl.h](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/include/nvblox/mapper/internal/impl/mapper_impl.h) | 深度融合登记更新块 |
| [nvblox/include/nvblox/mapper/mapper.h](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/include/nvblox/mapper/mapper.h) | Mapper 接口及默认非全层 ESDF 更新 |
| [nvblox/src/integrators/esdf_integrator.cu](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/src/integrators/esdf_integrator.cu) | site、失效清理、块内 sweep、邻块传播及同步 |
| [nvblox/src/map/blocks_to_update_tracker.cpp](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/src/map/blocks_to_update_tracker.cpp) | 按更新类型跟踪块状态 |
| [nvblox/src/mapper/mapper.cpp](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox/src/mapper/mapper.cpp) | 待更新块与 ESDF 调度 |
| [nvblox_torch/cpp/src/py_mapper.cu](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/cpp/src/py_mapper.cu) | Torch/C++ 查询与 stream 同步 |
| [nvblox_torch/cpp/src/sdf_query.cu](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/cpp/src/sdf_query.cu) | GPU signed ESDF/球半径/未知查询规则 |
| [nvblox_torch/nvblox_torch/examples/esdf/esdf.py](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/nvblox_torch/examples/esdf/esdf.py) | 稠密网格查询与可视化示例 |
| [nvblox_torch/nvblox_torch/examples/realsense/run_realsense_mapper.py](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/nvblox_torch/examples/realsense/run_realsense_mapper.py) | RealSense/VSLAM 表面重建示例，非完整保守 ESDF 规划链路 |
| [nvblox_torch/nvblox_torch/mapper.py](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/nvblox_torch/mapper.py) | Python Mapper、预分配输出和查询入口 |
| [nvblox_torch/nvblox_torch/mapper_params.py](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/nvblox_torch/mapper_params.py) | Python 可用参数接口 |
| [nvblox_torch/nvblox_torch/tests/test_benchmark.py](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/nvblox_torch/nvblox_torch/tests/test_benchmark.py) | 调用基准测试入口；未当成本机性能证据 |
| [release-notes.md](https://github.com/nvidia-isaac/nvblox/blob/5bd2d73da901ce0ba279c997a4c1f9f632b9309c/release-notes.md) | 官方性能口径与版本功能边界 |
