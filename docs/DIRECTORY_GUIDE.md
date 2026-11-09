# 目录说明

[返回文档索引](README.md)

2026-10-27 增量：新增 `/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native`，共有 14 个源码目录。其 `src/` 为 C++ 节点/CUDA exporter/几何测试，`config/` 为原生及 Python 同条件配置，`rviz/` 为默认点云与可选 Mesh 配置。`build_native/`、`install_native/` 是隔离构建产物；`artifacts/native_cpp_20261027/` 是本轮原始数据/图像/证据。根目录新增 `build_ur7e_native.sh`、`run_ur7e_native.sh`、`run_ur7e_common_depth.sh`、`run_ur7e_native_rviz.sh`。详见 [模块说明](/home/czz/perception_eye_in_hand/docs/modules/ur7e_nvblox_native.md)。下文 13 包树是早期静态分析快照，不包含本次新增包。

## 工作区根目录

```text
perception_eye_in_hand/
├── README.md                         项目与文档入口（本次新增）
├── docs/                             中文代码说明（本次新增）
│   ├── README.md                     阅读顺序及功能包索引
│   ├── DIRECTORY_GUIDE.md            本文：目录用途
│   ├── ARCHITECTURE.md               架构、数据流与契约
│   ├── CODE_WALKTHROUGH.md           关键函数调用流程
│   ├── ENTRYPOINTS_AND_FINDINGS.md   入口、环境与当前问题
│   ├── analysis_manifest.json        本次静态分析范围统计
│   └── modules/                      13 个源码目录的独立代码说明
├── src/                              ROS 功能包与 Python 实现
├── build/                            colcon/CMake 构建缓存、中间文件
├── install/                          已安装节点、Python 包及 share 资源
├── log/                              colcon 构建日志及历史运行记录
├── artifacts/gazebo_perception/      已保存的 Gazebo 建图验证产物
├── build_*.sh                        按任务构建指定包
├── run_*.sh                          清理环境并启动对应 launch
├── *_control.sh                      服务调用/状态查看包装器
├── source_realtime_nvblox.sh          直接实时库环境与动态库准备
├── HANDOFF_REAL_ESDF_SAGE.md          真机 ESDF/SAGE 历史交接说明
├── .vscode/                          编辑器配置
├── .git/                             版本控制元数据
└── .agents/、.codex/、.aws/           当前环境提供的隐藏目录
```

源码修改应从 `src/` 开始；`install/` 可能是旧版本或符号链接，不能只通过已安装内容判断当前源码行为。`build/`、`install/`、`log/` 通常可以由工具重新生成，但本次未删除或重建它们。隐藏环境目录不属于感知算法实现。

## src 下的模块分组

| 目录 | 内部代码位置 | 用途 |
| --- | --- | --- |
| `realtime_nvblox/` | `realtime_nvblox/` | 实时数据、SDK、VIO、nvBlox 与 ROS 基础库 |
| `realtime_nvblox_2/` | 仍为 `realtime_nvblox/` | 基础库副本；包名与上项相同，新增部分输出实现 |
| `ur7e_d435_gazebo/` | `scripts/`、`src/`、`third_party/` | 仿真场景、标定工具、安装几何检查及相机插件 |
| `ur7e_nvblox/` | `ur7e_nvblox/` | 图像时刻 TF 深度建图与 GPU 查询优化 |
| `ur7e_reconstruction/` | `ur7e_reconstruction/` | 仿真彩色重建、表面导出、扫描和评估 |
| `ur7e_sage_planner/` | `ur7e_sage_planner/` | 固定障碍模型下的 SAGE 规划/伺服基线 |
| `ur7e_perception_planning/` | `ur7e_perception_planning_core/` | 感知地图与在线规划集成、独立几何检查 |
| `ur7e_real_static/` | `ur7e_real_static/` | 真机停稳 ESDF、冻结场、离线规划及 Action 执行 |
| `ur7e_real_reconstruction/` | `ur7e_real_reconstruction/` | 只读真机彩色重建 |
| `ur7e_auto_reconstruction/` | `ur7e_auto_reconstruction/` | RTDE 扫描协调与门控彩色采集 |
| `ur7e_auto_esdf/` | `ur7e_auto_esdf/` | RTDE 扫描协调与门控深度 ESDF |
| `ur7e_gazebo_esdf_sage/` | 同名 Python 目录 | 预扫描后引导搜索和分段 SAGE 执行 |
| `ur7e_gazebo_esdf_sage_continuous/` | 同名 Python 目录 | 连续优化、异步优化器及短轨迹队列 |

每个功能包的实际目录树和源文件逐项说明见[分包索引](README.md#功能包文档)。

## 各包内常见子目录

| 子目录/文件 | 含义 | 阅读或修改时的注意点 |
| --- | --- | --- |
| `包名/` 或 `ur7e_perception_planning_core/` | 可导入的 Python 源码 | 节点、算法、几何、线程/进程协作均在这里 |
| `launch/` | ROS 进程编排 | 默认值以实际 `DeclareLaunchArgument` 和配置读取为准 |
| `config/` | YAML、标定、碰撞球、路线、RViz 配置 | 有些包含旧机器绝对路径；配置会改变行为与地图身份 |
| `scripts/` | 安装后的可执行入口或辅助工具 | 部分无扩展名文件实际是 Bash，负责选择 Conda Python |
| `test/`、源码中的 `tests/` | 单元测试、集成驱动、故障注入、审计和绘图工具 | 不全是 pytest；场景驱动可能移动仿真障碍或机器人 |
| `examples/` | 示例核查与速率监控 | 一般需要已运行的 ROS/GPU 链路 |
| `rviz/` 或 `config/*.rviz` | 显示设置 | 坐标系、话题、颜色解释应与当前运行模式一致 |
| `resource/` | ament 包索引占位资源 | 用于 ROS 包发现，不是算法模型 |
| `package.xml` | ROS 包身份、依赖和构建类型 | 目录名不一定等于其中声明的包名 |
| `setup.py`、`setup.cfg` | Python 包发现、console_scripts 及安装路径 | 适用于 ament_python 包 |
| `CMakeLists.txt` | C++ 编译、Python/资源安装、自定义消息生成 | 多个纯 Python 应用仍通过 ament_cmake_python 安装 |
| `validation/` | 过去测试的日志、结果、审计、截图 | 状态以具体日期/会话为准，不能当成当前通过证明 |
| `report/` | 实验报告、观测图、指标、模型快照和生成脚本 | 用于复核历史，不是默认运行输入 |
| `__pycache__/`、`.pytest_cache/` | Python/pytest 缓存 | 不属于应分析的源代码 |

## 专用资源目录

`ur7e_d435_gazebo/urdf/` 保存机械臂/相机 Xacro，`meshes/` 保存几何网格，`src/check_mount_meshes.cpp` 使用 FCL 检查安装几何。`third_party/realsense_gazebo_plugin/` 的 `include/` 和 `src/` 是本地编译相机插件；许可证在 `licenses/`，专题说明在 `docs/`。

`ur7e_sage_planner/assets/` 保存机器人资源，`weights/` 保存模型权重等规划资源，`licenses/` 记录第三方条款。`resources.py` 将配置内的 `package://` 引用解析为安装位置，外部 `storm_kit` 提供控制器核心。

`ur7e_perception_planning/msg/PlanningMap.msg` 定义跨节点原子地图协议，是 mapper 与 planner 之间的重要契约。修改它时需要同时检查 `grid.py` 的打包、解析和 Snapshot 验证。

`ur7e_real_reconstruction/hardware/` 提供相机、固定安装 TF 和机器人驱动入口；应用本身只读，但这些脚本会启动外部硬件组件。`scan_design/` 包含候选姿态生成、可见性/净空评估、路线选择、NPZ 数组与离线观察方案。

## 结果文件如何阅读

| 文件或子目录 | 含义 |
| --- | --- |
| `surface.npz`、`surface.ply` | 彩色重建顶点、三角形、颜色和相应元数据 |
| `frozen_map.npz` | 规划距离、来源、网格、模型身份、会话与校验和 |
| `native_esdf.npz` | 原生 nvBlox ESDF 查询及未知标记，与保守规划场区分 |
| `observations/` | 选定观察点图像/深度、内参、实际关节和图像时刻相机位姿 |
| `plan.json`、`result.json` | 自动扫描的准备计划和终态结果；不能只看“到过最后一点” |
| `trajectory.npz`、`planning/` | 静态目标任务的规划轨迹、地图绑定和优化产物 |
| `execution.csv` | 实际关节、指令及短段执行记录 |
| `events.jsonl`、`sage_feedback.jsonl` | 目标、拒绝、保持、优化结果和地图版本等逐事件证据 |
| `online/` | 在线占据、提交版本和相关感知证据 |
| `artifacts/gazebo_perception/*.json` | 已保存的图、验证与场景限制记录；`REPORT.md` 是其说明入口 |

应用默认通常将新数据写入用户目录下 `nvblox_output/` 的任务子目录，具体看选定 YAML、launch 和命令行参数。仓库内的 report/validation 是已保存副本；不能假设新运行会写回同一处。
