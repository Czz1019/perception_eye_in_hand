# RGB-D融合限频优化与三维ESDF复测

后续补充：已重新完成融合门限30/33.4/40Hz各三轮对照，首次补齐融合33.4Hz实测。见[九轮复测报告](FUSION_18HZ_MULTIRUN_RETEST_20261105.md)。以下表格保留其原测试条件和历史数值。

日期：2026-11-05。修改位于 `/home/czz/perception_eye_in_hand/nvblox_in_hand`；workspaces及父工程未改。本轮完成的是上一轮方案的第一阶段——为30Hz输入设置有余量的融合限频，并进行同场景对照。没有宣称其他调度、时序、自过滤和GPU优化已完成。

## 1. 本轮解决什么问题

此前640×480相机输入约30Hz，但深度/颜色融合只有约18Hz。官方ROS实现用 `当前图像时间戳－上一次融合时间戳 > 1/融合限频` 过滤消息。融合限频也设为30Hz时，约33ms的图像间隔会小于33.333ms门限，被主动丢弃。

本轮将默认realtime档的深度和颜色融合限频改为 **40Hz**，最小间隔由33.333ms变为25ms，让正常30Hz输入及其轻微抖动更容易通过。相机仍是30Hz，不生成额外图像，不伪造时间戳，也不代表每秒必定融合40帧。

结果：同轮对照中，默认官方组件的深度融合由18.23Hz升至29.71Hz，颜色由17.96Hz升至29.85Hz。ESDF调用仍约24.58Hz；局部重建误差未明显改变。

## 2. 实际改动清单

| 文件 | 改动 | 原因 |
| --- | --- | --- |
| [profiles/realtime.yaml](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/profiles/realtime.yaml) | integrate_depth_rate_hz、integrate_color_rate_hz：30→40；增加语义注释 | 给真实30Hz输入留时间门限余量 |
| [test_launch_contract.py](/home/czz/perception_eye_in_hand/nvblox_in_hand/tests/test_launch_contract.py) | 默认档检查改为两路限频40，保留3cm/ESDF30/层5检查 | 核对最终合并配置契约 |
| README、OPERATIONS、PARAMETERS_AND_TOPICS | 区分相机30、融合限频40、ESDF目标30 | 避免把配置限频当作传感器输出或实测 |
| artifacts/fusion_margin_test_20261105 | 新增本轮采集、离线分析、实际参数、地图、截图和日志 | 保留可核查的测试证据 |

没有改官方C++/CUDA融合算法、shouldProcess、图像时间戳、手眼、TF、ESDF点筛选、体素大小、地图范围及衰减。reference档、历史兼容配置和CLI覆盖规则保留；显式加载旧30Hz YAML或传depth_hz:=30/color_hz:=30仍会恢复旧门限，应检查最终参数。

本次依据本地官方节点已有参数实施，不需要安装新依赖或修改workspaces。

## 3. 保持的测试条件

- Depth和RGB均640×480，目标30Hz；实际编码16UC1/rgb8，独立CameraInfo匹配，fx/fy约320.255，cx/cy=320/240。
- 0.03m体素，3m最大融合距离，static_tsdf、机械臂图像时间TF、原UR7e+D435和高墙场景。
- 3D ESDF目标30Hz，10ms原生tick；Mesh/层实际公共发布目标5Hz；诊断点云目标5Hz。
- 关闭衰减/范围清图。没有环绕；沿用之前观察姿态与wrist_1 ±0.06rad小幅转动。
- 独立ROS域129、Gazebo端口11459。先到观察姿态，每轮新建地图，静态10秒、往复15秒并等待返回、停稳8秒。运动统计区间含姿态过渡，约22秒。

三轮顺序：baseline3d（官方组件，CLI覆盖融合限频30）→official3d（官方组件，默认融合限频40）→diagnostic3d（官方核心加原生体素导出，融合限频40）。

实际参数dump确认每轮限频、输入、3cm和ESDF模式。三轮来自同一个实时仿真场景，但不是同一录制bag；时间戳、噪声和运行顺序影响仍存在，因此是短时对照验证，不是完整随机化因果基准。

## 4. 实际频率与帧利用情况

### 4.1 统一口径结果

沿用官方save_timings中的 `ros/depth/integrate`、`ros/color/integrate`、`ros/esdf` 累计样本数，在静态末与运动末快照间差分除墙钟间隔。端点近似包含短暂统计保存间隙；不是CUDA事件测得的GPU完成率，也不保证每次ESDF有新块变化。

| 指标 | 官方原门限30 | 官方优化门限40 | 优化门限40＋诊断点云 |
| --- | ---: | ---: | ---: |
| 实际深度融合 | **18.23Hz** | **29.71Hz** | 29.72Hz |
| 实际颜色融合 | **17.96Hz** | **29.85Hz** | 29.86Hz |
| 3D ESDF更新调用 | 24.42Hz | **24.58Hz** | 24.54Hz |
| Mesh消息接收 | 4.75Hz | 4.74Hz | 4.75Hz |
| 项目3D点云消息 | 不发布 | 不发布 | 4.87Hz（目标5） |

这说明此前低融合率主要可由不合适的同频时间门限解释。优化恢复的是更多输入进入融合，不是把一次积分核函数重新加速，也不是提高相机采样率。

![同场景对照数据](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/test_summary.png)

### 4.2 从回调到融合的官方计数

| 官方计数差分 | 门限30 | 门限40 |
| --- | ---: | ---: |
| 深度图像回调 | 653 | 655 |
| 深度进入处理函数 | 652 | 656 |
| 深度进入积分 | 401 | 654 |
| 颜色图像回调 | 650 | 657 |
| 颜色进入处理函数 | 649 | 658 |
| 颜色进入积分 | 395 | 657 |

融合/回调比值约从61%提升至接近100%。快照不是同一时刻的逐帧关联日志，回调与处理边界会出现约1帧差异，不能把这些比值作为严格“零丢帧”证明。

仅用本轮接收时间戳离线重放门限，baseline深度约18.25Hz，优化轮约29.79Hz，与实测方向和幅度一致。这是门限模拟，不代替实际积分计数。

诊断metrics的部分静态短窗值可能受消息接收排队/边界影响，高于当时输入频率；正式比较采用上述官方timer区间计数，保留原始样本但不把短窗峰值当作稳定吞吐。

## 5. 延迟与数据新鲜度

官方 `ros/esdf_integration` 统计ESDF更新时最新已融合深度的数据年龄。本轮日志中：

| 指标 | 门限30 | 门限40 |
| --- | ---: | ---: |
| 日志滚动均值范围 | 49–56ms | **39–46ms** |
| 各滚动均值窗口的中位数 | 53ms | **43ms** |

本次对照约减少10ms；这不是逐帧延迟P50/P95，也不包含屏幕扫描和RViz绘制。未验证所有场景都能减少相同比例。

诊断轮消息到达时按最近/clock估计数据年龄：中位约59ms、P95约90ms。它受异步时钟采样和5Hz输出方式影响，不是屏幕端到端延迟。

运动区间官方主机timer：深度积分平均约0.325ms、颜色约0.341ms、ESDF调用约0.602ms。GPU没有用独立CUDA事件重新计时，不以这些主机值宣称完整GPU完成耗时。

## 6. 三维场和点云没有通过删点换取融合率

官方优化轮的有界3D服务，在与baseline相同的base_link AABB（最小角-1/-1/-0.2m、尺寸2/2/1.4m）返回221,952个值：

| 内容 | 原门限30 | 优化门限40 |
| --- | ---: | ---: |
| known | 17,369 | 17,377 |
| unknown | 204,583 | 204,575 |
| 有效正距离 | 10,782 | 10,781 |
| 有效负距离 | 3,502 | 3,510 |
| 有效零距离 | 3,085 | 3,086 |

这是指定AABB内结果，不能解释成全场景完整率；unknown仍保留，未强行填成自由。查询在频率窗口之后执行，不把查询耗时算进上面的融合率。

优化后的三维诊断停稳快照为18,178点：outside 10,893、inside 7,285、site 3,321（site为状态子集）。字段、原生体素中心、符号和有限值检查通过。点云仍是原生observed集合，最大距离±2m饱和，不是精确Mesh表面或带完整unknown契约的规划地图。

官方默认后端只加载官方组件，不产生项目点云或metrics；诊断后端是同一核心的附加检查分支，没有同时运行第二套地图。

## 7. 重建质量与仍未解决的问题

在配置墙体邻域、使用保存的world←base_link变换比较Mesh顶点到理想盒表面的距离，baseline与优化官方轮均约：

- 499个墙邻域顶点，绝对距离中位2.20mm、均值8.55mm；
- P95约 **37.60mm**；
- 距离≤20mm的顶点比例约82.6%；
- 最终Mesh均约2,498顶点、3,558三角形。

本轮局部指标未出现明显退化，也没有证明几何精度提高。该指标受网格采样和墙邻域定义影响，不是全场景或均匀面积误差。

以下优化尚未实施：

1. ESDF按目标相位调度：当前10ms tick和严格时间门限仍使30Hz请求约落到25Hz；本轮没有改它。
2. 真实采样时刻核验：图像时间戳生成方式和机械臂TF链未改，不能由内部一致性推断外部标定完全正确。
3. 机器人自过滤、历史表面清理与残影治理：本轮没有实现，也没有量化残影消退时间。
4. 队列最新帧策略、GPU异步/变化块导出：未重写；大地图仍需另测。

因此应把本轮结果表述为“成功融合率接近30Hz、ESDF输入更及时”，不能表述为“完整3D ESDF已稳定30Hz、精度和残影问题都解决”。

## 8. 真实窗口

优化后官方三维主体的原生彩色Mesh：

![优化后的官方3D Mesh窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/official3d_stopped.png)

优化后的可选三维原生ESDF点云：

![优化后的三维ESDF诊断窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/diagnostic3d_stopped.png)

截图来自本轮实际RViz后台窗口，不是生成图；视角存在交互差异，不作像素级精度对照。左上RGB仍可见机器人本体，说明自过滤问题还在。RViz约31fps只代表显示绘制。

## 9. 完整操作指令

先在旧终端Ctrl+C停止同域同端口的旧场景/mapper。只关闭窗口可能留下后台Gazebo。默认端口11457，检查可用 `ss -ltnp 'sport = :11457'`，不要盲目批量杀进程。

当前默认官方三维运行：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./run.sh gazebo_gui:=true camera_width:=640 camera_height:=480 esdf_mode:=3d
```

默认profile=realtime已经是两路融合限频40，无需额外参数。若需明确写出：

```bash
./run.sh gazebo_gui:=true esdf_mode:=3d depth_hz:=40 color_hz:=40
```

查看三维ESDF诊断点云，停止前一轮mapper后运行：

```bash
./run.sh gazebo_gui:=true mapper_backend:=diagnostic esdf_mode:=3d \
  esdf_view_mode:=observed esdf_view_hz:=5
```

场景已在同一环境运行且输入已是640×480时，加start_scene:=false，避免重复场景。不要同时启动两套mapper。真实D435使用start_scene:=false use_sim_time:=false launch_camera:=true；本轮未进行实机验收。

新终端核对实际参数和原生输出：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
source ./env.sh
ros2 component list
ros2 param get /nvblox_node integrate_depth_rate_hz
ros2 param get /nvblox_node integrate_color_rate_hz
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/color/image_raw --wall-time
ros2 topic hz /nvblox_node/mesh --wall-time
# 仅diagnostic后端：
ros2 topic echo /nvblox_node/metrics --once
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

官方后端更新/融合率看原生控制台或save_rates；不要拿输入topic hz或Mesh hz代替融合率。官方3d模式没有二维切片消息。

复现旧门限，只在单独一轮启动时覆盖：

```bash
./run.sh depth_hz:=30 color_hz:=30 esdf_mode:=3d
```

本地代码中rate=0会关闭相应处理，不代表无限频率。更高限频不保证更高输入或吞吐，负载增加时仍需监测队列和数据年龄。完整地图保存操作见 [操作说明](OPERATIONS.md)。

## 10. 验证及原始记录

- 两包构建、20项Python检查及既有C++检查通过；GPU导出算法本轮未改，未重复独立GPU单测，实际三维点云格式与状态检查通过。
- 三轮实际参数、双CameraInfo、组件装载、Mesh/地图保存与3D服务成功；测量期间关键进程持续运行。
- 主动停止部分launch时日志出现robot_pose的-2和ROS adapter shutdown消息，发生在测量/地图保存完成之后；本轮未修复此退出处理，不能与测量期间的mapper失效混为一谈。
- 本轮启动的Gazebo/mapper/RViz已停止，独立11459端口释放，并做了宿主只读检查。只测realtime档、640×480和5Hz诊断显示，未验收30Hz诊断显示、实机或大地图。

原始证据：

- [完整结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/results.json)、[统一计数与帧利用分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/analysis.json)、[质量指标](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/quality_metrics.json)。
- [baseline实际参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/baseline3d_runtime_parameters.yaml)、[优化实际参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/official3d_runtime_parameters.yaml)、[官方timer快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/official3d_wrist_motion_timings.txt)。
- [测试过程](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/measurement.log)、[原始样本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/samples.json)、[采集脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/run_measurement.py)。
- [构建日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/build.log)、[离线检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/offline_tests.log)、[TF审计](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/fusion_margin_test_20261105/robot_state_audit.json)。
