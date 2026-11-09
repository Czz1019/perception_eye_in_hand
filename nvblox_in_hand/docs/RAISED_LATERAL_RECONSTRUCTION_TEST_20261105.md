# 抬高相机、左右摆动的重建测试

日期：2026-11-05。工程：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

## 1. 本次动作修改

之前只摆动wrist_1，光轴和相机位置会产生前后、上下耦合变化。本次不再采用单关节摆动，而是通过本项目真实URDF逆运动学求解六关节协调轨迹：相机抬高到约0.82m，世界X（前后）与Z（高度）保持固定目标，沿世界Y（机器人左右方向）平移±0.08m。光轴保持原来朝下的方向；绕光轴的滚转不强制固定，以便选择可达且无自碰撞的逆解分支。因此动作是高位左右扫描，没有前后扫动目标，也没有环绕。

先用现有FCL/URDF检查关节限位、41个左右轨迹采样姿态、相邻段和初始到高位的进入路径，筛掉不可达/自碰撞候选后才执行仿真。该检查复用项目既有Guard/CollisionOracle，要求环境间隙大于2cm、自碰撞间隙大于5mm，不代表对真实硬件的安全认证。

左右周期为sin(0.2t)，目标持续40秒；最大目标左右速度0.016m/s，离线采样估计最大关节速度约0.060rad/s。进入高位和返回中心平滑执行，进入段最大目标关节速度约0.2rad/s。运动开始前检查仿真关节反馈是否接近已验证初始姿态或高位中心，避免从任意姿态直接执行轨迹。

本次只新增独立测试脚本和报告，没有改动生产感知C++、官方Mapper、机械臂模型或相机输入。测试结束返回高位中心，而不是降回原来的0.68m姿态。

## 2. 实际轨迹核对

| 指标 | 实测 |
| --- | ---: |
| 原观察位姿高度（URDF FK） | 0.6816m |
| 新观察中心高度（仿真TF） | 0.8200m |
| 高度增加约 | 13.8cm |
| 运动阶段世界X | 0.40000～0.40198m |
| 运动阶段世界Y | -0.03879～0.12121m |
| 运动阶段世界Z | 0.81998～0.82005m |
| 左右总跨度ΔY | 16.00cm |
| 前后残余跨度ΔX | 1.977mm |
| 上下残余跨度ΔZ | 0.072mm |
| 光轴相对阶段起始方向最大偏移 | 0.0316° |
| 光轴世界Z范围 | -0.991884～-0.991814 |
| 运动阶段墙钟时长 | 46.90秒，含检查/返回 |

这些来自约10Hz的实际TF采样，不是仅展示规划轨迹，也不是只转动RViz观察窗口。残余前后/上下位移来自IK采样插值与仿真跟踪，并非刻意前后晃动。相机始终高于墙顶0.5m并朝下。世界Y的左右不一定等同于随意旋转后RViz屏幕的水平方向。

## 3. 输入与实时性

输入640×480 RGB-D、目标30Hz；体素0.03m，范围3m，static_tsdf；机械臂图像时刻TF；融合门限40Hz，ESDF/点云显示门限33.4Hz，Mesh/地图层5Hz；TSDF衰减和半径清图关闭。三维颜色仍为−0.09～0.24m距离彩虹色，Points/3像素。

| 指标 | 实测 |
| --- | ---: |
| 深度融合 | 29.85Hz |
| 颜色融合 | 29.78Hz |
| ESDF更新调用 | 32.78Hz |
| 三维ESDF点云发布 | 29.22Hz |
| Mesh发布 | 4.74Hz |
| 点云关联输入年龄P50/P95 | 51/79ms |
| 仿真实时因子 | 0.9988 |

融合/更新采用现有官方save_timings计数差除以墙钟时间，区间包括保存初始Mesh的短暂间隔；发布采用真实消息接收时间。输入年龄为最近仿真时钟减点云关联深度时间戳，受时钟接收/回调排队影响，不是屏幕端到端延迟或GPU完成时间。ESDF调用和点云发布可能重复关联输入，不等于每秒全新地图数。

## 4. 重建结果

这里“扫描前”是已经抬高后静止约10秒的地图，不是之前低位测试的地图，随后左右扫描并返回，停止约8秒保存结果。

| 指标 | 高位静止、左右扫描前 | 左右扫描后 |
| --- | ---: | ---: |
| observed ESDF体素点 | 24184 | 33352 |
| inside / outside | 8386 / 15798 | 11931 / 21421 |
| Mesh顶点 | 3006 | 4307 |
| Mesh三角形 | 4306 | 6156 |
| 墙体邻域误差中位数 | 2.29mm | 3.73mm |
| 墙体邻域误差P95 | 35.00mm | 49.94mm |
| cube1参考表面接近率（3cm） | 54.77% | 58.26% |
| cube2参考表面接近率（3cm） | 48.84% | 58.67% |

有效ESDF点增加9168个（37.91%）。点数包含地面、自体和已有几何附近的距离体素，不等同于新增障碍物表面数或完整性百分比。

墙体指标只在已知场景两个盒形墙体附近、按Mesh顶点计算，并通过实测world←base_link变换对齐。P95不是全场景均匀表面积误差；参考表面接近率包含背面，不是严格可见面覆盖率。抬高后姿态、输入视野与前几轮不同，不能把轮间误差变化单独归因于左右动作。

### 结果判断与仍存问题

这次动作已满足“抬高、主要左右晃动”：相机抬高约13.8cm，左右跨度16cm，前后残余约1.98mm、上下残余约0.072mm，朝下光轴偏移约0.032°。相比单关节俯仰摆动，运动方向更明确。

左右扫描补充了观测，两墙参考表面接近率分别增加约3.49和9.83个百分点，但局部误差P95从35.00增至49.94mm，几何误差尾部变大，不能宣称整体质量变好。新的可见面增多也改变了参与统计的顶点，不能只靠这个指标证明原有表面恶化；后续应对相同表面区域逐点比较，并分开静态与运动融合。

实际Mesh中仍有自体重建、局部不平整和缺口，不能当作完整干净障碍物模型。机械臂相机运动时机器人自身也会动，而本轮采用static_tsdf且没有自体遮罩，自体进入视场后可能作为静态几何积累，属于后续需要对照确认的残影来源；其他可能因素包括图像时间TF、动态跟踪误差和边缘深度。换动作本身不修复这些问题。

下一步宜在相同左右轨迹下做有/无机器人自体遮罩、静止逐点采集/连续移动采集对照，结合图像时刻位姿和相同表面真值误差，确定尾部误差来自哪里。当前不修改感知核心来隐藏这些缺陷。

## 5. 真实效果图

### 高位静止

![高位初始ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_static.png)

### 左右运动中

![左右扫描ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_wrist_motion.png)

![左右扫描RGB输入](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_wrist_motion_rgb.png)

### 扫描停止后

![扫描后ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/overhead_stopped.png)

![高位左右扫描最终Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/mesh_after.png)

均为实际Gazebo/RViz运行窗口或真实RGB输入；Mesh窗口关闭URDF机器人外观模型以减少遮挡，但未删除地图里重建出的自体。窗口取景可能受交互影响，量化结果来自保存的体素与网格，不做截图逐像素精度比较。

## 6. 复现操作

仅在本项目Gazebo中执行，停止占用domain129和端口11459的旧场景；不要与其他关节命令程序同时运行。以下脚本使用已经生成并检查的motion_plan.json，仅允许从对应初始状态或高位中心开始。

终端A：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
./run_esdf.sh gazebo_gui:=true
```

终端B，先抬高到中心，静止约10秒后左右扫描：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
./check_sim.sh
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/view_sweep_isolated.py --check-only
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/view_sweep_isolated.py --seconds 0
```

```bash
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/view_sweep_isolated.py --seconds 40 --amplitude 0.08
```

注意此独立脚本的amplitude单位是**米**，不是之前wrist_1脚本的弧度；最大0.08m，受已检查轨迹范围约束。动作结束保持高位中心。若脚本拒绝当前关节状态，应停止其他动作，从新场景初始状态重试，不删除状态保护。

终端C加载同样环境后按项观察：

```bash
ros2 topic hz /nvblox_node/esdf_points --wall-time
# Ctrl+C后
ros2 topic hz /nvblox_node/mesh --wall-time
ros2 topic echo /nvblox_node/metrics --once
```

自动复现必须先停止手动场景。备份本次结果目录，自动运行会覆盖同名证据；DISPLAY以实际桌面为准：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/run_measurement.py
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/analyze_test.py
/usr/bin/python3 artifacts/raised_lateral_scan_20261105/analyze_quality.py
```

## 7. 证据与验证

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105`。

- motion_plan.json、planning.log、plan_motion.py：URDF逆解、选定高位轨迹、碰撞检查与进入路径；失败候选只在离线搜索，未执行运动。
- results.json、samples.json、camera_trajectory.json：实际启动命令、消息和TF轨迹。
- analysis.json、quality_metrics.json、footprint.json：频率、位移范围与几何分析。
- mesh_before.ply、mesh_overhead.ply、before_esdf.npy、overhead_esdf.npy：重建前后网格与原生体素。
- *_rates.txt、*_timings.txt、*_runtime_parameters.yaml：原有官方统计与实际参数。
- png：实际窗口及RGB输入；source_manifest.json：源码、计划与配置哈希。

完成脚本语法、离线轨迹限位/碰撞检查，以及运行时输入、发布器、原生点格式和高位TF核验。本轮没有修改感知源码，没有重新构建或重复运行之前的离线单测。自动脚本结束后退出自己启动的Gazebo、Mapper和RViz。
