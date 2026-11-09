# 障碍物上方转动相机：第二轮重建效果测试

日期：2026-11-05。工程：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

## 1. 本次测试与上次的区别

本次实际重新启动Gazebo、官方Mapper感知链和RViz，创建新地图；相机在障碍物上方朝下观察，缓慢转动后返回原姿态，没有绕障碍物环绕。上一轮为wrist_1±0.04rad，本轮增加到±0.08rad（约±4.58°），使观察方向变化更明显。正弦相位速度从0.6降为0.4rad/s，周期约15.7秒，转动目标30秒。这里0.4rad/s是正弦相位速度，实际关节最大名义角速度为0.032rad/s。

未改感知生产代码或分辨率，没有通过降低精度提高频率；新增内容仅为独立仿真测试脚本、原始证据和本文档。机器人先平滑进入之前的观察姿态，静止约10秒，保存初始网格，再转动并返回，停止约8秒后保存最终结果。

## 2. 实际相机位姿与输入配置

| 项目 | 配置或测量 |
| --- | --- |
| 输入 | D435仿真深度16UC1、颜色rgb8，640×480，目标30Hz |
| 体素/最大融合距离 | 0.03m / 3m |
| 融合/ESDF | static_tsdf，官方TSDF/Color/Mesh及原生3D ESDF；机械臂图像时刻TF |
| 频率门限 | 深度/颜色40Hz；ESDF/诊断显示33.4Hz；Mesh/地图层5Hz |
| 历史清理 | TSDF衰减与半径清图关闭 |
| 点云配色 | 固定−0.09～0.24m距离彩虹色，Points/3像素 |
| 实际运动阶段时长 | 36.89秒，含脚本检查/进入/返回 |
| 相机世界高度 | 0.6735～0.6903m |
| 墙顶世界高度 | 0.5m |
| 光轴世界z分量 | -0.9968～-0.9805，负数表示朝下 |
| 实测光轴方向总跨度 | 约9.14°，由运动期间采样方向的最大夹角计算 |

采集前通过world→camera_depth_optical_frame验证相机高于墙顶且朝下；上表又用运动期间约10Hz的TF采样确认全过程高度和方向。不是只转动RViz观察相机，也不是仅改变显示视角。

## 3. 实时性结果

| 指标 | 实测 |
| --- | ---: |
| 实际深度融合 | 29.82Hz |
| 实际颜色融合 | 29.79Hz |
| ESDF更新入口调用 | 32.92Hz |
| 三维点云接收/发布 | 29.37Hz |
| Mesh接收/发布 | 4.75Hz |
| 点云接收间隔P50/P95 | 31.77/40.40ms |
| 点云关联输入年龄P50/P95 | 57/86ms |
| 仿真实时因子 | 0.9987 |

实际融合/更新率来自已有官方save_timings累计计数差除以墙钟区间；该区间包含静止末端到运动开始之间保存初始Mesh的短暂间隔。发布率来自消息接收时间，RViz右下角FPS没有用作地图频率。每轮只有一个项目三维点云发布器。

关联输入年龄是最近收到的仿真时钟减点云关联深度时间戳，受时钟和回调排队影响；不是屏幕端到端延迟，也没有使用CUDA事件测量GPU完成时间。更新调用和点云消息可能关联相同输入时间戳，不能等同于每秒全新地图数。

## 4. 转动前后重建变化

| 指标 | 转动前 | 转动并停止后 |
| --- | ---: | ---: |
| 原生observed ESDF点 | 14747 | 19394 |
| outside体素 | 8912 | 11659 |
| inside体素 | 5835 | 7735 |
| site体素 | 2730 | 3548 |
| Mesh顶点 | 2124 | 2622 |
| Mesh三角形 | 3010 | 3742 |
| 墙体邻域误差中位数 | 2.19mm | 2.20mm |
| 墙体邻域误差P95 | 36.97mm | 37.60mm |
| cube1参考表面接近率（3cm） | 39.49% | 40.42% |
| cube2参考表面接近率（3cm） | 35.61% | 37.24% |

有效ESDF点增加4647个（31.51%）。说明视角变化让已观测距离体素范围扩大；新增点也包含地面、自体及已有几何附近的距离体素，不等于相同数量的新增障碍物表面点。site与inside/outside并非互斥三分类。

墙体误差按已知场景两个墙体盒的表面计算，只统计墙体邻域网格顶点，并使用实测world←base_link转换。它是局部顶点加权误差，不是全场景均匀表面积精度；参考表面接近率包含背面，不是严格可见面覆盖率。有效点、网格规模和接近率的增加也不能单独证明几何精度变好。

最终有界原生查询共221952格，known=18319，unknown=203633。查询范围与全层导出不同，数量不必一致；该大范围包含未观测远处区域，不能用它的known比例当作障碍物重建完成率。查询在频率采样之后执行，未混入上述实时性统计。

## 5. 真实截图

以下均为本次运行的实际RViz窗口或实际相机输入，没有用示意图替代。截图取景/显示开关可能受窗口交互影响，前后量化比较采用保存的网格和体素，而非逐像素对比。

### 转动前ESDF

![转动前ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105/overhead_static.png)

### 转动中的ESDF

![转动中的ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105/overhead_wrist_motion.png)

### 转动时相机RGB输入

![转动时相机RGB输入](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105/overhead_wrist_motion_rgb.png)

### 停止后的三维ESDF

![停止后的三维ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105/overhead_stopped.png)

### 最终Mesh表面重建

![最终Mesh表面重建](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105/mesh_after.png)

## 5.1 结果判断与不足

本次有效ESDF范围与网格规模增加，两面墙参考表面接近率分别从39.49%到40.42%、35.61%到37.24%；变化有限。墙面P95误差从36.97到37.60mm，中位数约2.2mm，不能宣称精度提升。转动补充观测和几何误差改善是两个不同指标。

最终Mesh仍有缺口、局部不平整和机械臂附近的额外重建，整体还不是完整干净的障碍物模型。相机RGB中能看到机器人自身，需后续通过自体遮罩对照确认其影响；仅凭截图不能把全部缺陷归因于自体，也需检查位姿、深度、历史融合和实际可见范围。

增大俯视转动幅度仍只能看到有限侧面，不能自动观测墙背面与被遮挡墙角。下一步可在保持相机位于上方的前提下补充少量不同俯仰方向，同时单独评估图像时刻TF、自体屏蔽和残影清理。此次不通过降低体素精度或显示过滤掩盖这些问题。

## 6. 手动复现命令

先停止占用domain129/端口11459的旧测试，不要同时运行多个Mapper或关节控制脚本。下列运动指令仅用于本项目Gazebo，不在实机上执行。

终端A：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
./run_esdf.sh gazebo_gui:=true
```

终端B：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
./check_sim.sh
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/view_sweep_isolated.py --check-only
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/view_sweep_isolated.py --seconds 0
```

静止观察约10秒，再执行上方转动：

```bash
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/view_sweep_isolated.py --seconds 30 --amplitude 0.08
```

该脚本保持world相机在原观察姿态附近，缓慢转动后返回，不做环绕。转动期间可在第三终端加载同样环境并逐项检查：

```bash
ros2 topic hz /nvblox_node/esdf_points --wall-time
# Ctrl+C后执行其他项
ros2 topic hz /nvblox_node/mesh --wall-time
ros2 topic echo /nvblox_node/metrics --once
```

## 7. 自动复现与原始证据

停止手动测试后才能运行自动采集。重新运行前备份本次artifact目录，自动脚本会覆盖同名结果；DISPLAY需匹配当前图形桌面。本次使用:1。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/run_measurement.py
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/analyze_test.py
/usr/bin/python3 artifacts/overhead_rotation_reconstruction_v2_20261105/analyze_quality.py
```

原始目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_rotation_reconstruction_v2_20261105`。

- results.json、samples.json：运行阶段、命令和原始消息记录。
- camera_trajectory.json：相机位置、光轴与采样时刻。
- analysis.json、quality_metrics.json：实测频率、体素统计和局部几何指标。
- mesh_before.ply、mesh_overhead.ply：转动前后网格；map_overhead.nvblx：原生地图。
- before_esdf.npy、overhead_esdf.npy：原生ESDF体素快照。
- *_rates.txt、*_timings.txt、*_runtime_parameters.yaml：原有官方统计与实际参数。
- 各阶段png及RGB图：真实窗口与输入。
- source_manifest.json：测试代码、配置和报告哈希。

本次脚本语法检查、运行时输入/相机位姿/发布器/原生点格式核验通过。感知生产代码没有改动，因此没有重新构建或重复运行之前已通过的离线测试。测试脚本结束后清理自己启动的Gazebo、Mapper和RViz。

[上一轮±2.3°测试](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/OVERHEAD_RECONSTRUCTION_TEST_20261105.md) · [完整逐终端测试指南](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_COMMAND_TEST_GUIDE_20261105.md)
