# 障碍物上方轻微转动相机：重建与三维ESDF测试

日期：2026-11-05。工程为 `/home/czz/perception_eye_in_hand/nvblox_in_hand`。

## 1. 本次做了什么

按要求在真实Gazebo仿真中测试：机械臂先平滑移动到之前的观察姿态，相机从障碍物上方斜向下观察，再缓慢小幅转动并返回原姿态。没有执行绕障碍物环绕，没有更改官方Mapper主体、融合精度或地图更新算法。

本次新增内容仅为独立测试目录中的采集、分析脚本和本文档；继续使用现有run.sh/诊断后端、官方TSDF/Color/ESDF/Mesh实现与机械臂图像时刻TF。所有动作只针对本项目Gazebo，采用独立domain129和端口11459。

## 2. 测试条件与动作

| 项目 | 配置 |
| --- | --- |
| 相机输入 | 深度16UC1、颜色rgb8，640×480，目标30Hz |
| 体素/最大融合距离 | 0.03m / 3m |
| 感知 | static_tsdf；原生3D ESDF；RGB融合；Mesh/层开启 |
| 深度/颜色融合门限 | 40Hz / 40Hz |
| ESDF/点云显示门限 | 33.4Hz / 33.4Hz |
| Mesh/层发布门限 | 5Hz |
| 历史处理 | TSDF衰减、半径清图关闭 |
| 三维配色 | 固定−0.09～0.24m距离彩虹色，Points/3像素 |
| 动作 | wrist_1围绕观察姿态±0.04rad；sin(0.6t)，转动阶段目标25秒 |
| 顺序 | 静止约10秒 → 保存转动前网格 → 轻微转动并返回 → 停止约8秒 → 保存最终网格/地图 |

观察姿态关节角为 `[0, -1.57, 1.1, -0.90879335, -3.21580312, -0.69520875]` rad。进入和返回采用平滑插值，最大指令速度约0.2rad/s；本次小幅周期运动的最大名义关节角速度约0.024rad/s。进入姿态及返回耗时不包含在25秒的周期运动参数内。

不是仅凭RViz视角判断相机位于上方：采集脚本读取world→camera_depth_optical_frame，要求相机z高于墙顶0.5m且光轴z分量为负后才继续；运动过程中继续记录相机位置与光轴。

## 3. 测量与验证方法

- 输入尺寸、CameraInfo/K/P、配置参数通过运行时核验；确认只有一个三维点云发布器。
- 保存转动前与停止后的原生ESDF体素快照，核验24字节点布局、原生体素中心、observed/inside和有符号距离。
- 核心频率取现有官方save_timings中ros/depth/integrate、ros/color/integrate、ros/esdf的计数差除以墙钟区间；点云/Mesh频率按真实接收时间计算。不是将配置门限或RViz FPS当作实测。
- 保存转动前后的Mesh，将顶点通过实测world←base_link转换到世界系，在已知两面墙邻域计算到墙体盒表面的绝对距离；附带参考墙面采样点在3cm内接近Mesh顶点的比例。
- 保存静止、转动和停止后的真实RViz窗口、RGB输入，以及最后单独Mesh显示窗口。

几何指标只针对墙体附近，按Mesh顶点加权，不代表全场景面积加权误差。参考表面接近率包含相机看不到的背面，不能当作真实可见面覆盖率。球体、机器人自体和地面未纳入该墙体精度统计。

保存转动前Mesh发生在静止与运动阶段之间；核心计数差区间包含这个短暂间隔。相机TF采样为约10Hz的latest位姿，用于确认运动范围，不替代逐图像时间戳的融合TF验证。

## 4. 实测结果

### 确认相机确实在障碍物上方

初始相机世界位置约(0.45475, 0.04118, 0.68163)m，光轴为(−0.1021, −0.0767, −0.9918)，与竖直向下约7.34°夹角。场景墙顶为0.5m，初始相机高出墙顶约18.2cm。

转动期间相机高度0.67751～0.68589m，水平位置x约0.45133～0.45800m、y约0.04116～0.04124m；光轴世界z分量−0.99510～−0.98694，始终朝下。相对运动阶段起始光轴的最大角偏移约2.285°，符合轻微转动而非环绕的要求。

周期转动设25秒，加上脚本检查/靠近/返回，实际运动阶段墙钟31.92秒。机械臂返回原观察姿态，停止阶段相机位置稳定。

### 实际频率与输入年龄

| 指标 | 本次结果 |
| --- | ---: |
| 深度实际融合 | 29.79Hz |
| 颜色实际融合 | 29.73Hz |
| ESDF更新入口调用 | 32.69Hz |
| 运动阶段三维ESDF点云发布 | 29.64Hz |
| 运动阶段Mesh发布 | 4.75Hz |
| 点云接收间隔中位数/P95 | 31.47 / 39.89ms |
| 点云关联输入年龄P50/P95 | 51 / 79ms |
| 仿真实时因子 | 0.9985 |

关联输入年龄是接收时最近/clock减点云关联深度时间戳，受时钟接收和回调排队影响，不是屏幕端到端延迟或GPU完成时间。32.69Hz更新调用、29.64Hz发布并不代表每秒相同数量的全新地图；本轮没有改变已知的重复时间戳/空调用判定逻辑。

### 转动前后重建与ESDF变化

| 指标 | 转动前 | 转动并停止后 |
| --- | ---: | ---: |
| 已观测ESDF点 | 14747 | 16997 |
| inside / outside | 5835 / 8912 | 6809 / 10188 |
| Mesh顶点 / 三角形 | 2124 / 3010 | 2362 / 3359 |
| 墙体邻域顶点误差中位数 | 2.19mm | 2.19mm |
| 墙体邻域顶点误差P95 | 36.97mm | 37.04mm |
| cube1参考表面接近率（3cm） | 39.49% | 40.09% |
| cube2参考表面接近率（3cm） | 35.61% | 36.26% |

有效ESDF点增加2250个，约15.26%；Mesh也有所扩展。但墙面误差基本不变，参考表面接近率只提升约0.60/0.65个百分点。这支持“小幅转动补充了部分观测”，不能支持“精度明显提升”或“已完成整个障碍物重建”。新点还可能包含地面、自体及已有几何附近的距离体素，并非2250个新增障碍物表面点。

最终同一个有界原生查询共221952格，known16422、unknown205530。该查询包含大量未观测远处区域，不能直接把known比例当作障碍物重建完成率；其范围与全层导出不同，所以known数与16997个点不必相等。没有转动前同范围查询，不能据此推断unknown减少了多少。

## 5. 实际截图与质量判断

### 转动前的三维ESDF

![初始静止ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105/overhead_static.png)

### 轻微转动过程

![转动过程ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105/overhead_wrist_motion.png)

![转动过程RGB输入](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105/overhead_wrist_motion_rgb.png)

### 转动结束后的ESDF

![最终ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105/overhead_stopped.png)

### 最终彩色Mesh

![真实RViz Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105/mesh_after.png)

这些是真实运行窗口，不是效果示意图。三维点云仍是距离体素，规则排列不表示重新造了查询采样网格；可见表面应看Mesh。

Mesh窗口中地面和机械臂附近的重建占据较大部分，墙面存在缺失及不平整，没有形成完整、干净的障碍物模型。相机实际RGB中仍有机械臂自体进入视场；这提示后续需检查自体屏蔽及有效工作区域，而不是靠增加点数或换色判定重建变好。截图本身不能区分每一处伪影究竟来自自体、位姿、深度或历史融合，仍需针对性对照实验。

这里只小幅改变俯视角，能看到的侧面变化有限；墙角和背面仍可能被遮挡。没有通过大范围环绕来掩盖这个局限。

测试脚本语法检查通过，运行时位姿、输入、发布器与原生点格式核验通过；本轮没有修改感知源码，也没有重新构建或重复运行之前已通过的离线单测。


## 6. 如何运行同样的动作

本次已执行完毕。以下是供再次运行的指令，不要在实机上执行测试运动脚本。

### 方式A：手动运行，便于观看

先停止占用domain129/端口11459的旧测试，且不要与其他关节控制程序同时运行。

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
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/view_sweep_isolated.py --check-only
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/view_sweep_isolated.py --seconds 0
```

先观察约10秒，让静态初始图积累，再执行：

```bash
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/view_sweep_isolated.py --seconds 25 --amplitude 0.04
```

这个独立脚本与原view_sweep.py的区别为domain129保护、默认幅度0.04rad和更慢的0.6rad/s正弦相位；姿态与官方感知链不变。它不会绕障碍物环绕，也不会重置/清空地图。

终端C，加载同样环境后分别检查：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
ros2 topic hz /nvblox_node/esdf_points --wall-time
# Ctrl+C后再查看其他项
ros2 topic hz /nvblox_node/mesh --wall-time
ros2 topic echo /nvblox_node/metrics --once
```

RViz可通过Move Camera+Orbit拖动观察。查看ESDF时关闭Mesh以免遮挡；查看表面则启用Official incremental RGB Mesh并关闭Diagnostic ESDF。切换显示不会改变体素精度。

### 方式B：自动采集与截图

必须先停止方式A的场景与Mapper。脚本会自行启动场景、Mapper、RViz及机械臂动作，并在结束后清理自己启动的进程。

重新运行前先备份结果目录；脚本会覆盖其中同名的结果、日志和截图。DISPLAY需符合当前图形桌面，本次使用:1。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/run_measurement.py
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/analyze_test.py
/usr/bin/python3 artifacts/overhead_reconstruction_test_20261105/analyze_quality.py
```

截图通过XComposite捕获本轮RViz窗口，不抓取整个桌面；桌面显示/X11不可用时截图步骤会失败，不会以旧图冒充。

## 7. 原始证据与后续分析

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/overhead_reconstruction_test_20261105`。

| 文件 | 内容 |
| --- | --- |
| results.json、samples.json | 测试阶段、命令、配置与原始接收样本 |
| analysis.json | 真实频率、点数和运动阶段相机位置/光轴范围 |
| camera_trajectory.json | 相机world位姿与光轴采样 |
| mesh_before.ply、mesh_overhead.ply | 转动前和停止后的原生网格 |
| quality_metrics.json | 墙体局部误差及参考表面接近率 |
| before_esdf.npy、overhead_esdf.npy | 原生ESDF体素快照，含距离和inside/site标志 |
| overhead_native_esdf.npy | 频率测试外的原生有界距离查询 |
| overhead_*.png、mesh_after.png | 实际RViz及相机RGB截图 |
| *_rates.txt、*_timings.txt、*_runtime_parameters.yaml | 原有官方统计、运行时参数 |
| source_manifest.json | 本次测试源码及配置哈希 |

下一步如果要提高完整性，应在保持图像时刻位姿正确的基础上增加适量倾角/不同朝向的可见面；本次轻转是局部视角补充，不保证墙角、遮挡背面全部被看到。若出现机器人自体重建或残影，需继续分析自体遮罩、时间同步和历史清理，不能用换色或点数增加代替质量验证。

更多操作与参数见[逐终端测试指南](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_COMMAND_TEST_GUIDE_20261105.md)和[三维发布及配色说明](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)。
