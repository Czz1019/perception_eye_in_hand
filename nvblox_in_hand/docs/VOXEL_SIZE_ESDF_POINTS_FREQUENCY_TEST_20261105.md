# voxel_size=0.02 / 0.03 / 0.05m时的三维ESDF点云更新频率

日期：2026-11-05。工程：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

本轮实际重启Mapper并加载三个体素大小，每档重复三次；主要测量话题为 **/nvblox_node/esdf_points**，不是把配置Hz或ESDF调用次数当点云更新率。

## 1. 固定条件

| 项目 | 本轮设置 |
| --- | --- |
| 变量 | voxel_size=0.02、0.03、0.05m，每档3次 |
| 输入 | D435仿真RGB/深度640×480，目标30Hz，16UC1/rgb8 |
| 融合门限 | 深度/颜色均40Hz |
| ESDF更新与显示门限 | 均33.4Hz，主tick约10ms |
| 工作范围 | 最大融合距离3m，ESDF最大距离2m；不构建固定ROI查询网格 |
| 位姿 | 已验证的约0.82m高位，保持静止；Mapper按图像时间查机械臂TF |
| 感知/导出 | static_tsdf，单个diagnostic后端；原生observed ESDF体素，inside/outside均保留 |
| Mesh/地图层 | 5Hz调度，相同记录订阅；三维RViz配置一致，3像素点 |
| 每轮 | 等待输入/Mesh/RViz、额外静止5秒、约20秒正式采样、保存体素/网格 |
| 顺序 | 0.02→0.03→0.05→0.05→0.02→0.03→0.03→0.05→0.02 |
| 本轮隔离环境 | ROS_DOMAIN_ID=134；GAZEBO_MASTER_URI=http://127.0.0.1:11563 |

起初11459端口被已有场景占用，第一次启动在采样前失败，单独保留attempt0_port_conflict并排除。正式九轮改用新域和新端口，没有关闭已有程序。背景场景可能占用同一机器资源，本测试不是独占硬件基准。

运行时逐轮dump参数并断言体素大小已生效；原生体素中心检查也按该轮voxel_size计算，避免仅改显示大小。每轮重新创建地图，避免将旧体素地图带入新分辨率。

## 2. 主要结果：esdf_points实际发布频率

下面频率为三轮均值；输入年龄/间隔分位数使用同档三轮正式采样消息合并计算。

| voxel_size | esdf_points发布Hz | 三轮范围 | ESDF点数范围/消息 | 新深度关联输出Hz | 输入年龄P50/P95 | 接收间隔P95 |
| --- | ---: | --- | --- | ---: | --- | ---: |
| 0.02m | **29.33** | 29.16～29.49Hz | 71,057～71,057 | 19.73 | 53/82ms | 40.73ms |
| 0.03m | **29.48** | 29.36～29.58Hz | 24,184～24,184 | 19.84 | 52/81ms | 40.16ms |
| 0.05m | **29.51** | 29.30～29.73Hz | 6,758～6,758 | 19.83 | 51/81ms | 40.16ms |

![体素大小与ESDF点云频率/点数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_voxel_size_comparison_20261105/voxel_size_esdf_frequency.png)

### 本轮结论

本轮不同体素下实际得到的/nvblox_node/esdf_points频率为：

- **voxel_size=0.02m：29.33Hz**，每条约71,057个点。
- **voxel_size=0.03m：29.48Hz**，每条约24,184个点。
- **voxel_size=0.05m：29.51Hz**，每条约6,758个点。

这是接收端实际统计的点云消息频率。三档在本场景短时测试中均能维持接近当前显示节拍的输出，不能据此认定0.05m和0.02m计算成本相同，也不能保证地图增大后仍相同。更粗体素主要降低点数/细节，未按点数减少比例提高Hz。

同时仍存在重复关联输入；表中的“新深度关联输出Hz”用于避免把重复消息当作新地图。未对几何精度做新真值验收，不以点数更多就宣称精度已改善。

### 点集规模与逻辑数据负载

| voxel_size | 最大测得点数/消息 | 纯点数据大小/消息 | 按实测频率估算单路点数据负载 |
| --- | ---: | ---: | ---: |
| 0.02m | 71,057 | 1.705MB | 50.02MB/s |
| 0.03m | 24,184 | 0.580MB | 17.11MB/s |
| 0.05m | 6,758 | 0.162MB | 4.79MB/s |

按每点24字节、十进制MB计算，不包含ROS/DDS协议、其他字段、额外订阅复制和图像流；这是逻辑点集负载估计，不是网卡实测带宽。


### 九轮逐轮值

| 顺序 | 体素/重复轮 | 点云发布Hz |  |  | 点数范围 |
| --- | --- | ---: | ---: | --- | --- |
| 1 | 0.02m / 1 | 29.34 |  |  | 71,057～71,057 |
| 2 | 0.03m / 1 | 29.51 |  |  | 24,184～24,184 |
| 3 | 0.05m / 1 | 29.50 |  |  | 6,758～6,758 |
| 4 | 0.05m / 2 | 29.73 |  |  | 6,758～6,758 |
| 5 | 0.02m / 2 | 29.49 |  |  | 71,057～71,057 |
| 6 | 0.03m / 2 | 29.36 |  |  | 24,184～24,184 |
| 7 | 0.03m / 3 | 29.58 |  |  | 24,184～24,184 |
| 8 | 0.05m / 3 | 29.30 |  |  | 6,758～6,758 |
| 9 | 0.02m / 3 | 29.16 |  |  | 71,057～71,057 |

## 3. 计算调用与融合，仅作辅助解释

| voxel_size | 深度融合Hz | 颜色融合Hz | ESDF更新调用Hz | 点云发布Hz：均值±样本标准差 | 相邻重复关联比例 |
| --- | ---: | ---: | ---: | --- | ---: |
| 0.02m | 29.75 | 29.83 | 32.60 | 29.33±0.17Hz | 32.7% |
| 0.03m | 29.87 | 29.89 | 32.71 | 29.48±0.12Hz | 32.7% |
| 0.05m | 29.78 | 29.85 | 32.88 | 29.51±0.21Hz | 32.8% |

“更新调用Hz”不等于GPU完成率；“点云发布Hz”包含重复关联同一深度帧的消息；“新深度关联输出Hz”按直接记录的header推进统计，不是几何变化率。输入年龄是回调时最近/clock减header的近似值，受时钟和排队影响，不含屏幕渲染，也不是独立CUDA计时。

## 4. voxel_size为什么影响点数，却不一定按比例影响Hz

1. 导出的是原生ESDF已观测体素中心，不是固定点阵。缩小体素通常增加体素和点数；本轮检查了坐标确实落在对应分辨率的原生中心。
2. 更新/显示仍受33.4Hz门限和10ms tick约束。只要计算与读回能跟上，减小体素主要增加负载，话题频率不一定明显下降；增大体素也不会自动突破显示门限。
3. 导出仍扫描已分配块、GPU压缩并同步读回，PointCloud2每点24字节。点数增加会增加扫描/传输/渲染需求，地图变大或背景负载增加时可能表现为频率下降或输入年龄尾部增大。
4. 点云消息可重复关联输入，不能仅依据不同体素下topic hz相近就证明真实信息更新能力完全相同。

本轮固定的是相机输入和最大融合距离，**不是所有算法物理尺度**。当前截断距离按4个体素、ESDF近表面site尺度按2个体素配置：

| voxel_size | TSDF截断尺度4voxel | ESDF近表面site尺度2voxel | 原生块边长8voxel |
| --- | ---: | ---: | ---: |
| 0.02m | 0.08m | 0.04m | 0.16m |
| 0.03m | 0.12m | 0.06m | 0.24m |
| 0.05m | 0.20m | 0.10m | 0.40m |

因此点数不应被要求精确遵循1/voxel_size³：已观测体积、截断带、块边界和可见表面分布都会变化。本轮回答的是现有配置体系下修改voxel_size的实际效果，不是将所有米制算法范围重新锁定后的纯分辨率成本实验。

0.05m能减少体素/颜色细节，10cm厚墙仅约2格；不能仅因频率不低就认定它重建更准确。0.02m更细，但不能修复自体污染、输入时间对应或残影。

## 5. 实际窗口截图

下面三图来自各体素首轮正式采样的真实三维ESDF窗口。点大小均3像素，距离色带相同，未通过放大方块或人为删点调整密度。窗口视角可能受交互影响，不能做逐像素质量比较。

### voxel_size=0.02m

![0.02m体素实际ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_voxel_size_comparison_20261105/voxel0p02_r1_measure.png)

### voxel_size=0.03m

![0.03m体素实际ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_voxel_size_comparison_20261105/voxel0p03_r1_measure.png)

### voxel_size=0.05m

![0.05m体素实际ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_voxel_size_comparison_20261105/voxel0p05_r1_measure.png)

## 6. 修改范围和运行指令

没有修改生产感知C++、官方CUDA算法或默认YAML。三个体素大小通过现有voxel_size CLI逐轮真实覆盖；日常默认仍为0.03m。新内容为独立采集/分析脚本及本文档；脚本完成语法核查，运行时核验了单发布器、输入与内参、各轮参数以及原生点格式。

手动查看三档时先停止旧场景与Mapper，下列三条分别执行，不要同时启动：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_esdf.sh gazebo_gui:=true voxel_size:=0.02 depth_hz:=40 color_hz:=40 esdf_hz:=33.4 esdf_view_hz:=33.4
```

```bash
./run_esdf.sh gazebo_gui:=true voxel_size:=0.03 depth_hz:=40 color_hz:=40 esdf_hz:=33.4 esdf_view_hz:=33.4
```

```bash
./run_esdf.sh gazebo_gui:=true voxel_size:=0.05 depth_hz:=40 color_hz:=40 esdf_hz:=33.4 esdf_view_hz:=33.4
```

监测终端加载同域环境后：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 param get /nvblox_node voxel_size
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

完整九轮复现需先停止占用domain134/端口11563的本轮旧实例。脚本包含进入0.82m高位动作，只用于本项目仿真；先备份本次artifact目录，以下会覆盖同名结果，DISPLAY以实际桌面为准：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
source ./env.sh
/usr/bin/python3 artifacts/esdf_voxel_size_comparison_20261105/run_measurement.py
/usr/bin/python3 artifacts/esdf_voxel_size_comparison_20261105/analyze_fusion_counts.py
/usr/bin/python3 artifacts/esdf_voxel_size_comparison_20261105/analyze_esdf_realtime.py
MPLCONFIGDIR=/tmp/esdf-voxel-mpl /usr/bin/python3 artifacts/esdf_voxel_size_comparison_20261105/plot_voxel_comparison.py
```

## 7. 原始证据与边界

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_voxel_size_comparison_20261105`。

- results.json、samples.json、cloud_records.json、depth_receipts.json：各轮配置、输入、直接header及消息到达记录。
- analysis.json、fusion_counts.json、esdf_realtime_comparison.csv：ESDF主要结果及原有官方调用计数。
- *_runtime_parameters.yaml：每轮实际生效voxel_size和其他参数。
- *_esdf.npy、mesh_*.ply：原生体素与网格快照；坐标检查使用该轮实际体素大小。
- *_timings.txt、*_rates.txt、mapping_*.log：官方原始统计。
- *_measure.png、*_rgb.png：真实显示与输入截图。
- attempt0_port_conflict：启动端口冲突的未采样尝试，排除在结果之外。
- source_manifest.json：脚本、配置与报告哈希。

每档仅三次短时静态测试，不能替代长时间增长地图、运动场景和实机测量。存在被保留的其他场景后台负载，尽管本轮域/端口隔离，GPU/CPU不是独占；交错顺序只能减小顺序影响，不能完全消除负载变化。
