# 工作台多视角重建：实际机械臂运动测试

2026-11-10。使用 `ur7e_nvblox_in_hand`，在新工作台 Gazebo 场景完成一次真实关节运动的扫描。这里的“真实”指仿真机器人实际关节反馈与相机位姿发生变化，没有连接或控制实机。保存了融合后的彩色 Mesh、原生地图、3D ESDF、14 组视点图像和实际运动轨迹；未修改融合算法。

## 1. 实际执行的运动

保持相机约 0.82 m 高度，沿世界 Y 方向左右移动，名义范围为初始位置 ±0.18 m；X 基本不变，不执行前后摆动或环绕。相机光轴约保持原向下方向，光轴滚转角自由调整以保持连续的腕部 IK 分支。

起点 → 正向端点 → 负向端点 → 起点，沿线短暂停留，每个采集位置停约 3 秒。命令关节速度上限 0.10 rad/s，实际运动记录约 84.30 秒。扫描前后返回相同初始观察姿态。

| 实际反馈 FK | 范围 |
|---|---|
| 相机 X | 0.399998–0.400017 m |
| 相机 Y | -0.138589–0.220996 m，总跨度约 35.96 cm |
| 相机 Z | 0.819975–0.819992 m |
| 轨迹反馈记录 | 810 个采样 |
| 保存图像视点 | 14 组 |

运动规划使用项目现有 `orbit_geometry.py` 的 URDF FK 和 FCL 检查逻辑；测试副本将碰撞检查二进制定位到当前 UR7e 工作区。场景器具按组使用保守包围盒，台面按 z=0 半空间处理，不将场景真值注入建图。规划连接段经过运动边界检查；810 个实际反馈姿态也通过离线检查：最小环境间距约 **60.24 mm**，最小非相邻连杆/相机自碰撞间距约 **14.13 mm**。这些检查仅适用于这条仿真路径和当前场景。

![实际相机 XYZ 轨迹](../artifacts/lab_multiview_20261110/actual_camera_motion.png)

实际视角序列动图（14 个停留采样点，每帧显示 0.5 秒；不是原速视频）：

![实际相机视角序列](../artifacts/lab_multiview_20261110/actual_camera_views.gif)

## 2. 与静止测试保持一致的设置

RGB/深度 640×480、2 cm 体素、4 体素即 8 cm 截断、启用颜色、3D ESDF、Mesh。融合输入仍来自模拟 D435 与图像时间对应机械臂 TF。没有降低分辨率、改变物体尺寸或导入真值表面来改善结果。

静止基线为上一轮 `lab_accuracy_20261110`。它的统计窗口为 45.25 秒，本轮为 109.65 秒，累计观测帧数不同，也不是同一份记录数据回放。因此这是相同场景和配置下的实际扫描对比，**不是等时长、等帧数的严格控制实验**。可以确认扫描补充了信息，但不能把全部改善单独归因于视角变化。

## 3. Mesh 完整性与精度

多视角 Mesh 导出 **32456 顶点、49472 个三角形**；静止基线为 15305 顶点、22095 个三角形。

### 完整器具外表面评估

为避免大面积台面掩盖器具缺失，对器具参考几何按三角形面积采样 80000 点，去除部件交叠形成的内部面、以及 z≤8 mm 的贴台面区域，剩余 **55241 个外表面采样点**。两份 Mesh 使用完全相同的采样点比较，不局限于初始相机能看到的正面。

| 完整器具外表面指标 | 静止 | 左右多视角 |
|---|---:|---:|
| 真值表面到 Mesh 最近距离均值 | 10.74 mm | **8.19 mm** |
| 最近距离 P95 | 34.01 mm | **24.32 mm** |
| 真值表面在 Mesh 5 mm 内的比例 | 39.03% | **44.86%** |
| 在 Mesh 10 mm 内的比例 | 61.95% | **70.10%** |
| 在 Mesh 20 mm 内的比例 | 82.82% | **91.12%** |
| 试管架/试管表面在 Mesh 10 mm 内 | 52.00% | **65.25%** |
| 试管架/试管表面在 Mesh 20 mm 内 | 77.25% | **90.37%** |
| 试剂瓶表面在 Mesh 20 mm 内 | 87.33% | **92.77%** |

这是最近表面距离下的完整性，不保证薄壁拓扑、独立管体、孔洞或颜色均正确。相邻器具间距很小时，一个错误连接面也可能接近真值采样点。尺度块和托盘在本轮这套指标下基本没有改善，不能声称所有物体都提高。

![分器具完整性和距离分布](../artifacts/lab_multiview_20261110/surface_completeness.png)

### 同一初始观察射线上的表现

将两份已保存 Mesh 从本轮相同初始深度相机位姿渲染，使用同一器具/台面参考像素掩码：

| 指标 | 静止 | 左右多视角 |
|---|---:|---:|
| 可见参照像素 Mesh 命中覆盖率 | 94.44% | **99.47%** |
| 同射线光轴深度 MAE | **6.02 mm** | 6.77 mm |
| 同射线光轴深度 P95 | **32.38 mm** | 35.65 mm |
| 排除台面后的器具深度 P95 | **47.87 mm** | 52.62 mm |

结论是：**缺面减少、侧面覆盖改善，但没有获得同等幅度的表面精度提升，部分深度误差反而变大。** 最近点完整性提高与同射线精度变差可以同时发生，例如错误连接面补上了空洞，却位于错误深度。

![左：实际 RGB；中：静止 Mesh；右：多视角 Mesh](../artifacts/lab_multiview_20261110/static_vs_multiview_mesh.png)

图像完全来自已保存的三角形和顶点颜色，绕过 RViz。上下两行分别是初始视点与实际侧向视点。Mesh 使用深度相机视点；RGB 相机有固有基线偏移，不以两张彩色图的逐像素差作为几何误差。

图中仍可看到管架轮廓糊化、薄杆融合和颜色混合。机器人本身也被深度相机看到；扫描时没有机器人自体遮罩，机械臂运动表面可能进入静态 TSDF 并留下历史几何。这是需后续单独检查的污染源，本轮没有自动清除它来美化画面。

## 4. 3D ESDF 结果

对两份地图使用相同 42000 个、与 2 cm 体素中心对齐的台面 ROI 采样点。范围沿用前一轮精度文档。

| 指标 | 静止 | 左右多视角 |
|---|---:|---:|
| 全图原生 observed ESDF 点 | 222408 | **582034** |
| ROI observed 采样点 | 31799 | **38015** |
| ROI unknown 比例 | 24.29% | **9.49%** |
| 两份地图共同 observed 的自由空间 MAE | 12.05 mm | **11.82 mm** |
| 共同区域 P95 | 21.04 mm | **20.64 mm** |
| 共同区域 RMSE | 14.03 mm | **13.69 mm** |

全图点数增加包括新视场的自由空间和场景周围区域，不能等同于器具表面数或只在 ROI 内增加的点数。共同观测区域使用相同 31042 个自由空间采样点比较，避免因有效集合不同造成误读。

### 新观测区域存在少量大误差

若纳入多视角地图全部 37159 个 observed 自由空间采样点：MAE=11.86 mm、P95=20.54 mm，但 **RMSE 增至 23.30 mm**。其中 198 点误差超过 50 mm，9 点超过 100 mm；4 点查询得到 **2.0 m** 上限值。

最明显例子：base 坐标 `(0.83, 0.15, 0.35)` m，场景真值距离约 **0.157 m**，ESDF 查询为 **2.0 m**，误差约 **1.843 m**。这些异常不能被较好的 MAE/P95 掩盖。

当前 `native/export.cu` 对 ESDF 只检查 observed、有限值和非负平方距离；`backend.py` 的 query 有效性检查主要排除 unknown 哨兵。它们不会单凭“达到 max_distance”判断该点的最近 site 是否正确有效。因此 observed **不保证**该数值在本场景里就是正确的可用间距。可能涉及新增零散观测块的 site/parent 有效性、距离传播状态或最大值语义，尚未完成逐体素内部状态追踪，不能在这里认定具体底层缺陷。

后续应对这 4 个点导出 parent/site 状态，验证增量更新与完整重算是否一致，再决定如何标记不可用距离；不应简单把所有 2 m 距离删去，真正远离障碍的截断距离可能有其正常语义。

![多视角 3D ESDF 的截面和误差](../artifacts/lab_multiview_20261110/esdf_accuracy.png)

这是 3D ESDF 的 z=0.11 m 截面，不是切换成 2D 模式。上述最大异常位于 z≈0.35–0.37 m，不能仅凭这个截面图判断不存在大误差。完整异常坐标保存在 `esdf_common_comparison.json`。

## 5. 实时性与地图增长

下表来自节点累计计数差/监测墙钟时间；不是 GPU 完成频率或外部订阅端实收频率，也不是端到端延迟。

| 统计内容 | 静止 | 左右多视角 |
|---|---:|---:|
| 深度回调收到 | 22.83 Hz | 22.26 Hz |
| 深度融合调用 | 18.70 Hz | 13.37 Hz |
| 颜色融合调用 | 18.45 Hz | 13.34 Hz |
| 3D ESDF 更新调用 | 18.54 Hz | 13.25 Hz |
| ESDF 点云发布调用 | 18.30 Hz | 10.21 Hz |
| Mesh 发布调用 | 4.46 Hz | 4.19 Hz |

本轮观测更多空间后，原生 ESDF 点数增加约 2.62 倍，地图、导出和发布负载随之增大。末段主机统计中，ESDF 导出含 GPU 等待均值约 38.70 ms，ESDF 打包发布均值约 109.47 ms，Mesh 读回均值约 40.32 ms。支持“全量点云导出和打包发布成为显著负担”的判断，但这些阶段有线程并发，不能简单把均值相加当作总帧延迟。

## 6. 如何继续改善

- 保留多视角，因为已经看到器具外表面覆盖改善；再用等时长或等输入帧数实验控制累计数据量的影响。
- 细管直径仅约 1–1.5 个当前体素，6 mm 架杆小于体素。进一步提高结构质量，需要固定输入比较更小体素及较短截断带，而不是无限增加相同精度下的重复融合。
- 对运动机械臂加入基于实际关节 TF 的自体深度遮罩，单独检查相机时间戳与 TF 对应，减少移动机器人被当作静态场景积累的影响。
- 优先追踪 2 m ESDF 异常点的内部 site/parent 状态与增量更新结果；不要用平均误差取代异常检查。
- 运行控制/规划所需 ESDF 查询与全量可视化发布应分别评价。后续可测试按需导出、局部显示或降低展示频率是否改善融合吞吐，本轮没有实施这些改动。

## 7. 手动复现此次仿真扫描

终端一，启动与本轮相同的场景及 UR7e 建图：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 \
  voxel_size:=0.02 truncation_vox:=4.0 esdf_mode:=3d
```

终端二，等待相机与建图已有输出后执行扫描。不要同时启动其他扫描或关节命令发布器；必须从本场景的初始姿态开始：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
export UR7E_LAB_TEST_OUT="$PWD/artifacts/manual_lab_scan_$(date +%Y%m%d_%H%M%S)"
python3 artifacts/lab_multiview_20261110/execute_motion.py
```

执行器检查 Gazebo 描述、推进的 `/clock`、关节反馈、活动的仿真控制器、工作台模型位置、起始关节姿态以及其他命令发布器；不启动机械臂实机驱动。若初始状态不符会拒绝运动。不要把这份特定仿真路径用于实机或其他场景。

扫描结束后，终端二保存新的 Mesh 和地图，变量仍使用该终端设置的输出目录：

```bash
ros2 service call /ur7e_nvblox/save_ply \
  ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '${UR7E_LAB_TEST_OUT}/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map \
  ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '${UR7E_LAB_TEST_OUT}/map.blox'}"
```

GUI 与 RViz 增加的负载可能让频率低于此次无 GUI 测试。按 Ctrl+C 停止启动终端结束仿真。扫描执行器停止发布后，仿真位置控制器保持最后目标。

## 8. 本轮结果与查看

保存目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_multiview_20261110`。

- `mesh.ply`、`map.blox`、`esdf_observed.ply`：实际结果。
- `static_vs_multiview_mesh.png`、`mesh_rgb_render.png`：离线实际 Mesh 渲染。
- `views/`：14 组实际相机 RGB、深度和图像时间 TF。
- `trajectory.json`、`motion_status.json`、`actual_collision_check.json`：运动反馈与检查。
- `comparison.json`、`accuracy_summary.json`、`esdf_common_comparison.json`：完整性、精度及异常点。
- `measurement.json`、`counter_rates.json`、`simulation.log`、`motion.log`、`save.log`：运行证据。
- `plan_motion.py`、`execute_motion.py`、`compare.py`：本轮测试与分析代码；融合功能包源码保持不变。

CloudCompare 查看本轮实际 Mesh：

```bash
CloudCompare /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_multiview_20261110/mesh.ply
```

本轮是一次扫描，不提供多轮置信区间。完整外表面统计比上一份“初始可见表面统计”范围更大，数值不可跨口径直接比较。采样包含当前路径未看到的侧面，故它适合衡量剩余缺失；仍不能证明全角度或所有孔洞都已正确重建。测试已结束，仿真进程已关闭。
