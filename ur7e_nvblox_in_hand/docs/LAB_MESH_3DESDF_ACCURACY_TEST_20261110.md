# 工作台场景：UR7e 功能包的 Mesh 与 3D ESDF 实测

日期：2026-11-10。使用 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`，即 Python ROS 接入 + `nvblox_torch` C++/CUDA 后端；本轮没有使用兄弟目录 `nvblox_in_hand` 的 ROS C++ 节点，也没有修改融合源代码。

本次实际启动新工作台 Gazebo 场景，采集深度与图像时间对应的 TF，导出 Mesh、原生地图和 ESDF，并与场景几何比较。所有图片来自实际数据。结果说明：大平面恢复较好，但细管、孔洞和架体结构有明显损失；ESDF 存在厘米级距离偏差和未观测区域，不能以显示连续或点数较多作为精度达标的依据。

## 1. 测试设置与范围

| 项目 | 设置 |
|---|---|
| 启动项 | `ur7e_nvblox_in_hand bringup.launch.py scene_variant:=lab` |
| 输入 | 仿真 RGB、深度均 640×480，传感器目标 30 Hz |
| 相机位姿 | 机械臂 TF；固定实验台俯视初始姿态，未执行环绕或摇动 |
| 体素 | 0.02 m |
| TSDF 截断 | 4 体素，即 0.08 m |
| 深度/颜色融合上限 | 40/40 Hz；不是实际输入或融合频率保证 |
| 3D ESDF 更新/导出目标 | 33.4/33.4 Hz |
| Mesh 目标频率 | 5 Hz |
| 衰减/半径清图 | 关闭 |
| 测试进程 | ROS_DOMAIN_ID=136，Gazebo 端口 11566；结束后已关闭 |
| 稳定采样窗口 | 45.25 秒，来自首末有效 metrics 的墙钟时间差 |

不是多视角全模型完整性测试，不包含拖动运动下的残影评价。本次首先确认固定视角下的误差分布。背面、被遮挡区域和未进入视场的区域不能据此判定已重建。

## 2. 精度参照如何建立

### Mesh

使用 `lab_scene.yaml` 与 `lab_scene.py` 的实际部件尺寸和位姿重建参考几何，包含台面、架体、试管、瓶子、托盘和尺度块。圆柱参考网格用 64 边近似，球体采用三次细分；孔环直接加载场景使用的 STL。几何参考不依赖融合出来的 Mesh。

从采集到的深度内参和该图像时间戳的 `base_link → camera_depth_optical_frame` 变换建立相同射线，分别渲染参考模型与导出 Mesh。主指标是**相同像素射线上的光轴深度差**，不是整个三维空间的最近点距离。另行提供双向最近表面距离，二者不能混为同一种“误差”。

评估对象位于 x=[0.17,1.01]、y=[-0.50,0.50] m，参考可见表面 z=[-0.001,0.40] m。参考几何不包含机械臂；原始深度比器具参考前移超过 6 cm 的前景像素作为机械臂遮挡排除，本轮排除 811 像素。这是实用遮挡筛选，仍有边缘和近距离遮挡歧义，不能视作完整机器人几何真值。

保留 140634 个有效参照像素。缺失 Mesh 像素计入覆盖率损失，不填零后参加平均误差，也没有将缺失忽略成“全部正确”。

### 3D ESDF

读取保存的同一份 `.blox` 地图，导出原生 observed ESDF。另在台面工作范围 x=[0.17,1.01)、y=[-0.49,0.50)、z=[0.01,0.40) m 上，以原生 2 cm 体素中心对齐采样，共 42000 点，保留查询返回的 observed/unknown。

对盒体、圆柱、球体、环形圆柱等场景实体计算解析距离。**自由空间中**到实体并集的最近距离可由各实体距离的最小值获得，用它与 ESDF 距离比较。器具内部不使用这一公式声称获得精确并集内部有符号距离，因此主精度指标限定在真值自由空间且 ESDF observed 的采样点。

采样网格只是离线评估工具，不参与建图，也没有把 unknown 点填入发布点云。精度指标限定在台面 ROI；全图 22 万个 ESDF 点不等于全部参加本次误差统计。

## 3. Mesh 结果

导出 Mesh：**15305 顶点、22095 个三角形**。

| 指标 | 实测 |
|---|---:|
| 参照可见像素上的 Mesh 命中覆盖率 | 94.44% |
| 同时命中且光轴深度误差 ≤20 mm，占全部参照像素 | 86.03% |
| Mesh 对场景真值：光轴深度 MAE | 6.02 mm |
| Mesh 对场景真值：光轴深度 RMSE | 16.69 mm |
| Mesh 对场景真值：光轴深度误差中位数 | 0.60 mm |
| Mesh 对场景真值：光轴深度误差 P95 | 32.38 mm |
| 原始深度对同一场景真值：MAE / P95 | 1.68 / 3.23 mm |
| Mesh 对原始深度：MAE / P95 | 6.17 / 33.68 mm |
| 可见 Mesh 点到参考几何最近表面距离：均值 / P95 | 2.17 / 10.57 mm |
| 可见真值点到 Mesh 最近表面距离：均值 / P95 | 2.57 / 11.00 mm |
| 可见真值点在 Mesh 最近表面 20 mm 内的比例 | 98.90% |

P95 表示 95% 的有效比较样本误差不超过该值，不是最大误差。最近表面距离可能匹配到相邻管、孔边或台面，因此它比同射线深度差小，**不能用最近表面 98.90% 的数字宣称试管和孔洞都完整恢复**。这里所有完整性比例都仅指当前视角可见参照表面。

### 分物体结果：同射线光轴深度差

| 区域 | Mesh 命中覆盖率 | MAE | P95 |
|---|---:|---:|---:|
| 台面 | 94.73% | 2.70 mm | 7.40 mm |
| 试管架与试管 | 90.52% | 22.92 mm | 65.17 mm |
| 试剂瓶 | 91.40% | 11.66 mm | 42.88 mm |
| 托盘与小样品瓶 | 95.30% | 8.13 mm | 28.90 mm |
| 分段锥形器皿 | 98.77% | 7.40 mm | 29.89 mm |
| 尺度块 | 99.98% | 5.14 mm | 27.18 mm |
| 全部器具，排除台面 | 93.80% | 13.57 mm | 47.86 mm |

台面约占参照像素的 69%，整体 0.60 mm 中位数被大量平面像素拉低。评价器具细节时，应看排除台面后的指标和试管架分组。

![实际 RGB、导出 Mesh、参考几何、深度与误差图](../artifacts/lab_accuracy_20261110/mesh_accuracy.png)

图中 Mesh 颜色由实际导出顶点颜色和三角形重心插值渲染，绕过 RViz。它采用深度相机视点，左上 RGB 来自 RGB 相机，因此两者存在相机固有基线视差，不能直接逐像素比较颜色。几何误差图使用统一深度相机射线。

## 4. 3D ESDF 结果

| 指标 | 实测 |
|---|---:|
| 全图原生 observed ESDF 点数 | 222408 |
| 评估 ROI 体素中心采样数 | 42000 |
| ROI observed 数 | 31799 |
| ROI unknown 比例 | 24.29% |
| 真值自由空间采样数 | 41110 |
| 其中 observed 数 | 31042 |
| 真值自由空间 observed 比例 | 75.51% |
| observed 自由空间距离 MAE | 12.05 mm |
| observed 自由空间距离 RMSE | 14.03 mm |
| observed 自由空间绝对误差中位数 / P95 | 10.37 / 21.04 mm |
| 有符号平均误差：估计值−真值 | +9.68 mm |
| 真值自由、查询距离却为负的点 | 502 |
| 真值离表面 ≥20 mm、查询距离仍为负的点 | 50 |

unknown 不参加距离 MAE，但单独计入缺失比例。502 个负距离点大多位于离真实表面不足一个体素的区域；另有 50 个超过这一范围，需要进一步按边界、薄结构与体素内部标记定位。本轮不能据此认定碰撞距离已经足够可靠。

| 真值到表面距离 | 有效样本数 | ESDF MAE | ESDF P95 |
|---|---:|---:|---:|
| 0–2 cm | 4127 | 13.92 mm | 33.50 mm |
| 2–5 cm | 5189 | 11.60 mm | 22.91 mm |
| 5–10 cm | 9277 | 10.59 mm | 19.11 mm |
| 10–20 cm | 10628 | 12.25 mm | 19.97 mm |
| ≥20 cm | 1821 | 15.32 mm | 22.83 mm |

![真实 3D ESDF 的 z=0.11 m 截面及误差](../artifacts/lab_accuracy_20261110/esdf_accuracy.png)

这是 **3D ESDF 的截面**，没有切换为 2D ESDF 算法。左图缺点表示 unknown；右图只比较真值自由且 observed 的区域，因此实体内部同样被排除。主色阶对应距离，误差图红色表示高估距离、蓝色表示低估距离。

![已观测 3D ESDF 台面区域](../artifacts/lab_accuracy_20261110/esdf_3d.png)

三维图为了可读性，仅显示 ROI 内真值表面 16 cm 范围的 observed 自由空间点，并抽取三分之一；完整原生点云另存于 `esdf_observed.ply`。

## 5. 同轮实时性记录

以下是节点累计计数差除以 45.25 秒监测墙钟时间，不是 GPU 完成频率，也不是外部订阅端实收话题频率。

| 内容 | 墙钟统计 |
|---|---:|
| 深度回调收到 | 22.83 Hz |
| 颜色回调收到 | 22.41 Hz |
| 深度融合调用 | 18.70 Hz |
| 颜色融合调用 | 18.45 Hz |
| 3D ESDF 更新调用 | 18.54 Hz |
| ESDF 点云发布调用 | 18.30 Hz |
| Mesh 发布调用 | 4.46 Hz |

本轮全分辨率复杂场景、完整颜色/层/ESDF/Mesh 链路未达到 30 Hz。传感器配置 30 Hz 也没有让节点收到 30 Hz 墙钟输入，需再用 `/clock`、Gazebo real-time factor 与相机端接收统计拆分仿真变慢和 ROS 丢帧。

监测器的外部 PointCloud2 接收记录为 0、Mesh 仅收到 1 条，因此**不能将该接收计数当作实际没有输出，也不能用本轮监测器宣称接收端达到了 18.30 Hz**。非空地图与节点发布计数已经验证；接收传输问题尚需独立检查。`timings_host_ms` 是各阶段主机计时，也不等价于完整相机到显示延迟，本次没有给出端到端延迟结论。

## 6. 从结果判断问题和后续方向

1. **细结构的分辨率明显不够。** 架体栏杆宽约 6 mm，试管直径约 23–30 mm，接近或小于 20 mm 体素所能稳定表达的尺度。一个试管直径只有约 1–1.5 个体素。原始深度可分辨相邻轮廓，融合到离散 TSDF 再提取三角形时，孔洞、薄壁、边缘更容易丢失或连接。试管组 P95=65.17 mm 和绕过 RViz 后仍出现变形支持这一判断，不能归结于 RViz 渲染。
2. **8 cm 截断范围相对器具间隙很大。** 在薄结构和深度不连续处，宽投影带、权重策略与缺失的邻域观测可能共同影响零交叉和 Marching Cubes。当前结果不能单独证明把截断缩短就能解决；下一步应固定输入分别比较 2/3/4 体素截断，以及 1 cm/2 cm 体素。
3. **单视角仍缺少侧面约束。** 多帧重复同一视角不能补出被遮住的孔内、瓶体背面和架体侧面。ROI 有 24.29% unknown，不应通过强填自由体素来获得“完整显示”。应增加少量有侧向视差的视角，另报可见/全表面完整性。
4. **ESDF 相对真实连续表面有系统偏差。** 台面邻近自由空间距离误差的中位数约 10 mm，恰为半体素；原生 site 中台面区域常见 z=-0.01 m，另有 z=-0.03 m 的内部 site。说明离散 site 的位置会影响到连续 z=0 台面的距离比较。全体自由空间平均高估约 9.68 mm，后续应检查 site 选择与表面体素偏置，并通过更小体素验证。不能未经验证统一减去 1 cm，复杂表面不具有相同偏差。
5. **近期优先级。** 先用同一记录输入比较分辨率/截断尺度，观察试管组误差是否下降；再做短距离左右视差扫描改善 completeness；最后分离 Mesh、层发布和 ESDF 导出的开销，针对墙钟吞吐优化。颜色和几何分别量化，避免仅凭彩色效果图判断 TSDF 正确。

这些是基于本轮证据的优化方向，本次没有修改算法或替换配置来美化测试结果。

## 7. 查看与复现

交互查看场景和重建：

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

查看实际导出数据：

```bash
CloudCompare \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110/mesh.ply \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110/ground_truth_lab.ply \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110/esdf_observed.ply
```

建议分别开关 Mesh 与真值，避免重合表面的闪烁；ESDF PLY 的 `distance` 为标量场，显示范围可设 0–0.2 m。Mesh 没有表面的地方，不能用 ESDF 自由空间点替代重建表面。

复现自动采集与离线精度计算（先关闭同端口已有仿真；使用新输出目录，避免覆盖本次证据）：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export UR7E_LAB_TEST_OUT="$PWD/artifacts/lab_accuracy_$(date +%Y%m%d_%H%M%S)"
python3 artifacts/lab_accuracy_20261110/run_test.py
python3 artifacts/lab_accuracy_20261110/export_fields.py
python3 artifacts/lab_accuracy_20261110/analyze.py
```

复现使用当前机器已安装的 NumPy、SciPy、Pillow、Matplotlib、trimesh、Open3D，以及功能包自身的 ROS/nvBlox 依赖。测试脚本不发送运动命令，仅启动隔离仿真、订阅、保存和分析。新输出目录必须位于本工作区以保持归档一致。

## 8. 数据索引与限制

结果目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110`。

- `mesh.ply`：真实保存的彩色 Mesh。
- `map.blox`：本轮原生地图，可离线加载并查询。
- `esdf_native.npy` / `esdf_observed.ply`：222408 个原生 observed ESDF 点。
- `esdf_roi_query.npz` / `accuracy_fields.npz`：ROI 查询及真值距离，保留 unknown 掩码。
- `ground_truth_lab.ply` / `ground_truth_depth.npy`：器具及工作台参考几何、同视点参考深度。
- `accuracy_summary.json`：总体与分组精度。
- `measurement.json` / `metrics_events.json` / `counter_rates.json`：运行统计及计算口径。
- `params.yaml` / `command.txt` / `simulation.log` / `save.log`：实际配置、启动命令与导出证据。
- `capture.json` / `depth_m.npy` / `rgb.png`：原始输入、内参和图像时间对应 TF。

这是单次静态仿真基线，不提供多轮置信区间。几何参照采用当前场景的名义尺寸，未包含真实传感器材质行为；不会证明实机达到相同精度。像素指标按图像采样加权，并非三角形面积均匀采样；大台面会占较多权重，因此已另报排除台面的器具指标。
