# UR7e：1 cm 体素、ESDF stride=2 工作台多视角测试

2026-11-10。已修改、编译并实际测试 `ur7e_nvblox_in_hand`。使用相同工作台、640×480 RGB-D 和左右约 36 cm 的机械臂扫描，采集 14 个实际视点，返回原观察姿态，保存 Mesh、原生地图、完整 ESDF 和 stride=2 显示子集。未使用 workspaces ROS 节点开展本轮融合。

## 1. 修改含义

- `voxel_size=0.01 m`：TSDF、Mesh 与完整 3D ESDF 使用 1 cm 体素。
- 新参数 `esdf_view_stride=2`：仅在 GPU 导出 3D ESDF **可视化点云**时，按全局体素索引沿 X/Y/Z 各隔一个体素取样。规则为三个索引都满足 `index % 2 == 0`，理论密集网格保留约 1/8。
- 显示采样间隔约 2 cm，但地图仍是 1 cm，查询仍能访问完整 ESDF。Mesh 不应用这一 stride，TSDF 融合和 raycast_subsampling_factor=4 不受该参数改变。
- 保留 `truncation_vox=4`，所以物理截断从原 2 cm×4=8 cm 变为当前 1 cm×4=4 cm。
- 2D 切片导出强制使用 stride=1，避免对单层 z 索引抽样后整层消失。

这次同时改变了体素精度、物理截断范围和显示采样负载，不能把所有改善单独归因于其中某一个参数。

## 2. 具体修改文件

以下路径均相对于 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`：

| 文件 | 修改 |
|---|---|
| `src/ur7e_nvblox_in_hand/config/base.yaml` | 默认体素改为 0.01，增加 esdf_view_stride=2 |
| `src/ur7e_nvblox_in_hand/config/realtime.yaml` | 实时 profile 同步 1 cm/stride2 |
| `src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py` | 默认值、正整数校验、3D 导出传参；metrics 增加导出点数、stride 和显示间隔字段 |
| `.../backend.py` | dense 导出保留，新增 strided GPU 接口分派 |
| `native/export.cu` | GPU compact kernel 在搬回 CPU 前按全局 XYZ 索引过滤；负索引和跨块采样保持一致 |
| `native/export.hpp` | 导出接口增加 stride 参数，C++ 默认值仍为 1 |
| `native/bridge.cpp` | 保留旧 3 参数 Torch op，新增 export_esdf_strided，避免破坏离线完整地图导出 |
| `tests/test_esdf_stride.py` | 参数拒绝测试、dense/stride 接口分派测试 |

`metrics.native_esdf_points` 为兼容旧消费者保留，但它原本就记录“最近一次导出点数”，现在包含 stride 选择后的数量，不应当作完整地图总数。新增 `esdf_exported_points`、`esdf_view_stride`、`esdf_view_spacing_m` 明确解释；完整点数通过离线 dense 导出测量。

## 3. 编译与正确性验证

本地原生 CUDA bridge 和 ROS Python 包均已编译安装。参数、旧接口兼容和话题/消息相关测试共 **10 项通过**。

额外在真实地图上做 GPU 结果校验：

- 旧地图 dense 导出 222408 点，stride2 导出 27856 点。
- 根据完整点的全局 XYZ 体素索引计算预期子集，再逐项比较排序后的所有 7 列：坐标、距离、observed、inside、site 完全一致，包含负坐标。
- 本轮 1 cm 地图也进行了相同检查，全部通过。

没有生成没有观测信息的占位点，没有把 stride2 当作融合体素改成 2 cm。

## 4. 本轮实际数据规模

| 项目 | 结果 |
|---|---:|
| Mesh 顶点 | 229719 |
| Mesh 三角形 | 335853 |
| 完整 observed ESDF 点 | **4960839** |
| stride2 导出点 | **623375** |
| 导出子集/完整点数 | **12.57%** |
| ESDF 精度评估 ROI 采样数 | 336000 |
| ROI observed 数 | 305079 |
| ROI unknown 比例 | **9.20%** |
| 原生地图文件 | 约 286 MiB |

点数比例不是严格 12.5%，因为稀疏观测边界不均匀。旧 2 cm 地图与新 1 cm 地图的空间占用、观测边界也不完全一致。

ESDF 精度按 1 cm 原生体素中心采样，ROI 边界与此前一致：x=[0.16,1.0)、y=[-0.5,0.5)、z=[0,0.4) m。旧 2 cm 为 42000 个中心，新 1 cm 为 336000 个中心；它们覆盖同一物理范围，但不是相同坐标集合。不能直接比较异常“个数”而忽略采样密度差异。

## 5. Mesh 精度与效果

沿用之前场景几何真值、相机图像时间 TF 和可见台面/器具 ROI 的评估方法。两轮均为左右多视角扫描，但并非相同 rosbag、等帧数或等机器负载的严格回放实验。

| 指标 | 上轮 2 cm / 截断8 cm | 本轮 1 cm / 截断4 cm |
|---|---:|---:|
| 参照可见像素 Mesh 射线命中率 | 99.47% | **100.00%** |
| 同射线光轴深度 MAE | 6.77 mm | **3.28 mm** |
| 同射线光轴深度 P95 | 35.65 mm | **13.56 mm** |
| 排除台面后的器具深度 MAE | 14.85 mm | **7.11 mm** |
| 排除台面后的器具深度 P95 | 52.62 mm | **29.45 mm** |
| 可见 Mesh 到参考最近表面距离 P95 | 见上一轮原始统计 | **3.49 mm** |

本轮在全部参照可见像素中，**96.78% 同时命中且深度差≤20 mm**。100% 命中不代表全物体所有表面都正确，也不保证所有孔洞、独立薄管或颜色均正确；错误连接面同样可以形成射线命中。

### 本轮分物体的同射线深度误差

| 区域 | MAE | P95 |
|---|---:|---:|
| 台面 | 1.57 mm | 0.86 mm |
| 试管架与试管 | 13.18 mm | **65.35 mm** |
| 试剂瓶 | 4.73 mm | 18.47 mm |
| 托盘与小瓶 | 3.02 mm | 10.72 mm |
| 锥形器皿 | 3.56 mm | 8.96 mm |
| 尺度块 | 2.55 mm | 12.89 mm |

台面多数像素误差很小，但少量长尾拉高 MAE，因此其 P95 小于 MAE 并不矛盾。试管架 P95 仍达到 65.35 mm，上轮为 68.98 mm：细结构的问题没有因整体指标改善而消失。

![实际 RGB、2cm Mesh、1cm Mesh](../artifacts/lab_1cm_stride2_20261110/mesh_2cm_vs_1cm.png)

图片由真实导出的 Mesh 三角形和顶点颜色渲染，不依赖 RViz。RGB 与 Mesh 分别来自 RGB/深度相机视点，存在固有基线偏移，精度指标使用统一深度相机射线。

## 6. 3D ESDF 精度与异常

主指标仅统计真值自由且 observed 的采样点；unknown 单独计数。

| 指标 | 上轮2 cm | 本轮1 cm |
|---|---:|---:|
| observed 自由空间距离 MAE | 11.86 mm | **5.48 mm** |
| 距离绝对误差中位数 | 10.09 mm | **5.00 mm** |
| 距离误差 P95 | 20.54 mm | **9.83 mm** |
| 距离误差 RMSE | 23.30 mm | **10.82 mm** |
| ROI unknown | 9.49% | **9.20%** |

误差中位数约 5 mm，与 1 cm 体素的半体素尺度一致；说明离散 site 位置的影响仍在，只是尺度减小。这是结果与机制一致的线索，不是把全部误差认定为固定偏置。

![完整3D ESDF的0.115m截面与误差](../artifacts/lab_1cm_stride2_20261110/esdf_accuracy.png)

这是从完整 1 cm ESDF 中取 z=0.115 m 的截面，**不是**降成 2D ESDF，也不是 stride2 点云的精度。显示 stride 不改变这里的查询结果。

仍然发现异常：296787 个 observed 自由空间采样点中，999 点误差>50 mm，61 点>100 mm，5 点返回 2 m 上限。最严重样本 `(0.915,0.375,0.175)` m，真值约 0.075 m，返回 2.0 m，误差约 **1.925 m**。这些点约占采样的极小比例，P95 不会暴露它们，但不能忽略。

真值自由却返回负距离的点为 2122 个，其中 186 个真值距表面≥20 mm。相较旧图的数目不能直接作为退化结论，因为新 ROI 采样密度是八倍。异常坐标保留在 `esdf_outliers.json`。

本轮没有修复 site/parent/增量传播及最大距离语义的问题；更小体素改善典型误差，没有使这些异常自动消失。

## 7. 实际运行频率

本轮监测窗口 **105.18 秒**。下表是节点累计计数差/监测墙钟时间，不是 GPU 完成频率或 RViz 显示频率。

| 内容 | 上轮2cm/stride1 | 本轮1cm/stride2 |
|---|---:|---:|
| 深度回调接收 | 22.26 Hz | **27.40 Hz** |
| 颜色回调接收 | 22.13 Hz | **27.93 Hz** |
| 深度融合调用 | 13.37 Hz | **22.09 Hz** |
| 颜色融合调用 | 13.34 Hz | **21.94 Hz** |
| 3D ESDF 更新调用 | 13.25 Hz | **18.27 Hz** |
| ESDF 点云发布调用 | 10.21 Hz | **18.26 Hz** |
| Mesh 发布调用 | 4.19 Hz | **4.78 Hz** |

本轮数值更高，但同时改变了空间精度、截断、GPU 导出实现和采样负载，输入实际接收频率也不同，所以不能单凭两次测试宣称 stride2 单独造成这些增益。要隔离 stride 的性能作用，还需同一记录数据下比较 1cm/stride1 与 1cm/stride2。

末段主机计时均值：ESDF 提交约 5.41 ms，ESDF 导出含 GPU 等待约 23.77 ms，点云打包发布约 28.97 ms，Mesh 读回约 54.37 ms，Mesh 打包发布约 75.92 ms。各阶段存在并发，不能直接相加当作相机到显示延迟。

本轮外部 best-effort 监测订阅器未收到完整 ESDF/Mesh 消息，因此没有给出接收端 topic hz 或显示 FPS。非空地图和发布调用已经验证，但不可把发布调用18.26 Hz表述成“RViz接收18.26 Hz”。RViz 的大点云话题应使用 Reliable QoS；传输接收仍需要另外验证。

## 8. 运行命令

更新代码已编译。当前默认已改为 1 cm、stride2；命令中显式写出以便复现。

需要重新编译时：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
```

终端一，场景、建图和 RViz：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 \
  voxel_size:=0.01 truncation_vox:=4.0 \
  esdf_mode:=3d esdf_view_stride:=2
```

终端二，执行相同的定高左右扫描：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
export UR7E_LAB_TEST_OUT="$PWD/artifacts/manual_1cm_scan_$(date +%Y%m%d_%H%M%S)"
python3 artifacts/lab_1cm_stride2_20261110/execute_motion.py
```

该脚本只支持此隔离仿真、初始姿态和已检查路线；不要同时启动另一个关节命令发布器。GUI/RViz 会增加负载，实际频率可能与无 GUI 测试不同。

保存手动扫描结果：

```bash
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '${UR7E_LAB_TEST_OUT}/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '${UR7E_LAB_TEST_OUT}/map.blox'}"
```

查看本轮已保存的结果：

```bash
CloudCompare /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_1cm_stride2_20261110/mesh.ply
# 或查看 stride2 ESDF，distance 为标量字段
CloudCompare /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_1cm_stride2_20261110/esdf_stride2.ply
```

恢复旧显示密度可显式指定 `esdf_view_stride:=1`；恢复旧体素对照用 `voxel_size:=0.02 esdf_view_stride:=1`。两者不是同一概念。

## 9. 结果目录与边界

`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_1cm_stride2_20261110/`：

- `mesh.ply` / `map.blox`：实际结果。
- `esdf_native.npy`：4960839 个完整 observed 点；`esdf_observed.ply` 为完整点云。
- `esdf_stride2.npy` / `esdf_stride2.ply`：623375 点显示子集。
- `esdf_roi_query.npz` / `accuracy_fields.npz`：完整精度的 ROI 查询，含 unknown 掩码。
- `export_summary.json` / `stride_verification.json`：GPU 子集校验证据。
- `accuracy_summary.json` / `esdf_outliers.json` / `comparison.json`：精度和异常。
- `measurement.json` / `counter_rates.json` / `params.yaml` / 构建与运行日志：配置和频率证据。
- `views/` / `trajectory.json` / `motion_status.json`：14 组实际视点与运动反馈。

本次没有进行抓取；射线命中率100%和典型距离误差改善不能替代目标位姿/夹爪接触精度、全表面完整性或异常距离检查。测试已结束，相关仿真和运动进程已关闭。
