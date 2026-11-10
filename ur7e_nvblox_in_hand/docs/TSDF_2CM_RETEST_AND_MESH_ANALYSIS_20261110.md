# 2cm、4体素截断复测：直接查看TSDF并定位Mesh缺失

日期：2026-11-10。目标：将ur7e_nvblox_in_hand改为2cm体素、保留4体素截断（8cm），用同一静止仿真输入重新测试，并直接检查TSDF距离/权重。没有操作实机；没有修改C++版或workspaces。

## 1. 已完成的修改

| 文件 | 修改 |
|---|---|
| src/ur7e_nvblox_in_hand/config/base.yaml | voxel_size: 0.03 → 0.02 |
| src/ur7e_nvblox_in_hand/config/realtime.yaml | voxel_size: 0.03 → 0.02 |
| src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py | DEFAULTS.voxel_size: .03 → .02 |

truncation_vox原本就是4.0，保持不变；新物理截断尺度为0.02×4=0.08m。本轮没有修改融合公式、权重、手眼标定、图像时间对应、原生TSDF导出桥或网格提取算法。reference profile本来已是0.02m。

5项现有测试通过，Python ROS包colcon编译通过；2cm运行参数导出确认voxel_size=0.02、truncation_vox=4.0。

显式命令参数优先：历史脚本/文档若仍传voxel_size:=0.03，会覆盖新默认值。根目录的旧双后端比较脚本默认0.03没有在本轮一起修改；推荐使用本文直接launch命令。

## 2. 测试方法

复用此前color_ab_20261110/input_bag，同一19.357秒静止仿真输入，640×480、最大融合范围3m、颜色开启、深度/颜色限频40Hz、ESDF更新/导出33.4Hz、Mesh5Hz、关闭衰减/半径清理。

依次新建两个Mapper：3cm/4体素作为同轮基线，2cm/4体素作为新配置。每轮回放结束导出Mesh与原生map.blox，停止运行后离线加载地图，用已有native export_layer接口提取所有已分配TSDF体素的xyz、distance、weight，不通过Mesh反推TSDF。

为避免仅看图猜测，执行三项检查：

1. 用相同相机K和图像时刻位姿，将Mesh投影到深度相机视图，计算覆盖率和重叠深度误差。
2. 直接绘制TSDF截面距离、权重，并与原始深度中间行曲线对照；导出零附近体素点云。
3. 对每个原始深度表面点检查附近8个规则网格样本的权重与符号，观察Mesh缺面与局部支持不足是否相关。

第三项是局部支持诊断，不是CUDA Marching Cubes逐单元拒绝日志，不能把相关性写成已完成全部源码根因归因。

## 3. Mesh复测结果

| 指标 | 3cm，4体素=12cm | 2cm，4体素=8cm |
|---|---:|---:|
| Mesh顶点 | 3006 | 7214 |
| Mesh三角面 | 4306 | 10660 |
| 对307200个有效深度像素的覆盖率 | 84.71% | 89.76% |
| 没有对应Mesh的像素 | 46963 | 31469 |
| 重叠区域深度误差中位数 | 0.335mm | 0.342mm |
| 重叠区域深度误差P95 | 17.80mm | 6.78mm |
| 重叠区域平均绝对深度误差 | 5.76mm | 4.82mm |

覆盖率提高约5.04个百分点，缺失像素减少约33%；P95降低约61.9%。中位数没有改善，说明改善主要体现在部分边缘/较大误差区域，不能写成所有表面精度都同步提升。

本轮既改变了网格尺寸，也改变了物理截断尺度，不能把收益全部单独归因于其中一个因素。若要继续区分，应另设2cm/6体素=12cm对照，本轮尚未执行。

误差仅在两者均有有效表面的像素上计算，不能掩盖剩余10.24%的未覆盖区域。数值表示对本轮输入深度的一致性，不是相对外部真值的绝对精度；该结果也不能直接替代此前实机灯板测试。

## 4. TSDF直接检查结果

| 指标 | 3cm | 2cm |
|---|---:|---:|
| 分配体素数 | 75776 | 168960 |
| 已观测体素（weight≥0.1） | 24184 | 71057 |
| 未观测体素（weight=0） | 51592 | 97903 |
| 零附近体素（weight≥0.1且abs(distance)≤1体素） | 3545 | 8369 |
| 非零权重最小/最大值 | 5 / 5 | 5 / 5 |
| 表面附近TSDF与输入投影距离残差P95 | 约1.00mm | 约1.00mm |

这里的TSDF是投影有符号距离，不是ESDF欧氏距离。残差计算为：将体素投影到原始深度，计算clip(depth(u,v)-Z_voxel, -mu, mu)，与保存的TSDF距离比较；只评价已观测且位于表面附近的样本。它不对未知区域打分。

观察：已观测区域距离值大体符合输入，权重已饱和，并没有证据支持“整个TSDF都被融合错位”。但观测支持是局部且不完整的，尤其在深度不连续处和视场边缘。TSDF已有值与其邻域足以生成连续网格不是同一个条件。

![TSDF距离、权重截面与Mesh深度对比](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/tsdf_2cm_check_20261110/tsdf_sections_and_mesh.png)

读图：左列红/白/蓝为正/零附近/负投影距离，黑线为原始深度截面；中列为权重，空白区域不应当成已观测自由空间；右列为同一深度视角下的Mesh深度。TSDF截面选取有限厚度的相机Y带，并非严格解析切面；深度边缘处可能同时包含相邻行不同表面。

## 5. Mesh差是否说明TSDF也差

需要分开三种情况：

- 距离数值错误：零交叉位置错误，提取Mesh当然会错。
- 距离数值合理但支持不完整：已有表面位置可以正确，邻域未观测导致缺面或边缘退缩。
- TSDF与角点都有效，Mesh仍异常：才进一步怀疑提取、块边界或输出路径。

本轮数据更支持第二类。原始表面点附近8样本检查发现：

| 项目 | 3cm | 2cm |
|---|---:|---:|
| 全部有效表面点具备8邻域权重支持的比例 | 84.16% | 88.84% |
| Mesh缺失像素中，附近存在权重不足样本的比例 | 94.83% | 89.99% |

本轮非零权重均为5，没有0～0.1之间的低权重样本。因此权重不足主要是未观测/缺少样本，而不是“只差一点权重就过门限”。直接把Mesh门限从0.1降到0.01不会补齐weight=0的区域；把未观测体素当作可靠表面则可能造出假三角面。

该检查并不能精确代替每个实际网格单元的判定，但已经把“整体融合错位”的模糊判断缩小为“边界/邻域支持与网格完整性”的明确调查方向。

## 6. 为什么静止重复融合也不会自动补好

每个深度像素是一个表面观测；Marching Cubes需要局部网格角点及邻块满足条件。粗网格的一个单元可跨越前景/背景、视场边界或遮挡轮廓。一部分角点永远没有得到有效支持，重复同一视角也不会自动观测到它们。

这不是说所有缺面都合理，而是说明：不能只用“深度图每个像素都有值”推导“每个三维网格单元都有完整支持”。两者采样位置不同。

本轮同视角主要平面仍然对应，并未复现整幅地图被明显平移/旋转的情况。此前俯视Mesh与相机RGB的直接比较，还可能叠加视角和颜色问题；本次只能对该静止仿真作结论，不能证明旧实机一定无坐标问题。

## 7. 建议解决顺序

1. 保留本轮2cm设置，先评价它对实机灯板边缘及薄结构的改善，不再以3cm结果推断2cm。
2. 对剩余缺面，增加实际Mesh单元级的拒绝原因统计：缺块、零权重、无符号变化、低权重。当前局部相关性诊断不能代替这个统计。
3. 区分观测内边缘与视场边界。对有明确平面支持的区域，考虑有约束的边界重建；未知/遮挡区不能无条件补面。
4. 多视角可以补观测支持，但不是静止正确性的替代；先确保静止平面、台阶的端到端几何测试通过，再评价运动。
5. 若已观测TSDF距离出现系统性偏差，优先检查模型/投影与位姿；不要直接用显示平滑掩盖。当前观测距离并没有表现出普遍大偏差。
6. 分别评估截断物理尺度、网格尺寸和视图块选择的影响。更细体素增加资源需求；本轮观测体素约增至2.94倍，尚未完成满速实时性能测试。

## 8. 运行记录与性能边界

同一录包深度约20.1Hz、RGB约16.4Hz。本轮监测计数差：

| 配置 | 深度融合调用 | 颜色融合调用 | ESDF更新调用 |
|---|---:|---:|---:|
| 3cm | 19.57Hz | 16.02Hz | 19.51Hz |
| 2cm | 19.51Hz | 15.91Hz | 19.35Hz |

两组都接近本轮输入，不能据此宣称2cm一定能或不能达到30Hz。Mesh采用原始序列化订阅，减少上轮监测器Python逐顶点反序列化开销；这轮频率不应与上轮不同监测负载的接收Hz直接排名。

## 9. 查看TSDF与Mesh

实际输出目录：
`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/tsdf_2cm_check_20261110/`

2cm结果可独立打开：

```bash
CloudCompare \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/tsdf_2cm_check_20261110/voxel_2cm/tsdf_near_zero.ply \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/tsdf_2cm_check_20261110/voxel_2cm/mesh.ply
```

两者均为base_link坐标，不要额外变换。TSDF PLY是零附近体素中心，不是提取表面或RGB纹理；颜色表达符号距离。CloudCompare可选择distance或weight标量场查看。全部已分配体素（含未知）保存在tsdf_voxels.npz中；未知以weight=0区分。

每组还有map.blox、params.yaml、measurement.json、mesh_depth_optical.ply、comparison.json、tsdf_stats.json。顶层summary.json保存详细支持率与误差。原生地图可用于后续离线重复诊断。

## 10. 完整运行指令

本次Python包已编译。后续修改后只需：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
colcon build --base-paths src --packages-select ur7e_nvblox_in_hand --executor sequential
source ./env.sh
```

### 同录包仿真复现

终端A（无需启动Gazebo或硬件）：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true \
  run_rviz:=true rviz_mode:=diagnostic \
  voxel_size:=0.02 truncation_vox:=4.0 max_range_m:=3.0 \
  enable_color:=true depth_hz:=40.0 color_hz:=40.0 \
  esdf_hz:=33.4 esdf_view_hz:=33.4 mesh_hz:=5.0 layer_hz:=5.0 \
  decay_hz:=0.0 clearing_hz:=0.0
```

终端B：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 bag play /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/color_ab_20261110/input_bag
```

录包包含/clock和TF，不加--clock，不同时启动实时仿真源，不循环回放。比较参数时退出旧Mapper，新建地图后重新播放。

### 实机使用新2cm配置

保持此前机械臂驱动、D435i和安装TF终端运行，在域135中：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=false \
  run_rviz:=true rviz_mode:=diagnostic \
  voxel_size:=0.02 truncation_vox:=4.0 max_range_m:=3.0
```

本轮没有执行这段实机指令。测试Mapper、回放均已结束，不会自动控制机械臂。
