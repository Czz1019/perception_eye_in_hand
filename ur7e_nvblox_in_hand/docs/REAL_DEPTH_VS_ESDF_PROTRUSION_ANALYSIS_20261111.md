# 深度数据与ESDF左上角突出区域：关联分析

日期：2026-11-11，Asia/Shanghai。对象：`ur7e_nvblox_in_hand`实机UR7e/D435i，5mm体素。本轮没有移动机械臂、改变参数、重启设备或修改感知源码。

## 1. 判断结果

**深度输入可以影响ESDF的观测范围和伪表面，但现有数据不足以把截图左上角的全部突出点直接归因于深度异常。** 本轮确实发现稀疏的远距返回，同时也确认大部分ESDF点表示自由空间，而不是障碍物表面。两种现象必须分开判断。

具体结论：

1. 截图显示的是observed模式的三维距离场，包含已观测自由空间。体积状或条状延伸不自动等于“多重建了物体”。
2. 保存的原生地图中确有小的孤立观测区域，部分没有surface site、没有内部体素，应优先按自由空间采样或历史观测检查。
3. 补采59帧深度包含少量1～2.05m的远距返回；其端点均在配置工作区之外，没有贴近上一轮保存TSDF表面，**不支持将它们直接认定为截图多余表面的同一来源**。
4. 大于2.05m的原始深度会被当前backend置0，不能直接作为当前参数下的有效表面融合。范围内远点仍可能通过射线前段影响工作区内的自由空间观测。
5. 没有同一时刻的图像、位姿、地图更新前后记录及该突出区域的明确三维ROI，不能确认截图中的“漂移”究竟是历史残留、真实多余表面、位姿投影偏差还是距离场显示范围变化。

## 2. 图片和数据不是同一坐标、同一时刻

![用户ESDF截图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/user_esdf.png)

![用户深度显示截图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/user_depth.png)

RGB/深度画面位于相机像素坐标，RViz距离场位于base_link坐标并经过观察视角旋转。**深度图左上角不等于RViz三维画面的左上角。** 不能只根据两个窗口方位相近判断二者对应。

本轮使用两组可核查数据：

- 保存地图：上轮实机约21:47保存的`real_realtime_record_20261111/map.blox`和mesh.ply。
- 新补采：约21:53，59帧原始16UC1深度、CameraInfo.K及每张深度图精确时间戳的base_link→Depth optical变换，约2.03秒。

补采期间相机和TF可用，但Mapper保存服务已不可用，无法取得同一时刻新地图。故分析明确保留时间差，不能将新深度作为截图当时的逐帧因果证据。

## 3. 深度数据中实际有什么

| 检查项 | 结果 |
|---|---:|
| 有精确时刻TF的深度帧 | 59 |
| 全帧无效 | 0 |
| 有效像素比例P5/P50/P95 | 83.68% / 83.77% / 83.85% |
| 每帧>2.05m像素数P50/P95 | 65 / 111.2 |
| 含1～2.05m像素的帧 | 14 / 59 |
| 1～2.05m累计像素观测 | 350（跨帧计数，不是350个独立物体） |
| 单帧1～2.05m像素最多 | 57 |
| 有效深度最大值在各帧间的范围 | 0.736～9.396m |
| 相机平移各轴变化范围 | 约0.029 / 0.029 / 0.041mm |

相机短窗口位姿基本静止，但深度边缘仍出现零散长距离返回。长距离返回是候选现象，**未通过实物距离真值确认的点不能一律判作坏点**：它们也可能看到台面边缘之外的背景。

![深度长距离返回](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/depth_long_returns.png)

左图采用固定0.2～0.8m显示，范围外会达到色标端点；中图红色为1～2.05m，洋红色为>2.05m，右图为每帧数量。该图不补洞、不更改数值。

用户的灰度深度图整体很暗，不能因此认定没有有效数据。之前保存帧中少量5m以上远点已经足以把主体半米区域压到很低灰度；显示归一化只改变亮度，不会改变Mapper收到的16位距离值。

## 4. 这些深度是否能进入融合

[backend.py:76](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:76)做解码、毫米转米和范围过滤：

```text
深度<=无效条件或<min_depth_m或>max_range_m
    → 置0
    → 核心TSDF不将其作为有效表面深度
```

实际范围为0.2–2.05m，因此：

| 原始深度 | 当前处理 | 对突出区域的解释力 |
|---|---|---|
| 0/无效 | 跳过有效表面融合 | 本身不会凭空生成远表面，但旧状态可能保留 |
| >2.05m | backend置0 | 不能直接用9.396m原值解释当前融合中的远表面 |
| 1～2.05m | 通过范围过滤 | 若投影、权重及块选择也通过，可影响地图 |
| 正常近场值 | 正常融合 | 仍可能因标定、运动时序或表面边缘采样造成偏差 |

对补采的350个范围内长返回，使用各帧真实TF反投影后：

- 世界端点x约1.280～2.056m，z约−1.051～−0.412m。
- 端点都在当前配置AABB之外：x[−0.1,1.1]、y[−0.6,0.6]、z[−0.05,1.05]m。
- 没有端点距保存的TSDF近零带≤2cm。

所以，这批新观测**没有直接证据对应保存地图中的那片多余表面**。但不能推出完全无影响：射线从相机到端点的前段可能穿过工作区，块选择和TSDF自由空间积分仍可能使范围内的一段空间成为observed；边界还按4cm体素块选取，而不是逐点精确裁切。

![远距返回的世界坐标和射线](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/long_return_rays_xy.png)

该图比较的是不同时间的数据，红线仅示例视线，不代表所有线都生成了障碍物。范围内长深度不等于正确，端点在范围外也不等于射线前段完全不参与。

## 5. ESDF里突出的是表面，还是自由空间

读取原生BLOX，未用查询点重新生成地图：

| 项目 | 保存地图结果 |
|---|---:|
| observed ESDF体素 | 1,171,455 |
| 正距离比例 | 87.11% |
| TSDF权重≥0.1的体素 | 1,171,455 |
| TSDF近零带abs(distance)≤5mm | 52,039 |
| ESDF按6邻接的连通区域 | 255 |
| 最大区域体素 | 1,170,847 |
| 其余254个小区域合计 | 608，约0.052% |

正距离通常表示物体外的净空采样，并非该位置有一个物体。TSDF近零带才更接近表面检查，但它也不是毫米级真值。

举例，一个保存的孤立ESDF区域有112体素，site=0、inside=0，距离约1.12～11.58cm；投影到新深度时，有效投影的92个体素均在当前表面前方超过2cm。这是**自由空间采样区域**的证据，不能把它当成112个漂浮障碍物。它只是一个数值示例，未证明与截图指定的左上条带逐像素一致。

另一个38体素小区域中，37个体素位于当前可见表面后方超过2cm，说明地图中存在本次新视角不支持的历史区域候选；由于时间差及遮挡，仍不能仅据此判定错误几何。

![同一世界坐标中的ESDF、TSDF和新深度](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/esdf_tsdf_depth_xy.png)

左图是完整observed距离场的XY投影，中图是保存TSDF近零带，右图是新深度的表面端点。它们展示不同物理量：**自由空间体积本来就不应与深度表面轮廓完全重合**。左图颜色上限0.2m仅为查看，原始2m值未删除。

不能把255个连通区域说成255个障碍物；自由空间、unknown间隔、稀疏观测都能影响连通性。Mesh按顶点索引的碎片统计也会受分块边界重复顶点影响，本轮不拿它作为物理漂移数量。

旧Mesh与新深度在222,473条共同有效射线上的轴向差，绝对值中位数1.66mm、P95约8.78mm。这仅表示整体数据间的一致性，**不是实机真值精度，也不能掩盖局部突出异常**。

## 6. 如果确实是多余表面，哪些链路可能造成

### 6.1 范围内远距跳点与累计历史

少量范围内的错误远点如果进入融合，可能在原表面后面建立错误零面，或在射线前方增加observed自由空间。一次错误深度涉及的不止一个最终显示点，因此少量像素有可能在累计地图里形成更明显的条带。

目前只做了范围过滤；没有在backend中显式执行跨帧深度一致性、孤立点过滤或深度边缘拒绝。不能靠提高融合频率保证离群深度自动消失。

### 6.2 无效帧不会创建新表面，但可能不清除旧表面

固定核心[TSDF更新模板](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_tsdf_integrator_impl.cuh:25)对无效深度返回；当前invalid_depth_decay_factor=−1，未启用无效深度衰减。decay_hz和clearing_hz也都是0。

此外，体素位于当前测得表面后方超过负截断带时，更新直接跳过。因此，后续较近的正确表面不一定清掉后方曾经错误融合的历史表面。无效像素和真实遮挡也可能使历史区域持续保留。这能解释“当前深度看起来正常，地图还有旧尾巴”的机制，但本轮没有逐次更新日志证明具体条带就是如此形成。

### 6.3 位姿与标定

本次59帧相机平移变化不到0.05mm，不支持这一小窗口发生了明显的随机位姿漂移；但不能排除更早手拖运动时的时间匹配误差、固定手眼外参误差、机械臂运动学误差或安装松动。

此前TF域未连通时，代码根本没有执行有效融合；那段“无TF”不能直接解释成“用错误TF融合出了重影”。应把TF缺失和TF数值错误分开。

### 6.4 距离场传播与显示

孤立自由体素、有距离上限值的区域及observed显示范围，与真正几何占据不是同一问题。先确认该区域是否存在TSDF零面、Mesh和site，再检查ESDF传播；不能将所有ESDF形状异常都归因于深度硬件。

## 7. 建议的下一步验证和方案

| 顺序 | 操作 | 判据 |
|---|---|---|
| 1 | RViz单独看Mesh，再看ESDF近表面带，暂不叠加 | 突出若只存在于大正距离体素，优先解释自由空间显示；仍需单独检查距离可信度 |
| 2 | 在base_link中明确圈选突出区域，记录xyz边界 | 避免旋转窗口后“左上角”指代改变 |
| 3 | 同步保存该区域形成前后的深度、CameraInfo、TF及地图 | 确认首次出现对应哪几帧，而非不同时间截图推测 |
| 4 | 用相同输入分别做单帧、静止短融合和手拖融合 | 单帧有异常优先查输入/投影；运动后才有则重点查时间和外参 |
| 5 | 对已确认离群点比较空间/时间一致性过滤 | 同时统计薄物体损失、unknown和耗时，避免粗暴滤掉细结构 |
| 6 | 验证局部地图衰减/清理，保留对照原图 | 解决历史残影时不误删被遮挡的真实物体 |

为临时区分显示与表面，可以只改显示选择，不改变5mm地图计算：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh

# Mapper在运行时才执行；先记录原参数
ros2 param get /ur7e_nvblox esdf_view_mode
ros2 param get /ur7e_nvblox esdf_view_band_m

# 示例：仅查看距离零面约2cm内的ESDF带
ros2 param set /ur7e_nvblox esdf_view_band_m 0.02
ros2 param set /ur7e_nvblox esdf_view_mode band
```

这只能分辨哪些点属于远离表面的自由空间，**不修复错误距离或伪表面，也不能因突出点被隐藏就宣布问题解决**。本轮没有执行上述参数修改。查看结束恢复此前的band值和observed选择。

## 8. 数据及复核

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111`。

- [capture.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/capture.json)：采集情况，记录当时Mapper保存服务不可用。
- [depth_pose_samples.npz](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/depth_pose_samples.npz)：59帧真实深度、时间戳、K和对应TF。
- [analysis.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/analysis.json)：原生距离场、TSDF与深度对照统计。
- [long_return_trace.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/long_return_trace.json)：范围内长返回的世界坐标和端点关联。
- [analyze.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/analyze.py)、[trace_long_returns.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_esdf_protrusion_20261111/trace_long_returns.py)：离线分析脚本。

当前证据支持“深度数据、历史融合和自由空间显示都需要检查”，不支持“相机坏了”或“只改ESDF颜色就修复”的结论。要确认截图指定突出区域的唯一来源，仍需同帧、同三维ROI的更新追踪。
