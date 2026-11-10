# 深度直接反投影点云与Mesh轮廓对照测试

日期：2026-11-10。没有修改现有感知源码、配置或融合参数。本轮新建的测试材料位于artifacts中。当前实机ROS域135没有相机或Mapper，因此选择项目自带仿真，并使用独立域136、Gazebo端口11566；没有操作真实机械臂。

## 1. 实际执行步骤

1. 启动640×480仿真场景。初始相机朝向地面，不适合检查障碍物，因此仅将虚拟机械臂移动到tools/motion_plan.json已有center_joints观察姿态。
2. 等虚拟机械臂停稳后，清空本轮测试Mapper地图，保证不混入接近过程的旧几何。
3. 采集时间戳相同的深度图和CameraInfo，同时读取该图像时间的base_link→camera_depth_optical_frame。
4. 不经过TSDF、ESDF或Mesh，直接按针孔模型逐像素反投影，保存307200点的PLY。
5. 保持仿真静止，用现有Python Mapper、v=0.03m融合并导出Mesh。
6. 把全局Mesh通过图像时刻TF的逆变换转到depth optical坐标系，用同一K、同一像素光线渲染Mesh深度，与原始深度做数值对照。
7. 在CloudCompare中打开原始点云与同坐标系Mesh，便于独立查看。

本轮只验证该静止仿真场景，不是用户昨日实机灯板场景的重现。

## 2. 直接反投影公式

```text
Z = 16UC1深度 × 0.001（米）
X = (u-cx) × Z / fx
Y = (v-cy) × Z / fy
```

输出保留在camera_depth_optical_frame：X向右、Y向下、Z向前。只保留有限且在0.1～3m内的像素，不做平滑、补洞、体素降采样或跨帧融合。图中为绘图速度抽取部分点，但PLY保留全部有效点。

实际K：fx=fy=320.2549133，cx=320，cy=240。编码16UC1，640×480。

## 3. 测试数据与结果

| 项目 | 实测 |
|---|---:|
| 深度像素总数 | 307200 |
| 0.1～3m有效深度点 | 307200，100% |
| 光轴深度范围 | 0.250～0.984m |
| Mesh顶点 | 3006 |
| Mesh三角面 | 4306 |
| 同视角Mesh与有效深度重叠像素 | 260237 |
| Mesh对有效深度像素的覆盖率 | 84.71% |
| 未覆盖比例 | 15.29% |
| 重叠区域绝对深度误差中位数 | 0.335mm |
| 重叠区域绝对深度误差P95 | 17.79mm |
| 重叠区域绝对深度误差均值 | 5.76mm |

原始点云能够看到两块障碍物、地面及机械臂轮廓。图像中目标较近，且包含机械臂自体；本轮没有加入自体遮罩。

在同一相机视角下，Mesh主要结构与输入深度对应，没有复现整体位置大幅错乱。缺面及较大残差集中在深度不连续边缘、图像边界及部分曲面区域；当前证据应重点追查Mesh完整性与边界行为，而不是直接认定全部几何投影错误。

重要限制：误差仅在双方均有有效表面的重叠像素计算，不能用很小的中位数掩盖15.29%的缺失区域；这些数值是对本轮输入深度的一致性，不是相对于真实场景真值的绝对精度。像素光线按采集K和整数像素中心定义，半像素约定也可能影响边缘残差。

RGB与深度来自不同光心，不能以RGB像素直接逐点配色证明外参正确。本次点云不附加未经遮挡校验的RGB颜色，显示色主要表达深度。

## 4. 图片

![RGB与原始深度](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/rgb_depth.png)

![原始反投影点云多视角](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/raw_cloud_views.png)

![相同相机视角下的深度、Mesh与误差](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/depth_mesh_same_view.png)

## 5. 用CloudCompare查看

```bash
CloudCompare \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/raw_depth_cloud.ply \
  /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/mesh_depth_optical.ply
```

两者已在同一depth optical坐标系，直接叠加；交替勾选点云和网格查看。不要对其中一个再次应用base_link变换，不需要ICP或人为移动对齐。PLY不依赖ROS持续运行。

## 6. 输出材料

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/depth_cloud_check_20261110/`

- raw_depth_cloud.ply：无融合、无降采样的307200点原始深度点云。
- points_depth_optical.npy / depth_m.npy：点坐标与米制深度。
- capture.json / T_base_depth.npy：内参、有效点统计、帧名及图像时刻位姿。
- rgb.png / rgb_depth.png：输入截图。
- mesh.ply：Mapper全局坐标下的网格。
- mesh_depth_optical.ply：只作刚体坐标转换的同相机坐标系网格。
- mesh_rendered_depth_m.npy / comparison.json：投影深度及数值结果。
- raw_cloud_views.png / depth_mesh_same_view.png：独立渲染图。
- scene.log / mapper.log / mapper_stats.json：运行记录。
- capture_depth_cloud_check.py / compare_depth_mesh.py：本轮采集和离线对照脚本副本，路径和域为本实验固定值，并非新安装的通用功能包。

## 7. 对后续排查的意义

本轮支持“应以原始深度点云为中间基准”的方法，但不支持宣称昨日实机问题已经解决。下一步应对实机灯板重复同样采集，保证同时记录实际CameraInfo和图像时刻TF；再分别度量原始深度覆盖、TSDF有效角点、Mesh缺面以及颜色可见性。此次原始点云轮廓较完整而Mesh缺面，提供了追查融合/网格完整性的明确方向。
