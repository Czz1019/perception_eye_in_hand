# 实验台复杂场景：构建与运行

2026-11-10。本次扩展仿真环境，用于观察 TSDF、Mesh、3D ESDF 在实验器具、细杆、孔洞、遮挡和多种颜色下的表现。

## 场景内容

| 部分 | 几何与用途 |
|---|---|
| 工作台 | 1.4 × 1.1 m，高 0.82 m；台面厚 5 cm，四条桌腿和横梁 |
| UR7e + D435 | 保留已有机器人、相机模型、640×480 输入和手眼变换；以已有俯视观察姿态初始化，不自动环绕 |
| 蓝色试管架 | 3×5 孔位，13 支试管，2 个空孔；双层细杆与真实开孔的托环 |
| 白色试管架 | 2×4 孔位，6 支试管，2 个空孔 |
| 试剂瓶 | 3 个不同高度、颜色和半径的瓶体，带瓶颈、瓶盖和标签 |
| 样品托盘 | 有底板与边框，内放 6 个带彩色盖的小样品瓶 |
| 锥形器皿 | 用分段圆柱近似变化截面的不透明器皿 |
| 尺度参照块 | 高度分别为 2、4、6、10 cm，便于检查细节损失与尺度 |
| 总览相机 | 独立 1280×800、2 Hz，供查看整个工作台；不接入建图 |

总计 177 个静态几何部件。各部件的 visual 与 collision 使用相同尺寸和局部位姿。器具采用不透明材料，以测试几何和颜色融合；没有模拟透明玻璃的红外透射、折射和失效行为。试管架 STL 是有真实内孔的闭合环形网格，不是用颜色画出孔洞。

## 坐标与初始姿态

台面上表面为 `world/base_link z=0`，地面在 `z=-0.82 m`。这样机械臂底座仍在原点，保持已有机器人坐标和手眼标定；只是地板下移、桌腿向下延伸。台面中心为 `(0.35, 0)`，范围为 x=[-0.35,1.05]、y=[-0.55,0.55] m。

`lab_initial_positions.yaml` 使用项目已有的俯视观察关节姿态，Gazebo 生成机器人时直接初始化到该姿态。本启动项不发送扫描运动命令。启动时相机与机械臂安装几何检查的最小间距为 12.14 mm；这只是安装部位检查，不代表已经完成所有机械臂姿态的工作台碰撞验证。

## 修改位置与结构

以下均在 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand` 内：

- `src/ur7e_d435_gazebo/config/lab_scene.yaml`：台面、器具位置、尺寸与颜色。
- `config/lab_initial_positions.yaml`（同一场景包内）：复杂场景初始关节姿态。
- `scripts/lab_scene.py`：生成静态模型 SDF；visual/collision 共用几何定义。
- `scripts/spawn_lab_scene.py`：通过 Gazebo `/spawn_entity` 加入完整工作台。
- `meshes/lab/*_ring.stl`：试管架开孔托环。
- `worlds/lab_workbench.world`：低位地面、灯光、GUI 总览视角、总览相机。
- `launch/scene.launch.py`：新增 `scene_variant:=lab`；lab 模式选择工作台 world 和初始姿态，移除旧零高度地板的几何。旧模式 `tall` 保留。
- `launch/lab_scene.launch.py`：仅场景启动入口。
- `src/ur7e_nvblox_in_hand/launch/bringup.launch.py`：转发 `scene_variant`，使场景、建图、RViz 可一起启动。
- `CMakeLists.txt`：安装新 world、脚本与模型；`test/test_lab_scene.py` 检查配对几何及孔环闭合性。

另外，场景启动显式清空 Gazebo 的 `profile` 参数，避免上层 mapper 的 `profile:=realtime` 被 Gazebo 当作物理引擎预设名。本次没有改动融合算法。已有 2 cm 体素、4 体素截断设置沿用之前修改。

## 编译

在一个新终端运行：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
colcon build --base-paths src \
  --packages-select ur7e_d435_gazebo ur7e_nvblox_in_hand \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF
source ./env.sh
```

## 方式一：完整场景 + Python 建图 + RViz

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab start_scene:=true use_sim_time:=true \
  launch_camera:=false gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 esdf_mode:=3d
```

这里只启动仿真相机，不启动实际 D435，也不连接实机。无需启动环绕扫描脚本。Gazebo 可查看工作台原始场景；RViz 查看融合后的地图。RViz `Fixed Frame` 设为 `base_link`，使用 `Move Camera` 工具拖动视角。静止视角下仍会存在被机械臂或器具遮挡的区域，不能把它们当作已观测自由空间。

## 方式二：仅查看场景

与方式一二选一，避免重复占用 Gazebo 端口：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_d435_gazebo lab_scene.launch.py gazebo_gui:=true
```

如果之后需要接入另一个建图实现，应保持此场景运行，在另一个终端启动其 mapper，设置 `start_scene:=false`、`use_sim_time:=true`，使用同一个 ROS 域和 `/camera/...` 输入。此场景的规范源码在 Python 工作区的场景包中；没有在 C++ 工作区再复制一份同名场景包。

## 话题检查

另开终端，先运行与启动终端相同的环境设置：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 topic list -t
ros2 topic hz /camera/depth/image_rect_raw --wall-time
# Ctrl+C 后再执行下一项
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

| 话题 | 内容 |
|---|---|
| `/camera/color/image_raw` | D435 仿真 RGB，640×480 |
| `/camera/color/camera_info` | RGB 内参 |
| `/camera/depth/image_rect_raw` | D435 仿真深度，640×480，16UC1 |
| `/camera/depth/camera_info` | 深度内参 |
| `/lab/overview/overview/image_raw` | 整个工作台的独立总览画面，1280×800 |
| `/lab/overview/overview/camera_info` | 总览相机内参 |
| `/ur7e_nvblox/mesh` | Python mapper Mesh，MarkerArray |
| `/ur7e_nvblox/esdf_points` | 已观测 3D ESDF 点云 |
| `/ur7e_nvblox/tsdf_layer` | TSDF 层点云 |
| `/ur7e_nvblox/color_layer` | 颜色层点云 |
| `/ur7e_nvblox/camera_path` | 来自机械臂 TF 的相机轨迹 |
| `/ur7e_nvblox/metrics` | Python mapper 的融合与处理统计；不是 `/stats` |

查看总览可在 RViz 增加 `Image`，选择总览 image_raw 话题。也可运行：

```bash
ros2 run rqt_image_view rqt_image_view /lab/overview/overview/image_raw
```

结束时在启动终端按 Ctrl+C。再次启动前确认原 gzserver 已退出；若端口被占用，先用 `ss -ltnp 'sport = :11566'` 确认所属进程，结束上一轮启动进程后再启动。

## 验证与实际画面

已完成：两个包编译、Python 语法检查、几何测试（1 项通过）、实际 Gazebo 生成、RGB/深度/总览图像接收及 mapper 非空输出检查。网格托环通过闭合性、朝向和拓扑检查。原始数据保存在 `artifacts/lab_scene_20261110`，其中 `output_check.json` 保存建图输出统计：一次采样收到 222405 个 ESDF 点，Mesh 含 66273 个三角形顶点（22091 个三角形）。这是当时地图状态下的输出规模，随观察过程变化。

本轮是场景接入检查，不作为稳定实时性或重建精度的性能基准。截图为实际 Gazebo 相机图像，不是重建质量的渲染预测。

![场景总览](../artifacts/lab_scene_20261110/overview.png)

![腕部相机 RGB](../artifacts/lab_scene_20261110/rgb.png)

![腕部相机深度灰度显示](../artifacts/lab_scene_20261110/depth.png)

2 cm 体素无法完整表达所有 6 mm 栏杆及毫米级间隙。评估时应将大物体的表面正确性、细结构的分辨率损失、遮挡导致的 unknown 分别分析，不能因为增加了复杂器具就预期所有孔洞都会在 Mesh 中保留。场景仍沿用原相机的深度噪声配置；本次没有同时调整噪声或映射参数。
