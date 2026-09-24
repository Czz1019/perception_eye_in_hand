# 单一 ESDF 近障碍显示与抬高观察姿态

最新高度和显示大小见 [VIEW_DISPLAY_TUNING.md](VIEW_DISPLAY_TUNING.md)：默认lift=0.40rad，显示为8mm球；下文旧的0.25rad/2像素说明已被更新。

RViz退出修正及最新运行方式见 [RVIZ_CRASH_FIX.md](RVIZ_CRASH_FIX.md)：prepare-view默认不再清图；Mesh改为稳定ID更新。本次未运行验证。

本次仅修改文件，未构建、未运行仿真、未执行姿态或验证。以下命令由用户自行执行。

## 改动

此前为了展示不同数据添加了多个ESDF话题，对日常使用确实重复。本版默认只创建并发布一个ESDF显示话题：

`/realtime_nvblox/esdf_observed`

它输出实际已观测且 `abs(ESDF distance) <= 0.15 m` 的体素中心，使用原有分段距离着色。unknown不显示；地面也是重建表面，地面附近仍会有点。这是表面附近的三维距离带，不是凭空生成、插值增密或物体识别点集。

底层TSDF/ESDF仍累积完整观测。在GPU上查询距离后过滤15cm带，再读回CPU；不会清除带外地图或仅保留当前视野。这个优化减少的是导出/传输/显示，不是将全部GPU更新局限到15cm带。

保留Mesh、camera_pose和stats：它们分别用于表面检查、位姿和状态，不是额外的ESDF场。默认不再创建esdf_3d/esdf_clearance/esdf_surface/esdf_legend；旧接口可通过 `publish_esdf_extras: true`恢复。稠密调试仍由publish_dense_debug独立控制，默认关闭。

RViz只有一个ESDF显示项，Style=Points、Size(Pixels)=2，不再用球体。Mesh默认保留开启；它可能遮住内部点，检查ESDF时可暂时取消Mesh勾选。

## 参数

`src/realtime_nvblox/config/gazebo_robot.yaml` 新增：

```yaml
near_obstacle_distance_m: 0.15
publish_esdf_extras: false
```

- 想只看更薄的表面带，可改0.05或0.10；体素仍按当前voxel_size_m离散，不会因显示带变小自动变精细。
- 想看更宽的距离场，可改0.30。
- 颜色继续沿用clearance_thresholds_m：默认15cm带主要显示红色/橙色，负距离为紫色；黄/蓝被距离带过滤并非数据丢失。
- 修改后重启mapper。RViz显示项名称中的15cm是文字标签，自定义范围后可自行重命名。
- 默认模式下stats的known_cells/total_cells描述导出的近障碍点，不再代表全部已观测体素数量。
- 启用extras后仍以esdf_observed作为近障碍显示，其他话题仅供需要的旧订阅者使用。

## 观察姿态

原命令不变：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

现在默认在旧姿态基础上将肘关节减少0.25rad，wrist_1补偿增加0.25rad，目标是将相机抬高并保持原俯视方向，扩大地面场景覆盖。不是修改手眼外参，也不是改变RViz视角。

新默认关节顺序及目标（rad）：

```text
shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
0, -1.57, 0.85, -0.65879335, -3.21580312, -0.69520875
```

只作为当前仿真场景的未验证预设；未执行碰撞检查/运动验证，不能承诺具体高度增量或所有障碍物都可见。执行时由你观察Gazebo中的平滑运动，姿态不合适可中断命令。需要回到旧预设：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --original-view
```

球体方向也使用抬高预设：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --view sphere
```

原脚本会在姿态设置后清图，使新视角重新开始融合。相机的物理视场角没有修改。

## 运行步骤

1. 从原启动终端Ctrl+C结束旧mapper和场景。旧节点继续运行时，旧话题和旧RViz显示不会自动消失。
2. 自行构建：

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
```

3. 三个终端分别设置：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
source ./source_gazebo_env.sh
```

4. 终端一：

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

5. 终端二：

```bash
./run_gazebo_mapping.sh run_rviz:=true
```

6. 终端三，待场景服务就绪后：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

7. RViz加载本版配置后，只保留`ESDF near obstacles (15 cm)`。删除或取消勾选以前手工添加的重复PointCloud2，确认Style为Points、Size(Pixels)为2。若旧会话仍显示Spheres，在File/Open Config打开工程中的`src/realtime_nvblox/rviz/gazebo_esdf.rviz`。

你可自行观察：表面附近有小点，远离表面的已知空间不显示；关闭Mesh后点不再被表面遮挡；相机调整后图像覆盖是否符合需求。没有在本次代你运行这些操作。

注意：为适配单一输出，现有验证脚本已改为订阅esdf_observed，运动比较只涉及近表面点；未运行该脚本。旧文档中默认启用多个ESDF显示项的描述，以本说明为准。
