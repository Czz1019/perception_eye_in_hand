# 多帧重建 Mesh 与 ESDF 显示：用户运行和验收步骤

RViz退出修正及最新运行方式见 [RVIZ_CRASH_FIX.md](RVIZ_CRASH_FIX.md)：prepare-view默认不再清图；Mesh改为稳定ID更新。本次未运行验证。

最新默认显示已收敛为一个ESDF近障碍小点话题，观察姿态已调整；请先看 [NEAR_OBSTACLE_VIEW.md](NEAR_OBSTACLE_VIEW.md)。本次未运行验证。

本次仅新增/修改代码与配置，未构建、启动、执行测试或测量性能。以下操作由你自行执行。既有性能报告不能代表本次开启Mesh后的速度。

## 新增内容与区别

- `/realtime_nvblox/mesh`：从当前累计TSDF提取的完整三角网格，MarkerArray，默认启用、最多2Hz更新。有Mesh订阅者才提取。不是当前单帧点云，不另建地图，不运行SLAM。
- `/realtime_nvblox/save_mesh`：保存同一地图的完整Mesh到 `base_link_mesh.ply`。即使没有Mesh订阅者或关闭周期提取，也可主动保存。
- `/realtime_nvblox/esdf_clearance`：已观测ESDF的分段距离颜色；原始distance字段不变。
- `/realtime_nvblox/esdf_surface`：`abs(distance) <= voxel_size_m`的近表面体素，辅助定位障碍物。它是一个体素厚度的近似带，不是精确表面；精确重建形状看Mesh。
- `/realtime_nvblox/esdf_legend`：距离颜色文字图例，1Hz发布。
- 原 `/esdf_3d`、`/esdf_observed`、地图保存及严格时间戳TF逻辑保留。

颜色默认含义（米）：

| 颜色 | ESDF距离 | 含义 |
|---|---|---|
| 紫 | d < 0 | 重建障碍物内部 |
| 红 | 0 <= d <= 0.05 | 表面附近 |
| 橙 | 0.05 < d <= 0.15 | 距离较近 |
| 黄 | 0.15 < d <= 0.30 | 中间距离 |
| 蓝 | 0.30 < d <= 0.50 | 显示范围内较远的已观测区域 |
| 不显示 | unknown或超出显示带 | 不代表安全空间 |

这些阈值是可视化距离，不包含机械臂半径、停止距离、定位误差等因素，不能直接当作安全判据。

Mesh为不透明浅灰蓝色，不使用相机RGB纹理。每次完整Marker快照先DELETEALL再添加所有块，处理网格缩小、空网格和重置后的旧三角形残留；没有跳着抽取三角形来制造显示孔洞。相机没看到的背面/遮挡区域仍然可能缺面。

## 配置

默认启动读取 `src/realtime_nvblox/config/gazebo_robot.yaml`；只有显式传入config参数才换文件。本次保留你当前的 `voxel_size_m: 0.05`、`esdf_hz: 30.0`，新增：

```yaml
enable_mesh: true
mesh_hz: 2.0
clearance_thresholds_m: [0.05, 0.15, 0.30]
```

Mesh从同一累计TSDF提取，但更新频率低于ESDF，两者显示时间可能略有差异。默认2Hz是仿真时间上限，不保证墙钟2Hz。提取仍占用同一GPU工作线程；Marker打包在独立latest-only发布线程，依旧可能增加CPU/GPU开销。若影响融合速度，先改mesh_hz为1.0，或在RViz取消Mesh勾选。

5cm体素意味着表面有离散化误差，小物体也可能不精细；仅改变RViz点大小不能提高重建分辨率。需要更细表面时可自行尝试3cm体素，但应重新计量性能。

## 1. 自行构建

先在旧启动终端Ctrl+C结束旧mapper和场景，再执行：

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
```

若构建失败，先保留错误，不继续启动。没有必要删除整个build/install目录。

## 2. 每个终端统一环境

三个终端分别执行：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
source ./source_gazebo_env.sh
```

## 3. 启动

终端一：

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

终端二：

```bash
./run_gazebo_mapping.sh run_rviz:=true
```

终端三，只有需要改变初始朝向时：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

这是你主动执行的仿真关节移动/清图操作，不修改外参。若服务发现失败，按RVIZ_TROUBLESHOOTING.md排查，先确认场景和mapper同域。

## 4. RViz观察顺序

加载本版 `src/realtime_nvblox/rviz/gazebo_esdf.rviz`，Fixed Frame=base_link。

### 先单独看重建

勾选 `Reconstructed Mesh`，取消 `ESDF Clearance Bands`、其他ESDF点云，保留RobotModel与Camera RGB。

观察墙面是否完整平整，球体是否成形，地面是否稳定。Mesh是累计表面，颜色不会还原原始RGB。相机看向新区域后，Mesh应补充新表面。

### 再单独看距离场

取消Mesh，勾选 `ESDF Clearance Bands` 和 `ESDF Distance Legend`。

颜色应根据距表面的距离分段变化；勾选 `ESDF Near Surface (within one voxel)`可辅助定位红色近表面体素。不要同时开启多种重叠点云，否则可能遮挡颜色。新版默认用RGB8读取点云内颜色；不要改成按Z着色。

### 最后叠加

同时勾选Mesh和Clearance Bands，观察距离带是否围绕重建表面。内部紫色点可能被不透明Mesh遮住，查看内部时关闭Mesh。图例位于base_link中左侧上方，可移动视角查看。

手工添加时：Mesh/Legend使用MarkerArray、Reliable、Volatile、队列1；Clearance/Surface使用PointCloud2、Best Effort、Volatile、队列1。

## 5. 你可以执行的功能验收

查看统计：

```bash
ros2 topic echo /realtime_nvblox/stats --once
```

预期 `mesh_updated` 和 `mesh_published`增加，看到表面后 `mesh_triangles > 0`。`mesh_stamp_ns`是Mesh最近融合输入的时间戳；`mesh_update_readback`为提取/读回阶段耗时。若存在 `mesh_error` 或 `error`，保留完整异常。

有订阅者才能触发Mesh周期提取，单看topic存在不能证明网格非空。RViz勾选Mesh后观察，或自行执行：

```bash
ros2 topic hz /realtime_nvblox/mesh
ros2 topic hz /realtime_nvblox/esdf_clearance --qos-reliability best_effort
```

两条分别执行，用Ctrl+C结束。Marker数量不是三角形数量，以stats中的mesh_triangles为准。

保存重建：

```bash
ros2 service call /realtime_nvblox/save_mesh std_srvs/srv/Trigger '{}'
```

默认保存到 `~/nvblox_output/realtime_nvblox_gazebo/base_link_mesh.ply`，具体以服务响应为准。它是完整后端Mesh，不是RViz截图。可用你已有的MeshLab/CloudCompare检查表面；本次没有安装查看器。

清图：

```bash
ros2 service call /realtime_nvblox/reset_map std_srvs/srv/Trigger '{}'
```

旧Mesh应删除，随新有效深度重新出现；若只是检查旧图是否清掉，需要注意新深度还在不断重建。不要用Gazebo全局reset替代日常清图。

## 6. 你可以执行的运动与性能比较

先关闭ESDF，只看Mesh，再由你选择运行已有的小幅仿真运动脚本：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --exercise-simulation --output /tmp/mesh_observation_run.json
```

这条脚本会移动仿真机械臂、保存地图并清图，适合你同时观察Mesh；其自动断言主要针对ESDF/TF，不能替代Mesh几何质量检查。墙面若出现双层、拖尾、物体位置随相机变化，应检查位姿/时间同步与相机模型，而不是只调颜色。

分别在Mesh关闭、Mesh开启2Hz、Mesh开启1Hz时记录30秒：相机深度输入Hz、depth_integrated速率、esdf_published速率、receive_to_result的P50/P95、mesh_update_readback耗时。保持相同观察姿态和RViz其他显示项，避免不同渲染负载干扰比较。本次没有实测，不能沿用之前约18Hz结论。

## 文件改动

- gazebo_ros_node.py：新增Mesh提取/发布线程与latest-only槽、save_mesh服务、新显示话题、统计和epoch清理。
- reconstruction_visuals.py：新增完整Mesh Marker快照、分段颜色和图例。
- gazebo_robot.yaml/realsense_ros.yaml：新增Mesh及颜色阈值参数，保留原参数。
- gazebo_esdf.rviz：增加Mesh、距离带、近表面与图例显示；原ESDF显示保留为可选。
- setup.py、README和运行说明：安装和链接本说明。

新增功能未运行验证；没有更换位姿来源、启用SLAM、修改外参或重写现有后端算法。
