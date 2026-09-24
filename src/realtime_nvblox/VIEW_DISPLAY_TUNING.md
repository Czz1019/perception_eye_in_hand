# 再次抬高相机与中等大小ESDF显示

本次只改文件，未构建、启动、移动机械臂或运行验证。新姿态没有验证实际高度、视野或运动碰撞。

## 参数位置

1. `src/realtime_nvblox/examples/verify_gazebo_mapping.py`：main()中 `--view-lift-rad` 的default由原先固定0.25rad改为0.40rad。它是相对最初低视角的肘关节偏移，wrist_1做反向补偿，旨在抬高相机并保持俯视方向；不是米数，不是相机安装外参。范围0到0.6是本脚本允许的调整区间，不是碰撞安全认证。
2. `src/realtime_nvblox/rviz/gazebo_esdf.rviz`：`ESDF near obstacles (15 cm)`下Style=Spheres、Size (m)=0.008、Alpha=0.85。即8mm显示大小，介于之前18mm球和2像素点之间。球的显示尺寸不等于地图体素尺寸，随视角远近占用不同屏幕大小。
3. `src/realtime_nvblox/config/gazebo_robot.yaml`：near_obstacle_distance_m=0.15仍控制表面附近显示范围；voxel_size_m仍控制地图分辨率。本次没有改变它们，也没有增加ESDF话题。

新默认目标（rad，顺序shoulder_pan/shoulder_lift/elbow/wrist_1/wrist_2/wrist_3）：
`[0, -1.57, 0.70, -0.50879335, -3.21580312, -0.69520875]`。

## 已有场景和mapper正常运行时

不需要重复启动场景，也不需要为了脚本参数重新编译。新的终端先使用同一环境：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
source ./source_gazebo_env.sh
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

默认不清图。由你在Gazebo观察移动过程及相机画面；脚本预设未经本次验证。

想比默认低一点，使用0.30；返回上一版抬高量，使用0.25：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --view-lift-rad 0.30
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --view-lift-rad 0.25
```

以上是可选命令，选择一条，不必连续执行。`--original-view`仍返回最初低姿态并忽略lift参数。球体方向可加`--view sphere`。

## 应用新显示

正在运行的RViz不会自动重新读取文件。在RViz菜单File/Open Config打开：

`/home/czz/perception_eye_in_hand/src/realtime_nvblox/rviz/gazebo_esdf.rviz`

或者直接在当前显示项中手动设置：

```text
Style: Spheres
Size (m): 0.008
Alpha: 0.85
Color Transformer: RGB8
```

仍显大可试0.006，仍不明显可试0.010。不要通过增大voxel_size_m调整显示球大小。Mesh会遮挡内部球点，检查距离颜色时可临时取消Reconstructed Mesh勾选。

若要永久保存当前RViz手动设置，保存到自己的配置文件；避免不小心将旧重复PointCloud2显示项写回默认配置。

尚未启动时，按NEAR_OBSTACLE_VIEW.md启动场景和mapper，再执行上述姿态命令。若工程非symlink安装，需自行构建以更新安装目录配置；直接打开源码中的rviz配置不依赖安装目录是否更新。
