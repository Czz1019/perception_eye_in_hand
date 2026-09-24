# prepare-view后RViz退出：修正与运行说明

本次只读取已有文件/日志并修改代码，没有构建、运行或测试。

## 已有证据与修正

2026-09-24 12:48:24启动的launch日志记录rviz2退出码-11；系统崩溃报告也记录Signal 11，时间12:48:43，没有可用调用栈。用户贴出的CUDA epoch=1初始化是正常重建日志，发生在退出记录之后，不能据此判断CUDA或重置是根因。

针对Mesh显示链路修改：

1. 后端Mesh数组在持有后端锁、Mesh对象仍存活时完成独立CPU拷贝，避免返回值依赖底层Mesh存储生命周期。
2. 不再每次Marker快照执行DELETEALL；保持稳定namespace/ID，正常更新同ID，只有网格缩小/重置才显式删除多余ID。
3. 发布前剔除越界索引、非有限坐标和退化三角形。不是抽样减面，保留全部有效面。
4. 每个Marker由10000降到2000个三角形，缩小单个渲染对象。
5. `--prepare-view`默认只调整观察姿态并等待地图，不再自动清图；需要清图时显式加`--reset-after-view`。完整exercise-simulation测试仍按原流程清图。

上述修正未运行确认。没有堆栈，不能断言已排除所有RViz/Ogre/显卡驱动问题；也没有通过禁用Mesh冒充修复。

## 由你执行的运行步骤

先在原mapper启动终端Ctrl+C。RViz退出不表示mapper退出，不要重复启动第二个mapper。如果场景仍正常，可保留场景。

构建：

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
```

每个终端统一环境：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
source ./source_gazebo_env.sh
```

没有运行场景时，终端一启动（已有正常场景不要重复启动）：

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

终端二启动新mapper和RViz：

```bash
./run_gazebo_mapping.sh run_rviz:=true
```

终端三移动观察姿态，默认保留累计地图：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

确认视角合适后，只有希望重新开始融合时才自行清图：

```bash
ros2 service call /realtime_nvblox/reset_map std_srvs/srv/Trigger '{}'
```

也可主动要求姿态设置后清图：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --reset-after-view
```

不需要修改外参，也不需要恢复大球点云或稠密网格。

## 若仍退出，由你保留的诊断信息

- launch中真正的rviz2退出行和退出码，而不是只有CUDA初始化信息。
- stats中的error/mesh_error，以及mesh_triangles是否异常。
- RViz关闭Reconstructed Mesh显示是否仍会退出，可用于区分Mesh渲染和其他显示；这只是定位步骤，不代表完整功能已修复。
- 对应时间的 `/var/crash/_opt_ros_humble_lib_rviz2_rviz2.1000.crash` 和launch日志。若无调用栈，不能仅凭-11区分Ogre、GPU驱动或其他插件。

旧版说明中“prepare-view会自动清图”和“每次Mesh DELETEALL”不再适用，以本文为准。本次没有代你执行任何启动或验证。
