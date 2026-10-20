# Python 稀疏 3D ESDF：运行与用户验收说明

RViz空白、相机画面缺失或终端出现`>`时，先看 [RVIZ_TROUBLESHOOTING.md](RVIZ_TROUBLESHOOTING.md)。两个启动脚本现共用根目录source_gazebo_env.sh；未显式设置时默认域78、localhost=1、Gazebo端口11478。RViz新增默认RGB和可选深度图像显示。

**本次修改未构建、未启动节点、未运行测试。以下命令由用户自行执行。**
旧版 `artifacts/gazebo_perception/` 的结果属于固定查询网格版本，不能证明本版通过，也不能作为本版性能数据。

## 1. 本版解决什么

保留现有 UR7e/Gazebo 场景及实测手眼外参，感知主体为 Python：

```text
ROS Image + CameraInfo + 图像时间戳 TF
  -> Python ROS 前端
  -> nvblox_torch CUDA TSDF / 3D ESDF
  -> 已观测体素中心的有效 ESDF
  -> PointCloud2 / RViz / 地图文件
```

nvblox内部本来就是稀疏体素块地图。旧版额外对固定长方体建立规则查询网格，查询和传输许多未知点。新版默认不生成该查询网格。

本机 Python binding 提供 `tsdf_layer_view().get_all_blocks()` 和 `query_layer(ESDF, ...)`，没有公开 ESDF layer view。本版在GPU按128个块分批读取**正权重TSDF体素**，生成原生体素中心，仅查询这些候选位置的真实ESDF。查询后在GPU剔除unknown和非有限距离，才读回CPU。不是把TSDF截断距离伪装成ESDF，也不是仅在RViz隐藏已传输的unknown。

Python负责调度和ROS接口，底层计算仍依赖nvblox/CUDA，不是纯CPU Python算法。已有 `ros_mapper`、SDK真机runtime、cuVSLAM支持不变；共享Backend新增一个导出方法，原查询方法保留。

### 输出语义变更

| 话题 | 本版内容 | QoS |
|---|---|---|
| `/realtime_nvblox/esdf_3d` | 已观测候选体素上全部有效3D ESDF，不含unknown；有订阅者才打包发送 | Best Effort / Keep Last 1 |
| `/realtime_nvblox/esdf_observed` | 上述结果中 `abs(distance)<=0.5m` 的显示带 | Best Effort / Keep Last 1 |
| `/realtime_nvblox/esdf_grid_debug` | 固定查询网格，可能含unknown；默认不存在 | Best Effort / Keep Last 1 |
| `/realtime_nvblox/camera_pose` | ESDF更新所用图像时刻的相机位姿 | Best Effort |
| `/realtime_nvblox/stats` | 输入/融合/发布频率、延迟、体素数、epoch | Reliable |

点云仍有 x/y/z/rgb/distance 字段。**点数及顺序不再固定，不能再按52111点验收，也不能把不同帧同一行当成同一位置。**读取全部ESDF的旧订阅者要改为Best Effort；需要固定数组的规划器应显式使用稠密调试接口或按需查询。

`known_cells == total_cells` 表示本次导出的全部点都是有效ESDF；`observed_cells`是近距离显示带的点数；`allocated_tsdf_blocks`是地图活动块数。`total_cells`不等于库内部全部分配体素数。

**当前不在相机视野内但过去已经观测的体素会保留。**这是地图记忆，不是unknown浪费。稀疏块边缘仍可能含未知体素，CUDA库也会预留内存；本版消除的是应用层固定空间扫描和unknown传输，不保证显存随可见点数同比下降。地图长期移动探索仍会增长。

### 和 Isaac ROS 的关系

使用同一类nvblox TSDF→3D ESDF算法与世界坐标位姿约定，但本版不是Isaac ROS的接口/性能完全等价实现，没有NITROS、增量块消息、相同RViz mesh插件或ESDF服务协议。

官方Isaac ROS ESDF visualizer本身也可通过AABB查询稠密场，所以“查询网格”不是算法错误；问题是默认大范围反复查询并显示unknown。本版选择稀疏已观测输出来解决这一问题。未经同数据集、同参数对照，不能声称效果和速度完全一致。

参考：
- https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html
- https://nvidia-isaac-ros.github.io/v/release-4.2/repositories_and_packages/isaac_ros_cumotion/isaac_ros_esdf_visualizer/index.html
- https://github.com/nvidia-isaac/nvblox

## 2. 本次性能相关改动

- 默认关闭固定长方体查询，unknown在GPU过滤后不传输到ROS。
- `/esdf_3d`使用Best Effort，并在无订阅者时跳过完整点云的CPU打包；RViz默认只看距离带。
- 关闭RealSense仿真插件额外的`depth/color/points`原始点云输出；mapper只需要Image和CameraInfo。
- 没有Gazebo原生图像订阅者时，跳过原生transport的重复消息拷贝；ROS图像仍发布。
- 保留当前用户配置的640×360深度/RGB、30Hz深度及1Hz红外，不恢复到1280×720。
- `/clock`配置为100Hz，robot_state_publisher的发布上限为100Hz，减少图像必须等待后续时钟/TF的时间；实际TF仍取决于关节状态输入。
- 深度转换显式剔除非有限值，避免NaN转换为uint16。

这些是基于源码的改动，不是已经测得的提速结论。稀疏导出仍需遍历活动块，地图很大时可能比小型固定ROI更慢。相机渲染与nvblox共用GPU，应用层处理慢也不等于传感器自身输出慢，要按第6节分开测量。

## 3. 自行构建

先在原场景/感知启动终端Ctrl+C。旧进程不会自动加载新插件、URDF或Python代码。

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
source install/local_setup.bash
```

本次包含C++仿真插件修改，因此不能仅重启Python跳过构建。不要删除整个build/install目录。若环境混入别的工程，使用新的终端构建。

## 4. 三终端运行现有Gazebo场景

**三个终端都先执行相同环境设置，之后再启动各自程序：**

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
```

### 终端一：场景

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

保持运行。性能对比时可用 `gazebo_gui:=false`，物理和传感器仍在服务器运行。若端口占用，先结束原服务器；不要同时起两个同域场景。不要只关闭Gazebo窗口而留下gzserver。

### 终端二：Python感知及RViz

```bash
./run_gazebo_mapping.sh run_rviz:=true
```

使用默认 `config/gazebo_robot.yaml`。不要继续传入旧验收目录中的test_config.yaml。

RViz应加载本版配置，Fixed Frame为`base_link`。启用`ESDF observed`；可选`ESDF all observed voxels`。关闭之前手工添加、来源不明确的`PointCloud2`项，尤其是旧稠密网格/原始相机点云。

截图中`ESDF observed`未勾选，额外`PointCloud2`勾选，但其Topic未展开，不能仅凭截图认定灰点来源。新版默认两个ESDF话题都不发送unknown。若仍出现规则灰色大盒子，先核对实际订阅话题及是否仍运行旧节点。

### 终端三：设置仿真观察姿态

初始机械臂朝外上方，可能看不到近处障碍物。只设置观察姿态：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

这会平滑移动仿真关节并清图，不改外参，不执行整套运动/性能验收。球体视角：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view --view sphere
```

脚本现在等待最多15秒发现Gazebo服务和joint_states，而不是2秒后直接assert。超时会打印ROS_DOMAIN_ID与发现的服务。

若仍提示缺少服务，由你检查：

```bash
ros2 node list --no-daemon
ros2 service list | grep -E 'pause_physics|reset_simulation|spawn_entity'
ros2 topic list | grep -E '^/clock$|^/joint_states$|^/camera/depth/'
```

必须在**场景启动前**设置域78；在第三个终端设置不会影响前两个既有进程。若gzserver退出，先处理其启动日志。存在服务命名空间前缀时，应按真实服务名调整脚本，不能删除仿真检查后对未知控制器发送命令。

## 5. 清图、保存与可选稠密调试

```bash
ros2 service call /realtime_nvblox/reset_map std_srvs/srv/Trigger '{}'
ros2 service call /realtime_nvblox/save_map std_srvs/srv/Trigger '{}'
```

默认文件：`~/nvblox_output/realtime_nvblox_gazebo/base_link_map.nvblox`。

需要旧网格做诊断时，在gazebo_robot.yaml设置：

```yaml
publish_dense_debug: true
```

重启mapper后，另行订阅 `/realtime_nvblox/esdf_grid_debug`。只有启用且有订阅者才查询，grid_size_m/grid_resolution_m/grid_center_m仅用于此功能；普通运行保持false。该诊断会增加开销。

## 6. 由用户执行的测试计划

本次没有执行以下测试。建议先只启动场景，再启动mapper，最后打开RViz；每阶段保持相同姿态采样30秒，避免把渲染开销误判为融合开销。

### A. 相机和TF基线（mapper未启动）

```bash
ros2 topic hz /camera/depth/image_rect_raw --qos-reliability best_effort
ros2 topic hz /clock --qos-reliability best_effort
ros2 topic hz /joint_states --qos-reliability best_effort
```

每条用Ctrl+C结束。记录相机实际频率；若此时已低，优先检查Gazebo渲染和并行GPU负载，而不是改ESDF。

### B. 仅启动mapper

```bash
./run_gazebo_mapping.sh run_rviz:=false
```

另一个已设置环境的终端：

```bash
ros2 topic echo /realtime_nvblox/stats --once
ros2 topic hz /realtime_nvblox/esdf_observed --qos-reliability best_effort
```

期望：error为空，depth_integrated/esdf_updated/esdf_published持续增加；output_mode为sparse_observed_voxels；有场景观测时known_cells>0；tf_missing不持续高速增加；不再要求固定52111点。

### C. 添加RViz比较

在保持mapper运行时，另一个同域终端：

```bash
rviz2 -d ~/perception_eye_in_hand/src/realtime_nvblox/rviz/gazebo_esdf.rviz \
  --ros-args -p use_sim_time:=true
```

不要再启动第二个mapper。比较深度输入频率、融合频率、ESDF频率和接收到结果P50/P95；RViz角落FPS不是深度或ESDF频率。stats延迟是最近最多512次滚动采样，包含TF/时钟等待，不是曝光到显示总延迟，也不是硬实时上界。

### D. 可选自动运动验收（只供你自行执行）

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py \
  --exercise-simulation --output /tmp/sparse_esdf_walls.json
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py \
  --exercise-simulation --view sphere --output /tmp/sparse_esdf_sphere.json
```

脚本已按空间坐标匹配稀疏体素，检查相机移动后固定世界位置的距离变化；不再假定网格顺序固定。保存/清图服务也包含在此操作测试中。

另一个没有场景的域测试严格TF丢帧及epoch恢复：

```bash
ROS_DOMAIN_ID=79 python3 src/realtime_nvblox/examples/verify_gazebo_rejection.py
```

可选单元/GPU回归：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/realtime_nvblox:$PYTHONPATH \
  python3 -m pytest -q src/realtime_nvblox/test src/realtime_nvblox/realtime_nvblox/tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_GPU_TESTS=1 PYTHONPATH=src/realtime_nvblox:$PYTHONPATH \
  python3 -m pytest -q src/realtime_nvblox/realtime_nvblox/tests/test_gpu_esdf.py
```

现有GPU单测覆盖后端融合/网格查询，不单独证明新增稀疏导出正确；必须结合运行验收。稀疏导出新增接口可能受nvblox_torch版本影响，升级绑定后需重新检查API。

## 7. 可选：真实RealSense使用ROS驱动输入

**不能把物理相机驱动用作Gazebo虚拟相机。**Gazebo仍由仿真传感器提供图像。真实相机可以经realsense2_camera发布标准ROS图像，再复用本Python前端，避免mapper直接打开SDK。

已新增 `config/realsense_ros.yaml`：use_sim_time=false；输入默认 `/camera/depth/image_rect_raw` 和 `/camera/depth/camera_info`。本机未在 `/opt/ros/humble/share` 发现realsense2_camera，本次没有安装它；需自行准备驱动及真机机器人TF。

支持下列参数的驱动版本可参考（先自行查看 `--show-args`）：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:="''" camera_name:=camera \
  enable_depth:=true enable_color:=false \
  enable_infra1:=false enable_infra2:=false \
  depth_module.depth_profile:=640x480x30 \
  pointcloud.enable:=false align_depth.enable:=false
```

然后用同一ROS域、真实时钟及**真实机器人曝光时间TF**启动：

```bash
./run_gazebo_mapping.sh run_rviz:=true \
  config:=/home/czz/perception_eye_in_hand/src/realtime_nvblox/config/realsense_ros.yaml
```

不要把Gazebo TF用于真实相机图像。驱动版本会影响命名空间/话题名，以实际Image.header.frame_id和CameraInfo为准修改配置。保留原始深度光学系，若改为彩色对齐深度，必须同步换CameraInfo和frame，不能只换topic。相机内部TF由一个发布者负责，避免URDF和RealSense驱动重复发布。原SDK路线仍可用，但不可与ROS驱动同时独占同一个相机。

官方驱动参数参考：https://github.com/realsenseai/realsense-ros

## 8. 限制及改动文件

- 本次未构建/测试，不承诺10Hz或30Hz已达成；配置是目标/上限。
- Python导出不是原生增量ESDF层传输；活动块多时导出仍有代价。GPU计算、CPU读回、ROS发布应分别衡量。
- 不做动态自体过滤、动态障碍清除、闭环避障或精确复制Isaac ROS界面。
- 全局Gazebo reset在之前环境中出现关节状态/TF停发；日常清图用reset_map。若全局reset后失去TF，重新启动场景，不使用latest TF绕过。
- Image不含epoch ID，跨epoch数值完全重叠的延迟图像无法仅凭header无条件识别；已入队/在途旧epoch结果被淘汰。

修改：
- `realtime_nvblox/nvblox_backend.py`：新增稀疏观测支持ESDF导出，保留旧方法及真机调用。
- `realtime_nvblox/gazebo_ros_node.py`：稀疏输出、订阅者门控、诊断网格、仿真/真实ROS时钟。
- `config/gazebo_robot.yaml`、`config/realsense_ros.yaml`：默认稀疏/可选真实ROS相机配置。
- `launch/gazebo_mapping.launch.py`：mapper和RViz统一读取配置中的时钟模式。
- `rviz/gazebo_esdf.rviz`：稀疏话题说明和Best Effort订阅。
- `examples/verify_gazebo_mapping.py`、`verify_gazebo_rejection.py`：适配稀疏点数和坐标匹配，等待服务发现。
- `test/test_gazebo_depth.py`：更新测试替身的时钟属性，未执行。
- `ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro`：关闭额外原始点云。
- `ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/RealSensePlugin.cpp`：无订阅时避免Gazebo重复消息拷贝，过滤非有限深度。
- `ur7e_d435_gazebo/config/gazebo_clock.yaml`、`launch/scene.launch.py`：100Hz时钟及TF发布上限。
