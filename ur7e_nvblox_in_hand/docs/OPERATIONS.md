# 从编译到仿真测试的完整操作

当前默认已调整为 **5mm体素、实时Gazebo实验台、RViz**，不需要回放。最新简明启动和实时监测步骤见 [5mm默认实时仿真](LIVE_GAZEBO_5MM_DEFAULTS_20261111.md)。下文保留其他模式和历史显式参数示例。

实机UR7e＋D435i请优先按[实机专项操作文档](REAL_UR7E_D435I_TEST_GUIDE_20261109.md)分开启动驱动和相机；其中使用历史真实话题与安装TF，并说明当前内置相机入口的参数兼容限制。

以下以本项目实际路径为例。所有操作针对新的ur7e_nvblox_in_hand，不修改相邻nvblox_in_hand或workspaces。测试域135、Gazebo端口11565，与之前工程隔离。

## 1. 新机器部署与本机编译

已验证基础：Ubuntu22.04、ROS2 Humble、Python3.10、NVIDIA驱动、CUDA Toolkit、兼容PyTorch和nvblox_torch。驱动、Toolkit、PyTorch构建CUDA版本是不同概念；当前验证组合见THIRD_PARTY.md。必须在目标机器编译桥接/相机插件；换GPU按实际计算能力设置UR7E_CUDA_ARCH，例如本机4060为89。

新机器先安装ROS Humble、兼容驱动和CUDA Toolkit。再按需执行显式依赖安装：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./install_dependencies.sh --system
```

已有系统依赖、只缺Python依赖时用`./install_dependencies.sh`。脚本将包安装到当前用户Python环境，可能改变已有Python依赖；不会自动在build/run时执行。wheel应与vendor记录的提交一致，check_wheel.py会拒绝不匹配的版本；不要绕过检查。没有下载识别模型、cuVSLAM或部署整套workspaces。

本机依赖齐全，直接编译：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
```

编译包含本地Torch桥接和三个ROS包。**只加载/opt/ros/humble与本工程自己的install，不source原workspaces或兄弟工程。** build.sh设置的架构默认89，其他GPU需在构建前设置`export UR7E_CUDA_ARCH=实际架构`。

编译后检查接口与GPU基本功能：

```bash
source ./env.sh
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 tools/check_backend.py
./run.sh --show-args
```

检查后端使用合成平面，不启动相机/机械臂；它验证原生体素中心/标记、符号距离、查询、Mesh、保存加载清图和2D切片，不等于实机精度验收。

## 2. 推荐仿真顺序：先场景与高位姿态，再开始建图

### 终端1：纯场景

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
./scene.sh gazebo_gui:=true
```

正式频率测试可将GUI改为false；本轮测试GUI关闭、RViz开启，打开额外窗口会改变负载。端口占用时先确认旧实例，不要结束无关任务的Gazebo。

### 终端2：高位观察姿态

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
source ./env.sh
/usr/bin/python3 tools/view_sweep.py --check-only
/usr/bin/python3 tools/view_sweep.py --seconds 0
```

路径从已检查的初始姿态移动到约0.82m高位中心。脚本检查Gazebo、关节、robot_description和控制器，不匹配会拒绝；只用于本项目仿真，不能直接发送给实机。

### 终端3：开始感知并查看3D ESDF

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
./run.sh start_scene:=false rviz_mode:=diagnostic \
  voxel_size:=0.03 depth_hz:=40 color_hz:=40 \
  esdf_hz:=33.4 esdf_view_hz:=33.4 mesh_hz:=5 layer_hz:=5
```

保持静止约15秒，再在终端2执行小幅左右扫描：

```bash
/usr/bin/python3 tools/view_sweep.py --seconds 20 --amplitude 0.08
```

命令沿world Y方向移动，不环绕、不前后往复，结束后回中。命令幅度上限±8cm，但20秒不是完整正弦周期，实际跨度以记录为准。

若只需一键场景+感知，不移动机械臂：

```bash
./run.sh gazebo_gui:=true rviz_mode:=diagnostic
```

不要同时执行这条与终端1的场景启动。先高位后Mapper的分步方式更适合质量对照，避免把接近过程写进静态地图。

## 3. 观察视图与话题

只启动一个Mapper。可在同一RViz勾选Mesh、3D ESDF；也可在额外终端加载环境后运行下列一种：

```bash
./rviz.sh mesh
./rviz.sh diagnostic
```

两条分别执行，不是要求同时打开。额外窗口会增加GPU/CPU负载。Fixed Frame=base_link，Views=Orbit，在3D区域左键旋转、中键平移、滚轮缩放。Mesh为真实RGB顶点颜色；ESDF彩虹表示距离，默认−0.09～0.24m，点大小3像素。

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| /camera/depth/image_rect_raw | sensor_msgs/Image | 原始深度16UC1或32FC1 |
| /camera/color/image_raw | sensor_msgs/Image | 原始RGB，程序也支持bgr8 |
| /camera/*/camera_info | sensor_msgs/CameraInfo | 各自内参和光学frame |
| /ur7e_nvblox/esdf_points | sensor_msgs/PointCloud2 | 3D原生observed体素中心，距离和标记 |
| /ur7e_nvblox/static_esdf_pointcloud | sensor_msgs/PointCloud2 | 仅2D模式的原生切片 |
| /ur7e_nvblox/mesh | visualization_msgs/MarkerArray | 当前完整彩色Mesh，标准RViz插件显示 |
| /ur7e_nvblox/tsdf_layer | sensor_msgs/PointCloud2 | 按最小权重筛选的原生TSDF体素 |
| /ur7e_nvblox/color_layer | sensor_msgs/PointCloud2 | 原生颜色体素与权重 |
| /ur7e_nvblox/camera_pose | geometry_msgs/PoseStamped | 成功融合深度对应的机械臂相机pose |
| /ur7e_nvblox/camera_path | nav_msgs/Path | 机械臂TF相机轨迹，非SLAM，限长/约5Hz |
| /ur7e_nvblox/metrics | std_msgs/String | 累计计数、墙钟频率、CPU提交/读回/打包计时、版本与库路径 |

3D点云每点24字节：x/y/z/intensity(float32)、rgb(uint32)、observed/is_inside/is_site(uint8)。正负距离与inside一致。它不是Mesh表面点；已知自由空间也可有ESDF体素，unknown不会被填成安全距离。3D模式不应等待二维话题。

Mesh、ESDF显示和地图层按订阅需求导出。没有相应订阅时，不额外生成/读回这些显示数据；ESDF内部更新与显示导出是两件事。

## 4. 监测实际频率

每个终端均执行环境块：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
source ./env.sh
ros2 param get /ur7e_nvblox voxel_size
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

Ctrl+C停止最后一条，再分别检查：

```bash
ros2 topic hz /ur7e_nvblox/mesh --wall-time
# Ctrl+C后
ros2 topic hz /camera/depth/image_rect_raw --wall-time
# Ctrl+C后
ros2 topic hz /camera/color/image_raw --wall-time
```

融合频率看metrics累计计数或wall_hz，不能拿输入hz/渲染FPS冒充。esdf_submit_ms是CPU调用计时；导出包含GPU等待，二者不是独立CUDA kernel计时。详细的发布Hz、新输入关联Hz、年龄需由自动脚本直接记录header与到达时刻。

## 5. 开关、体素和模式

可在启动时设置以下开关：enable_depth、enable_color、enable_esdf、enable_esdf_view、enable_mesh、enable_layers、publish_pose、publish_path、publish_esdf_distance_slice；完整参数用`./run.sh --show-args`查看。

运行中可切换主要开关和频率，例如：

```bash
ros2 param set /ur7e_nvblox enable_color false
ros2 param set /ur7e_nvblox enable_color true
ros2 param set /ur7e_nvblox esdf_view_mode band
ros2 param set /ur7e_nvblox esdf_view_mode observed
```

关闭发布不等于删除RViz已经缓存的画面。原生ESDF选择支持observed（内外侧已观测）、band（距离带）、inside（内部）；inside不是完整距离场，也不是重建表面。

体素和模式必须停止旧Mapper，重新启动新地图：

```bash
./run.sh start_scene:=false voxel_size:=0.02 rviz_mode:=mesh
# 或停止后选择
./run.sh start_scene:=false voxel_size:=0.05 rviz_mode:=diagnostic
# 或停止后选择二维
./run.sh start_scene:=false esdf_mode:=2d rviz_mode:=2d
```

启动参数profile=realtime为当前测试档；reference使用2cm和较低颜色/ESDF频率，仅为参考数值。自定义ROS YAML通过mapper_config:=绝对路径传入；CLI覆盖最后生效。每个参数用途见config/base.yaml及源码。decay_hz和clearing_hz默认0；若开启清图，必须同时设置正clearing_radius_m。

## 6. 保存、加载、清空与查询

以下在已加载环境的监测终端执行，目录可自行修改：

```bash
mkdir -p /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual/map.blox'}"
ros2 service call /ur7e_nvblox/save_timings ur7e_nvblox_interfaces/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual/timings.json'}"
ros2 service call /ur7e_nvblox/load_map ur7e_nvblox_interfaces/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual/map.blox'}"
ros2 service call /ur7e_nvblox/clear_map std_srvs/srv/Trigger '{}'
```

load_map先在临时Mapper中加载并检查体素大小，不匹配保留原地图。保存/加载/清图等服务由GPU所有者执行，可能暂停融合，应放在性能采样窗口之外；超时后正在执行的GPU/文件操作可能继续完成，不保证撤销。

距离与梯度查询：positions为global_frame中的XYZ三元组序列，返回distances、gradients和observed。unknown不能按自由空间使用。

```bash
ros2 service call /ur7e_nvblox/get_esdf_and_gradient ur7e_nvblox_interfaces/srv/QueryEsdf "{positions: [0.4, 0.0, 0.3], update_esdf: false}"
```

也可指定use_aabb=true、aabb_min和aabb_size，以体素中心采样；这是**按需查询服务**，不是地图构建/默认显示路径。限制max_query_points防止大网格拖垮处理。这里的类型与原nvblox_msgs不同，旧客户端需适配。

## 7. 自动验收与截图

先退出域135/端口11565上的手动场景与Mapper，再执行：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
source ./env.sh
DISPLAY=:1 /usr/bin/python3 tools/sim_test.py
```

脚本创建新的artifacts/simulation_时间目录，执行场景检查、高位姿态、静止/左右扫描、原生点/查询检查、文件服务、颜色/选择开关、清图/加载、2D切片和层/Mesh/切片开关测试；只结束自己启动的进程。截图仅捕获本工程RViz窗口；已有多个本工程RViz时会拒绝，避免误截其他窗口。DISPLAY按实际桌面设置。结果状态必须为completed；失败目录保留但不当成成功样本。

## 8. 实机与rosbag输入入口

本轮只验收Gazebo与合成GPU检查，下面入口保留但未据此宣称实机性能已验收：

```bash
./run.sh start_scene:=false launch_camera:=true use_sim_time:=false
./run.sh start_scene:=false use_sim_time:=true rosbag:=/绝对路径/rosbag目录
```

实机需要额外安装realsense2_camera及SDK，并由真实机械臂发布准确关节/手眼TF；本包不控制实机机械臂。bag默认使用已录制/clock；没有/clock时可传rosbag_args:="--clock"，不要创建两个时钟源。
