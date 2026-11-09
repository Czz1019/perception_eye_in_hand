# 重构版完整操作说明

2026-11-05后续：查看三维点云使用 `./run_esdf.sh`，显式开启原生体素诊断导出；三维色带调整为−0.09～0.24m，二维保留0～1m。[发布条件、真实截图和新实测](ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)。

2026-11-05最新：realtime两路融合门限40Hz、相机30Hz；ESDF与诊断显示门限33.4Hz。官方ESDF调用32.78Hz，诊断点云29.44Hz；关联深度时间戳推进约19.75Hz，非30Hz全新地图。详见 [门限、配色与完整实测](ESDF_334HZ_AND_COLOR_TEST_20261105.md)。

最新输入已按用户要求改为640×480，两路CameraInfo已实测核对；本轮只测3D，结果见 [640×480三维ESDF复测](INPUT640_3D_ESDF_TEST_20261104.md)。

后续复测已完成，实际频率、截图和仍存质量问题见 [重构版仿真复测](REFACTORED_EXAMPLE_TEST_20261104.md)。下面是运行指令，不能把配置目标当作实测。

适用2026-11-04基于workspaces/nvblox_examples重构后的代码。以下为操作指令；重构后的后续仿真验证记录见顶部最新报告。代码原理、来源和验证边界见 [重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md)。

## 1. 环境与构建

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
```

默认构建本工程两个包，依赖 `/home/czz/workspaces/isaac_ros-dev/install` 中官方ROS/CUDA/NITROS库。不会修改父工程或workspaces。需要从本地upstream快照全量重编时可使用 `./build.sh --from-source`，启动时指定 `IN_HAND_SOURCE_BUILD=1`；该路径本轮未重新验收。不要删除其他工作区build/install。

如需之后自行运行离线检查，可执行 `./test.sh`。本轮已通过20项Python与既有C++策略检查；GPU导出单测未单独重跑，运行时导出已验证。

## 2. 默认仿真：官方三维感知主体

先在旧启动终端Ctrl+C停止同域同端口的旧场景和mapper。只关闭Gazebo/RViz窗口不一定停止后台进程。检查端口可用 `ss -ltnp 'sport = :11457'`，不要盲目批量杀进程。

终端A：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run.sh gazebo_gui:=true
```

默认：本项目Gazebo深度和彩色均640×480，30Hz目标；原UR7e+D435场景、官方NvbloxNode组件、static_tsdf、3d ESDF、RGB融合、Mesh、地图层接口及机械臂相机路径。realtime档体素3cm、传感器目标30Hz，深度/颜色融合限频40Hz，ESDF门限33.4Hz、Mesh更新/地图层发布目标5Hz，实际频率见最新报告。RViz默认看彩色Mesh，不自动移动机械臂，不执行环绕。

若旧场景仍在使用1280×720，必须先停场景并重新启动才能切到640×480；start_scene=false不会更改外部相机分辨率。可用camera_width:=1280 camera_height:=720恢复上一轮尺寸。

如果同环境的本项目场景已经运行，只启动感知及显示：

```bash
./run.sh start_scene:=false
```

不要同时运行两条启动命令。默认后端不发布项目自定义esdf_points或metrics，不能据此判断官方ESDF未工作。

## 3. 官方二维ESDF显示

停止上一轮mapper，再运行：

```bash
./run.sh esdf_mode:=2d
```

入口自动选择official_2d.rviz，显示 `/nvblox_node/static_esdf_pointcloud`。切片高度默认0.25m，输入投影高度范围0到0.8m，来自机械臂场景配置。需调整时显式指定：

```bash
./run.sh esdf_mode:=2d slice_height:=0.25 slice_min_height:=0.0 slice_max_height:=0.8
```

这是原生二维模式，不承诺同时保留一套独立完整三维ESDF。`publish_esdf_distance_slice=true`也不会让本地官方3d模式自动产生二维点云。

## 4. 查看三维ESDF诊断点

停止上一轮mapper后，显式选择诊断后端：

```bash
./run.sh mapper_backend:=diagnostic esdf_mode:=3d esdf_view_mode:=observed
```

此时使用项目派生节点调用官方融合/更新，并额外导出原生ESDF体素，自动打开诊断RViz。默认显示门限33.4Hz，三维采用固定−0.09～0.24m距离彩虹色带，保留3像素点显示；outside与inside均输出，字段包含距离及observed/is_inside/is_site。unknown不输出，但不能由“没有点”断言该位置自由。

有限近障碍带或更高诊断目标频率可单独设置：

```bash
./run.sh mapper_backend:=diagnostic esdf_view_mode:=band esdf_view_band_m:=0.12 esdf_view_hz:=30
```

band会截掉更远有效距离；inside只显示内部侧；surface仅是inside的旧别名。提高诊断频率会增加全块扫描/同步负载，不能把它混进无附加输出的官方性能基线。不要为查看诊断另开第二个同名mapper。

单独开显示，终端先加载相同环境：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./rviz.sh mesh
# 或 ./rviz.sh 2d
# 或 ./rviz.sh diagnostic
```

显示模式必须与运行后端匹配。默认 `./rviz.sh` 是Mesh。实机显示使用 `IN_HAND_USE_SIM_TIME=false ./rviz.sh mesh`。旧artifact中的RViz配置不是新入口配置。

## 5. 检查输入、组件和官方统计

终端B：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
source ./env.sh
ros2 component list
ros2 node info /nvblox_node
ros2 topic list -t
ros2 service list -t
./check_sim.sh
```

默认容器名nvblox_container，组件名nvblox_node。CameraInfo、图像、TF和时钟都需正常；check_sim只读检查，不发关节命令。

逐项运行，采样后Ctrl+C：

```bash
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/color/image_raw --wall-time
ros2 topic hz /nvblox_node/mesh --wall-time
# 仅2d模式：
ros2 topic hz /nvblox_node/static_esdf_pointcloud --wall-time
# 仅diagnostic后端：
ros2 topic hz /nvblox_node/esdf_points --wall-time
ros2 topic echo /nvblox_node/metrics --once
```

官方3d ESDF更新看控制台 `ros/update_esdf` 和save_rates，不拿Mesh Hz替代。该指标是更新任务调用次数，不保证每次有新深度或新地图变化；GPU完成时间也需独立测量。topic hz和RViz订阅会产生真实发布需求，应记录比较时负载。

## 6. 运行档、自定义参数和功能开关

每条指令是一轮独立启动：

```bash
# 继承workspaces base的2cm、颜色/ESDF 5Hz及衰减/清图数值
./run.sh profile:=reference
# 明确使用当前3cm、核心33.4Hz门限档
./run.sh profile:=realtime
# 保留相同工作范围，按需指定不同分辨率
./run.sh voxel_size:=0.02 max_range_m:=3.0
# 自定义配置，未指定CLI数字时继承YAML
./run.sh mapper_config:=/绝对路径/自定义.yaml
# 仅几何，不融合颜色
./run.sh enable_color:=false
# 不请求普通体素层，仍保留Mesh
./run.sh enable_layers:=false
# 关闭Mesh计算周期和正常Mesh发布需求
./run.sh enable_mesh:=false
# 不运行显示，保留官方感知
./run.sh run_rviz:=false
# 显式选择历史清理策略；会影响地图保存与覆盖
./run.sh decay_hz:=2 clearing_hz:=1 clearing_radius_m:=2
```

reference仍采用本项目机器人、输入、范围和颜色开关，不是workspaces整个应用的完全复制。本地update_mesh_rate_hz未用于独立调度，Mesh/体素层共用publish_layer_rate_hz任务；mesh_hz是公共发布频率的兼容别名，与layer_hz同时指定须一致；关闭enable_layers不等于关闭整个公共发布任务。disabled命名空间不要另行订阅。

如果已经运行一个外部容器，可用attach模式只加载官方组件：

```bash
# 终端A，加载env后：
ros2 run rclcpp_components component_container_mt --ros-args -r __node:=nvblox_container
# 终端B，现有本项目场景及相同env：
./run.sh start_scene:=false attach_to_container:=true
```

需要容器中尚无同名mapper。诊断后端不支持attach。独立感知入口为 `ros2 launch nvblox_in_hand nvblox.launch.py`；它本身不启动场景、机械臂或路径。

## 7. 原生地图、Mesh和统计保存

在已加载env的终端运行：

```bash
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/scene_map.nvblx'}"
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/scene_mesh'}"
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/rates.txt'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/timings.txt'}"
```

以success及实际输出文件为准。load_map使用FilePath和有效保存路径。原生get_esdf_and_gradient服务保留，但应只在所需AABB上按需调用，避免每帧请求全图。本地实现没有明确xyz梯度数组，规划接入前需核实距离、unknown和梯度契约。

## 8. 真实D435与机械臂pose

```bash
./run.sh start_scene:=false use_sim_time:=false launch_camera:=true
```

仅启动感知及可选相机，不启动真实机械臂运动控制。真实机器人驱动须预先提供完整关节/URDF/手眼/光学TF。D435驱动publish_tf=false，避免重复TF；此配置依赖外部完整相机链。按实际驱动输出名称设置depth_topic、depth_info_topic、color_topic、color_info_topic，四者应分别正确，不能拿深度CameraInfo给彩色图像。

## 9. 可重复输入回放

记录完整输入：

```bash
source ./env.sh
ros2 bag record -o artifacts/d435_robot_input /camera/depth/image_rect_raw /camera/depth/camera_info /camera/color/image_raw /camera/color/camera_info /joint_states /tf /tf_static /clock /robot_description
```

回放前停止实时场景、相机及重复TF源。bag应包含必需的TF和标定，RobotModel显示还需要robot_description。包含已录制/clock时：

```bash
./run.sh start_scene:=false use_sim_time:=true rosbag:=/绝对路径/d435_robot_input
```

bag没有/clock时，才用 `rosbag_args:="--clock"` 生成回放时钟。不能让两个来源同时发布时钟。回放附加参数通过shlex拆分为参数数组，不经shell执行。公平比较还需相同体素、范围、颜色开关、清理策略和订阅负载；本轮未做回放验收。

## 10. 旧扫描脚本兼容边界

不自动环绕。旧orbit_scan工具仍依赖诊断后端metrics，不能直接把默认官方后端的“没有metrics”当感知故障。需要使用原扫描工具时，先显式用mapper_backend=diagnostic启动，并阅读原工具碰撞检查和固定场景限制；本轮没有执行或验收扫描，也没有改写它的运动逻辑。

运行后的窗口效果、质量、更新率及延迟以对应输入尺寸的实测报告为准，不以配置目标替代。
