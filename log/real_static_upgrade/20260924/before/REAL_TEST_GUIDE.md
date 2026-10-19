# 真实场景测试说明

## 1. 本版本边界

流程：**示教器/已确认路径带动预扫描 → 到点稳定采集 → 冻结静态 ESDF → 发布 xyz → SAGE 离线规划 → 预览 → 显式请求低速执行**。

- 本包不自动驱动机器人探索未知空间。预扫描移动由操作员在已确认无碰撞的路径上完成，软件自动选取稳定后的深度帧。
- 机器人底座和场景在地图使用期间必须固定。没有动态障碍检测、人进入监测或自动重建图；场景变化立即停止并废弃旧地图。
- 不启动 Gazebo，不发布仿真的 Float64 位置指令。唯一运动接口是配置的 `FollowJointTrajectory` Action。
- 默认 `execution_enabled: false`；发布目标只生成规划和预览，永远不会自动执行。
- 尚未做真实 UR7e、真实相机、实体接触、制动或重建精度验收。软件模拟器的误差不是硬件精度指标。

## 2. 环境与独立设备驱动

应用在 `whole_control` 运行；UR 官方驱动、RealSense 驱动和 RViz 使用系统 ROS Humble。先确认官方驱动版本与真实 UR7e/PolyScope 匹配。
本机目前已有 RealSense ROS 4.56.4；本次没有安装 UR 驱动，也没有连接机器人或相机。

### UR 驱动

按官方流程配置 External Control、提取这台机器的运动学校准参数，启动机器人驱动。以下为接口示例，必须先检查所安装版本的启动参数：

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py --show-args
# 将实际 IP 和已提取的工厂校准文件代入：
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:=192.168.1.10 \
  kinematics_params_file:=/home/wqj/ur7e_real_setup/robot_calibration.yaml
ros2 control list_controllers
ros2 action list -t
```

示例 IP 不是本包默认连接地址。本包本身不连接机器人 IP。如果该版本没有 `ur7e` 类型，先解决版本兼容，不要直接替换成其他型号。

本包默认 Action 为 `/scaled_joint_trajectory_controller/follow_joint_trajectory`。确认实际控制器名称和其插值设置：
**必须支持含位置、速度、加速度端点的 quintic spline（五次样条）**。本包在每个拐点提供零速度、零加速度，并对整段关节插值进行碰撞检查；各关节沿同一单调进度插值。
不支持改变空间曲线的额外平滑器，也没有测试 passthrough 控制器。不要在未验证插值语义时换控制器。
必须只有一个活动的关节运动控制器，不要同时运行仿真 SAGE/原规划包的执行节点。

官方说明：
- [UR 驱动启动与校准参数](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/startup.html)
- [UR 轨迹控制器、速度缩放和取消行为](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html)

### 真实相机

以本机安装的 RealSense ROS 4.56.4 为例，先用支持的深度格式，关闭不需要的彩色和红外输出。分辨率/帧率必须是设备实际支持的组合：

```bash
ros2 launch realsense2_camera rs_launch.py --show-args
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  enable_depth:=true enable_color:=false \
  enable_infra1:=false enable_infra2:=false \
  depth_module.depth_profile:=640x480x15
ros2 topic list -t
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo --once /camera/camera/depth/camera_info
```

实际话题和 `frame_id` 为准。默认模板使用 `/camera/camera/...`，不是仿真的 `/camera/...`。
本包支持 16UC1 毫米与 32FC1 米，使用与输入深度匹配的 CameraInfo。第一版使用原始校正深度，不需要对齐到 RGB。
RealSense 的 `enable_sync` 只处理相机内部数据流，不会自动同步机器人反馈；必须验证图像时间与 TF 时间对应关系。
[RealSense ROS 官方说明](https://github.com/realsenseai/realsense-ros)。

## 3. 必须准备的真实模型与配置

### 3.1 同一份校准 URDF

准备展开后的 URDF XML，而不是 Xacro 源文件。必须包含：

- `base_link` 下的六个标准 UR 关节，使用这台机器的运动学校准参数；暂不支持关节名前缀或附加活动夹爪关节。
- `tool0` 和实际安装相机的固定 TF 链，直到 `camera_depth_optical_frame`（名称可配置）。
- 实际关节限位。

可从完整的运行时 `/robot_description` 导出，避免 `ros2 topic echo` 的 YAML 转义破坏 XML：

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/setup.bash
/home/wqj/miniconda3/envs/whole_control/bin/python -B \
  -m ur7e_real_static.export_description \
  --output /home/wqj/ur7e_real_setup/calibrated_with_camera.urdf
```

如果该话题只有机械臂，没有相机固定链，先完善统一描述。导出工具不会凭空生成或标定相机外参。
SAGE 与 CPU 检查都使用这份模型；每次建立 SAGE 规划任务时执行 CPU/GPU FK 一致性检查。采集/执行时再与实际 TF 比较：位置误差超过 3 mm 或姿态误差超过 0.01 rad 会拒绝使用。
这些是模型一致性阈值，不是实机绝对精度保证。

### 3.2 手眼标定与 TF

用实际安装完成手眼标定，关系为：

`base_link ← tool0(q_actual, image_time) ← depth_optical_frame`

如果标定给的是 `tool0 → color_optical_frame`，需要与相机内部 color/depth 外参组成到 depth optical 的链。
不要把光学坐标系的标定直接贴到相机壳体 `camera_link`；不要加载仿真侧装外参。
固定 TF 每个 child 只能有一个确定的父节点和有效发布源，避免 URDF、static_transform_publisher、相机驱动重复发布冲突变换。
若统一 robot_state_publisher 发布全部相机 TF，可关闭相机驱动 `publish_tf`；若使用相机驱动内部 TF，则在规划 URDF 中记录同样的完整固定链。

验证：

```bash
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_ros tf2_echo tool0 camera_depth_optical_frame
```

换多个实际姿态观察同一静止墙面，应基本重合，不能出现随姿态移动的重影。UR 控制器 `base` 与 ROS `base_link` 不一定等同；默认目标推荐直接使用 `base_link`。

### 3.3 碰撞模型和配置副本

```bash
mkdir -p /home/wqj/ur7e_real_setup
cp /home/wqj/realtime_nvblox_ws/src/ur7e_real_static/config/real.yaml \
  /home/wqj/ur7e_real_setup/real.yaml
cp /home/wqj/realtime_nvblox_ws/src/ur7e_real_static/config/collision_spheres.template.yaml \
  /home/wqj/ur7e_real_setup/collision_spheres.yaml
```

模板中的机械臂球仅作为待核对起点，不是已验收的实机模型。`tool0` 的球列表故意为空，必须测量并填入相机、支架、工具的包裹球；为空会拒绝启动。
球心坐标属于对应 link，不能把世界坐标填进工具球列表。实际负载和 TCP 也须在机器人控制器中正确配置。

编辑 `real.yaml`：

```yaml
robot_urdf: /home/wqj/ur7e_real_setup/calibrated_with_camera.urdf
collision_spheres: /home/wqj/ur7e_real_setup/collision_spheres.yaml
output_directory: /home/wqj/nvblox_output/my_real_scene
execution_enabled: false
commissioned: false
use_sim_time: false
max_joint_speed: 0.05
max_joint_acceleration: 0.10
```

同步修改真实话题、相机 frame、Action 名和地图范围。速度/加速度单位为 rad/s、rad/s²，默认是起步参数而不是已经在真机验证的安全速度。

`commissioned_free_boxes` 默认空。它仅允许录入独立测量、确认无障碍的固定空间，作为显式来源为 2 的初始化先验，需要 `commissioned: true`。
不要为了绕过“未知空间”错误，把整个工作区写成自由空间。默认没有仿真的 12 cm 起始先验、7.5 cm 扫描走廊，也不会沿机械臂运动自动清空空间。

## 4. 第一次测试：只扫描、只预览

### 4.1 构建与启动

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_real_static.sh
./run_ur7e_real_static.sh \
  config:=/home/wqj/ur7e_real_setup/real.yaml run_rviz:=true
```

驱动先独立启动，本命令只启动应用和可选 RViz。
另一个系统 ROS 终端：

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/setup.bash
ros2 topic echo /ur7e_real_static/status
ros2 service call /ur7e_real_static/start_scan std_srvs/srv/Trigger '{}'
```

看到 `SCANNING` 后，通过示教器或独立确认过的路径改变观察位置，每个位置停稳约 1～2 秒，确认 `frames` 增加。
默认关节速度低于 0.015 rad/s、持续稳定 0.5 s 后才使用深度帧；至少 15 帧、3 个有区别的观察姿态才能冻结。
视角数量只表示基本采集量，不代表任意目标通道都已覆盖。相机转向空白/无效区域时不会伪造新地图。

RViz：

- `Measured surface ESDF`：原生观测距离场近表面，颜色代表距离，不是物体 RGB。
- `Planning free-space clearance`：正距离自由空间，可按需开启；不把未知体素画成墙面。
- 碰撞球：检查真实相机/支架/工具是否被覆盖。
- 路径：目标规划成功后显示经检查的 TCP 轨迹；整臂碰撞另由模型检查，不能只看末端线。

### 4.2 冻结

机械臂停稳、地图仍有新帧时：

```bash
ros2 service call /ur7e_real_static/freeze_map std_srvs/srv/Trigger '{}'
```

成功后为 `MAP_READY`，融合停止，保存 `frozen_map.npz`。必须保留原始时间戳，静态地图不因为超过 1.5 s 就自动过期。
`MAP_READY` 只代表已经冻结；目标通道是否可用在规划时检查。
如果报告图像不新鲜，等下一帧融合完成再调用，不要先关闭相机。

### 4.3 设置近距离目标并预览

下面坐标只是消息格式示例，必须换成当前实际 tool0 附近、已观测自由区中的目标；第一例建议仅位移几厘米。

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link}, pose: {position: {x: 0.5, y: 0.0, z: 0.45}, orientation: {w: 1.0}}}'
```

支持有 TF 的其他 frame（如 world），空 frame 拒绝。目标目前是 **tool0 的 xyz**，不跟踪 orientation，不是示教器上任意自定义 TCP。
等待 `PLAN_READY`，检查 RViz 路径和碰撞球。消息 `/ur7e_real_static/preview_trajectory` 包含完整关节轨迹。
SAGE 在固定地图上离线运行，规划时间不进入机器人执行周期。规划失败时不发送任何指令。

典型拒绝：

| 信息 | 含义与处理 |
|---|---|
| `Start body is unknown...` | 起始整臂周围缺少自由空间证据；补观测，或独立确认并记录有限固定自由区 |
| `No observed collision-free goal IK` | 终点未观测/有障碍/不可达，检查目标和地图 |
| `No checked route...` | 有界搜索未找到通道，补观测或换目标；不等于数学证明不可达 |
| `TF_MODEL_MISMATCH` | 实际 TF 与文件不一致，检查工厂校准、光学 frame 和手眼链 |
| `CAPTURE_ERROR` | 内参/帧变化或融合异常，修复后重新 start_scan，旧地图不得继续使用 |
| `Robot moved since planning` | 机器人当前位置不再等于 A，保持静止后重新发布目标 |

## 5. 第二阶段：低速实机执行

完成模型、外参、碰撞包络、控制器插值和静态工作区确认后，停止应用，把配置改为：

```yaml
commissioned: true
execution_enabled: true
```

重新启动不会自动加载地图或运动。模型和建图参数没变且环境没变时可显式加载：

```bash
ros2 service call /ur7e_real_static/load_map std_srvs/srv/Trigger '{}'
```

地图有模型/外参/建图参数指纹和内容校验，文件不匹配会拒绝；但软件无法仅凭文件检测物体被人搬动，必须确认现场。
加载后只有规划距离场可恢复显示；原生近表面显示来自扫描过程，本版本没有保存可继续融合的 nvBlox 原生层。

重新发布目标并等待 `PLAN_READY`，确认现场仍静态，然后执行：

```bash
ros2 service call /ur7e_real_static/confirm_scene std_srvs/srv/SetBool '{data: true}'
ros2 service call /ur7e_real_static/execute std_srvs/srv/Trigger '{}'
```

只有显式 execute 才发送 Action；成功返回表示已发出接受请求，不代表已到达。观察 `SENDING → EXECUTING → GOAL_REACHED`。
软件同时要求：关节反馈新鲜且停稳、实际起点与规划起点一致、TF/模型一致、相机内参一致、地图版本一致、完整轨迹再次通过净空与速度/加速度检查。
控制器报告成功后还检查实际 tool0 位置误差不超过 2 cm。

执行期间发布新目标会先取消当前动作，**不会自动接着走新目标**；等 `STOPPED` 后重新确认场景、发布目标、检查并 execute。
当前版本不支持滚动替换轨迹。

### 停止与失效

```bash
# 取消本包发出的轨迹；等待取消结果和稳定关节反馈：
ros2 service call /ur7e_real_static/stop std_srvs/srv/Trigger '{}'
# 场景/基座/相机安装改变，停止并废弃当前地图：
ros2 service call /ur7e_real_static/invalidate std_srvs/srv/Trigger '{}'
```

`stop` 不会控制示教器或其他程序发出的独立运动。`STOP_UNCONFIRMED` 表示取消或停稳没有确认，须使用机器人自身控制处理，不能把服务响应当作实体已经停止。若 Action 接受/结束响应丢失，软件会锁定此状态；先在机器人侧确认已取消和停稳，再重启应用，不能仅凭短暂静止认定旧请求不会执行。
应用对关节断流、TF 不一致、Action 反馈超时、跟踪误差和实际状态进入不允许空间请求取消。
正常 Ctrl-C/SIGTERM 会在 ROS 上下文还有效时请求取消并短暂等待；断电、SIGKILL、整机卡死无法保证送达取消请求。
本包发送完整轨迹，某些控制器在客户端消失后会继续已接收的轨迹，因此真实测试必须保留机器人独立停止手段，不能依赖 Python 看门狗替代它。

## 6. 记录与验收

输出目录：

- `resolved_config.yaml`：本次实际配置。
- `events.jsonl` / `status.json`：状态变化和当前状态。
- `frozen_map.npz`：规划 ESDF、自由来源、标定/模型指纹、采集时间、视角及校验。
- `trajectory.npz`：规划位置、时间、目标、地图 ID、SAGE 次数；重启后不会自动执行该文件。
- `planning/`：统一规划 URDF、SAGE 参数及空几何真值配置。

建议同时录制 joint_states、TF/TF_static、depth、CameraInfo、状态和控制器反馈，保留采集时刻。
先验证近距离 A→B，再增加经过已观测通道的绕障目标，记录实际位置误差、最小模型净空、执行状态和取消行为。
本次软件模拟结果见 [validation/RESULTS.md](validation/RESULTS.md)，不能据此跳过实机验收。
