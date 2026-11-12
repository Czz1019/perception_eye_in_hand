# 默认5mm实时Gazebo仿真：直接启动、RViz与实时频率

日期：2026-11-11。工作区：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`。这是**现场Gazebo相机数据直接建图**，不是rosbag回放，也不连接实机。

## 1. 已完成的修改

| 文件 | 修改 | 目的 |
|---|---|---|
| [base.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/config/base.yaml) | voxel_size=0.005；工作区改bounding_box | 默认5mm，限制地图范围 |
| [realtime.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/config/realtime.yaml) | voxel_size=0.005、clearing_hz=0、decay_hz=0 | 保持5mm并显式关闭清图，覆盖base中正频率/负半径冲突 |
| [node.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py) | 节点后备默认值与上述参数一致 | 避免直接运行节点时体素、边界、深度范围、队列又变成旧设置 |
| [bringup.launch.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/launch/bringup.launch.py) | 默认scene_variant=lab | 默认一起启动实验台场景、相机、Mapper与RViz |
| [scene.launch.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/launch/scene.launch.py) | 默认scene_variant=lab | 独立scene.sh入口也使用实验台 |
| [watch_realtime.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/tools/watch_realtime.py) | 新增现有metrics的简洁终端查看器 | 持续显示输入、融合、ESDF更新/发布、Mesh发布及点数 |

未改变TSDF/Color/ESDF/Mesh算法、相机内参外参、实验台物体或机械臂运动逻辑。`reference`配置仍作为可显式选择的其他模式；默认启动使用`realtime`。

## 2. 现在默认是什么

- 启动实时Gazebo实验台：桌子、试管架、瓶子、托盘等。
- 机械臂直接实例化在原有高位观察姿态，不自动环绕或扫描。
- 相机RGB与Depth均640×480，传感器配置30Hz。
- voxel_size=0.005m；truncation_vox=4，即2cm截断距离。
- 完整3D ESDF仍5mm；显示stride=2，即1cm格点间距。
- 深度/颜色融合准入上限40Hz；ESDF更新/显示上限33.4Hz；Mesh/地图层上限5Hz。
- 工作区x=[−0.10,1.10]、y=[−0.60,0.60]、z=[−0.05,1.05]m，按块选择。
- 保留修改前基础配置中的深度范围0.05–2.05m和输入队列4；与此前归档回放配置0.1–3m不同，不混淆。
- 衰减和半径清图关闭，保留累积地图。
- 仿真输入自动选择reliable；真实相机的sensor_data默认策略仍保留。
- 不需要`mapper_config`，也不加载artifacts中的旧测试YAML。

当前base中的clearing_hz和decay_hz均为1.0、clearing_radius_m为−1.0，默认realtime配置显式覆盖clearing_hz=0、decay_hz=0，避免无效清图组合阻止启动，并保持本次静态重建观测。只加载base而不加载realtime时仍需正确设置这一对参数。

默认配置加载顺序：节点后备值→安装目录base.yaml→realtime.yaml→可选mapper_config→命令行显式覆盖。本指南不传mapper_config，直接使用修改后的项目默认值。

## 3. 终端1：编译并启动现场仿真

先退出之前的回放Mapper/旧RViz，以免误看旧窗口。下面使用独立域164、Gazebo端口11665，避免与前面的域163回放混用。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh

export ROS_DOMAIN_ID=164 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11665
./run.sh
```

`./run.sh`现在默认就是：实验台、现场相机、5mm Mapper和RViz。不需要另启相机、不需要播放记录，也不需要在命令行重复全部参数。

等待机器人/实验台生成和Mapper ready，通常还需几秒让TF、图像和地图开始更新。Gazebo必须处于播放状态；暂停会停止仿真时钟与相机更新。退出时在该终端Ctrl+C，正常关闭这一组启动的进程。

若端口占用，先确认此前自己的启动是否已退出：

```bash
ss -ltnp 'sport = :11665'
```

不要直接结束其他任务的Gazebo。

## 4. 终端2：持续查看实时频率

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=164 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11665

python3 tools/watch_realtime.py
```

或者只观察30秒：

```bash
python3 tools/watch_realtime.py --duration 30
```

每秒一行：

```text
wall_s | depth_in color_in | depth_fuse color_fuse | esdf_update esdf_pub mesh_pub | exported_points
```

| 列 | 含义 |
|---|---|
| depth_in / color_in | Mapper实际接收到的深度/颜色频率 |
| depth_fuse / color_fuse | 融合host调用返回后的频率 |
| esdf_update | 内部ESDF更新host调用频率 |
| esdf_pub | ESDF点云publish调用频率 |
| mesh_pub | Mesh发布调用频率 |
| exported_points | 当前显示导出点数，受stride影响，不是全图原生体素数 |

工具只订阅已有String metrics，不订阅大地图，不控制机械臂，也不新增算法计时字段。它不是GPU完成频率或RViz FPS。若超过5秒没有metrics，会提示检查域、Mapper和/clock；没有仿真时钟时metrics定时器也会停。

若Mesh发布为0，先检查RViz中是否勾选Mesh：当前节点按订阅需要生成Mesh，而不是无条件回读大网格。

## 5. 终端3：确认参数、输入与实际接收

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
export ROS_DOMAIN_ID=164 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11665

ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox truncation_vox
ros2 param get /ur7e_nvblox workspace_bounds_type

# 查看完整生效参数
ros2 param dump /ur7e_nvblox
```

应分别看到0.005、4.0、bounding_box。

下面每条分别运行，Ctrl+C后再换下一条：

```bash
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/color/image_raw --wall-time
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
ros2 topic hz /ur7e_nvblox/mesh --wall-time
```

`topic hz`显示该终端的实际消息接收频率，可能与节点publish调用频率不同，大消息订阅本身也会增加开销。查看仿真是否有时钟和数据：

```bash
ros2 topic info /clock
ros2 topic info /camera/depth/image_rect_raw
ros2 topic info /camera/color/image_raw
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

深度按自己的optical frame解释，RGB使用color optical frame；不应靠改frame字符串将未对齐深度变成彩色坐标。

## 6. RViz怎么查看

使用本次`./run.sh`新打开的RViz，Fixed Frame应为base_link、Views应为Orbit。

| 内容 | 类型/话题 |
|---|---|
| 重建Mesh | MarkerArray：/ur7e_nvblox/mesh |
| 完整3D ESDF的显示采样 | PointCloud2：/ur7e_nvblox/esdf_points |
| 原始RGB | Image：/camera/color/image_raw |
| 原生TSDF层 | PointCloud2：/ur7e_nvblox/tsdf_layer |
| 原生颜色层 | PointCloud2：/ur7e_nvblox/color_layer |
| 相机轨迹 | Path：/ur7e_nvblox/camera_path，来自机械臂TF |

看Mesh时勾选Wheel RGB Mesh、暂时取消3D ESDF；看距离场时反过来。ESDF是有体积的距离采样，叠加在Mesh上会遮挡表面细节。鼠标左键旋转、中键平移、滚轮缩放；可选择保存的Top overview视角。

默认不自动移动机械臂，静止相机仍持续接收和融合新图像，足以检查实时链路。多视角完整性与静止持续更新是不同测试；本次验证没有发送任何机械臂运动指令。

## 7. 本轮真实现场验证结果

直接使用`./run.sh`，未提供rosbag或外部mapper_config。现场Gazebo和RViz均开启，初始姿态静止。预热后观察约30秒，稳定计数窗口29.06秒。验证订阅者同时订阅Mesh和ESDF。

| 指标 | 本轮现场值 |
|---|---:|
| Mapper深度接收 | 29.98Hz |
| Mapper颜色接收 | 29.98Hz |
| 深度融合调用 | 22.27Hz |
| 颜色融合调用 | 21.92Hz |
| ESDF更新/发布调用 | 21.13Hz |
| ESDF订阅端接收 | 20.82Hz |
| Mesh发布调用 | 4.58Hz |
| 末帧ESDF导出点 | 469,441 |
| CameraInfo接收→点云接收代理P50 | 173.28ms |
| 同上代理P95 | 207.96ms |
| 仿真秒/墙钟秒 | 0.998 |

这是本机一次静止现场观察的结果，不代表每次启动或运动时都相同。回放结果、GPU完成频率和RViz画面FPS都不能直接替代本表。延迟是接收代理，未测相机曝光到屏幕的完整延迟。

现有metrics最近样本的host平均耗时：ESDF更新11.11ms、导出/等待17.17ms、点云打包30.76ms、Mesh更新/回读44.27ms。5mm的地图计算与全量输出、两个GUI的负载均存在，所以33.4Hz参数不保证达到30Hz。这次主要验证默认启动和实时链路，并未重新进行全场景精度验收，也未解决此前的颜色扩展或ESDF少量距离异常。

数据：[live_result.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_default_5mm_20261111/live_result.json)、[启动日志](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_default_5mm_20261111/launch.log)。验证结束只关闭自己启动的域164/端口11665实例，用户其他实例未被终止。

## 8. 保存本次自己的结果

保持终端1运行，在已加载环境的终端3执行：

```bash
TEST_OUT="$PWD/artifacts/live_5mm_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUT"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/map.blox'}"
ros2 param dump /ur7e_nvblox > "$TEST_OUT/params.yaml"
ros2 topic echo /ur7e_nvblox/metrics --once > "$TEST_OUT/metrics.yaml"
```

PLY为实际三角网格，可用CloudCompare打开；BLOX保存TSDF/Color/ESDF等地图层，不能代替独立Mesh PLY。

## 9. 运行中进行高位左右扫描

保持仿真和Mapper运行，另开终端。以下仅适用于本项目Gazebo仿真。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=164 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11665
source ./env.sh

# 只检查仿真、关节反馈和控制器，不发送运动
python3 tools/view_sweep.py --domain-id 164 --check-only

# 在高位观察姿态附近左右各4cm，扫描35秒，结束返回中心
python3 tools/view_sweep.py --domain-id 164 --seconds 35 --amplitude 0.04
```

一个左右往复周期约31.4秒，35秒可覆盖正负两侧。需要扩大时可使用`--seconds 65 --amplitude 0.08`，最大允许幅度为左右各8cm。轨迹由已有离散关节解插值执行，沿世界Y方向小幅扫描；不是环绕，也不是任意目标点的通用运动规划。

工具验证推进中的仿真时钟、Gazebo机器人描述及活动的forward_position_controller，仅从预设初始姿态或高位中心启动。若提示当前姿态不匹配，不应绕过检查直接发送任意关节命令。正常完成返回高位中心；Ctrl+C仅停止继续发布，控制器保持最后目标，不自动返回中心。扫描期间可在另一个终端运行`python3 tools/watch_realtime.py`查看现有融合和ESDF指标。

本次仅为工具增加`--domain-id`参数（默认135保持兼容），以支持当前164域；无需重新编译。已经在当前164域完成只读检查，仿真反馈和控制器正常，当前关节与高位中心吻合；没有代替用户执行扫描。
