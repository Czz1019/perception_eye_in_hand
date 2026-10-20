# RViz空白 / 看不到相机：本次修正及自行运行步骤

本次只修改文件，未构建、启动或测试。不能将下面的预期输出当成已确认结果。

## 已确认的问题

1. 用户粘贴的grep表达式缺少末尾单引号，bash显示`>`是在等待命令结束。此时继续输入的内容会进入未结束的命令，并非新的ROS查询。先Ctrl+C，再一条一条执行下面的命令；下面不使用grep。
2. 原RViz配置只有模型、TF和ESDF，没有Image显示。现在增加默认开启的Camera RGB和默认关闭的Camera Depth。
3. 场景、mapper和诊断终端必须使用同一ROS域、相同本机发现设置。新增根目录`source_gazebo_env.sh`供三者共用；未显式设置时默认域78、本机发现1、Gazebo地址http://127.0.0.1:11478，已有环境变量优先。脚本会打印实际环境。
4. 本场景使用本地资产，launch设空GAZEBO_MODEL_DATABASE_URI，避免gzclient请求在线Insert模型目录。metal_peg_board等在线模型名称查询错误，不足以证明gzserver/深度相机失败；必须另看服务器日志。

## 第一步：结束旧程序、重新构建

在旧场景和旧mapper/RViz的启动终端分别Ctrl+C。已运行节点不会自动加载修改。不使用广泛pkill。

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
```

本版及上一版涉及launch/RViz资源和C++插件，所以先构建。若失败，停止后续步骤，保留完整构建错误。

## 第二步：每个终端显式统一环境

分别在三个终端执行；显式export可以覆盖终端里遗留的其他域。

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
source ./source_gazebo_env.sh
```

三处打印的环境应一致。ROS_DOMAIN_ID改变后，旧进程不会迁移到新域，必须重启旧进程。

## 第三步：终端一只启动场景

```bash
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

保持运行。如果日志出现gzserver退出/模型生成失败/控制器加载失败，先解决这里的问题，不要继续启动mapper。在线模型目录错误与服务器退出需要区分。

若端口占用，可以自行查看：

```bash
ss -ltnp 'sport = :11478'
```

从原启动终端关闭旧服务器，或确认它就是要复用的场景。不要只关闭窗口而留下服务器。

## 第四步：终端三检查场景数据

每次只复制一行，不要将终端提示符也复制进去。

```bash
ros2 node list --no-daemon
```

```bash
ros2 service list
```

```bash
ros2 topic list
```

预期场景节点包含robot_state_publisher、controller_manager、gazebo_realsense_camera；服务包含pause_physics、reset_simulation、spawn_entity；话题包含clock、joint_states、camera/depth/image_rect_raw。实际名称带前导`/`。

- node list没有场景节点：优先核对三个启动终端打印的环境和gzserver是否退出。此时RViz空白不应先归咎于nvblox。
- node list正常、CLI服务/话题疑似旧缓存：自行执行`ros2 daemon stop`及`ros2 daemon start`后重查。
- 场景节点存在但相机话题不存在：查看场景终端是否成功加载RealSense Gazebo ROS插件。

进一步分别查看（各条等待数据；长时间没有输出时Ctrl+C）：

```bash
ros2 topic echo /clock --once --qos-reliability best_effort
```

```bash
ros2 topic echo /joint_states --once --qos-reliability best_effort
```

```bash
ros2 topic echo /camera/depth/camera_info --once --qos-reliability best_effort
```

```bash
ros2 topic hz /camera/depth/image_rect_raw --qos-reliability best_effort
```

最后一条持续运行，观察数秒后Ctrl+C。不要echo整张Image的像素数据。

## 第五步：终端二启动mapper和本版RViz配置

```bash
./run_gazebo_mapping.sh run_rviz:=true
```

RViz里应该有：Camera RGB、Camera Depth、RobotModel、TF、ESDF observed。

- **Camera RGB**是独立图像面板，不是3D窗口中的点云。默认开启，话题/camera/color/image_raw，Best Effort，raw传输。
- **Camera Depth**可手动勾选，话题/camera/depth/image_rect_raw，显示原始深度图。
- **RobotModel**：Fixed Frame=base_link；robot_description采用Reliable、Transient Local。
- **ESDF observed**：/realtime_nvblox/esdf_observed，Best Effort。

若没有这两个Image项，说明打开的是旧配置或旧RViz。关闭旧RViz，重新从上面的launch启动；也可以在RViz的File/Open Config选择工程内`src/realtime_nvblox/rviz/gazebo_esdf.rviz`。不要同时运行两个mapper。

看图期间会多订阅RGB图像；若只做性能测量，可取消Camera RGB/Depth勾选。

## 第六步：相机画面正常后设置观察姿态

终端三执行：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

只平滑调整仿真关节、清图并等待地图就绪，不执行完整运动性能测试。初始姿态朝外，ESDF为空可能只是还没看到障碍物。

脚本等待Gazebo服务最多15秒；如果失败，将实际错误及Available services列表保留。不要删除保护检查后向未知控制器发命令。

随后查看：

```bash
ros2 topic echo /realtime_nvblox/stats --once
```

判断：

| 现象 | 下一步 |
|---|---|
| 没有stats | 看mapper终端是否异常退出、ROS域是否一致 |
| depth_received=0 | 检查深度话题与mapper配置 |
| depth_received增加但tf_missing持续增加 | 检查joint_states和图像时刻TF，不用latest代替 |
| error非空 | 保留完整error及mapper traceback |
| depth_integrated增加、known_cells=0 | 检查实际深度画面和观察姿态 |
| known_cells>0、RViz仍没有点 | 核对ESDF显示启用、Topic、Best Effort、Fixed Frame以及显示项Status |

新版ESDF是稀疏有效体素，不要求固定52111点。Camera RGB与ESDF是两条不同显示链：图像可显示但ESDF为空，或RGB禁用但深度建图正常，都需要分别判断。

## 仍失败时需要保留的内容

三个终端的环境打印；未截断的gzserver/mapper错误；`ros2 node list --no-daemon`输出；RViz Displays里红色Status具体文本。只给gzclient在线模型目录错误，无法判断整个ROS数据链是否正常。

完整算法说明、RealSense ROS驱动迁移和由用户执行的性能测试计划见GAZEBO_MAPPING.md。
