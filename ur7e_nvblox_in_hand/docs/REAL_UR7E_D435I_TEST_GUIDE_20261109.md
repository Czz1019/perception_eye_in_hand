# UR7e＋D435i实机感知测试：完整操作说明

日期：2026-11-09。对象为新工程 `ur7e_nvblox_in_hand`。本轮只核查代码、历史实测记录和官方接口，新增操作文档/输入示例；没有连接机器人、启动相机或执行实机运动。下列频率是配置目标，实际结果需由本次操作采样。

## 1. 从历史实测中确认的设备信息

| 项目 | 代码/历史实测记录 |
| --- | --- |
| 机械臂 | 用户确认UR7e |
| 控制柜IP | 192.168.56.100 |
| PC回连地址 | 历史为192.168.56.38，需按当前网卡确认 |
| PolyScope | 历史记录5.21.2，当前版本未现场读取 |
| 相机 | D435i，历史SN 349622072343，固件5.15.1.55、USB3.2 |
| 世界/地图frame | base_link |
| 工具frame | tool0 |
| 深度frame | camera_depth_optical_frame |
| 颜色frame | camera_color_optical_frame |
| 相机机身frame | camera_link |
| 深度输入 | /camera/camera/depth/image_rect_raw、/camera/camera/depth/camera_info |
| 颜色输入 | /camera/camera/color/image_raw、/camera/camera/color/camera_info |
| 工厂运动学校准 | 历史 /home/wqj/ur_arm/my_robot_calibration.yaml |
| 手眼标定原件 | 原仓库 src/ur7e_real_static/config/handeye_d435i.yaml |

依据：[旧实机驱动脚本](../../src/ur7e_real_reconstruction/hardware/start_robot_driver.sh)、[相机脚本](../../src/ur7e_real_reconstruction/hardware/start_camera.sh)、[安装TF脚本](../../src/ur7e_real_reconstruction/hardware/start_camera_mount.sh)、[2026-09-27实测报告](../../src/ur7e_real_static/report/2026-09-27_real_scan_report.md)、[2026-09-28彩色重建报告](../../src/ur7e_real_reconstruction/report/2026-09-28_real_reconstruction_report.md)。

旧报告曾记录Dashboard返回UR5，而用户确认型号为UR7e。启动时保留驱动的型号/校准检查；若当前出现不匹配，应核实铭牌、控制器与驱动支持，不通过关闭校验或随意改ur_type绕过。

## 2. 本次实机测试架构与范围

```text
UR官方驱动 → joint_states → 校准robot_state_publisher → base_link→tool0
                                                        ↓ 手眼安装TF
D435i驱动 → 内部camera_link→depth/color optical TF ← camera_link
   ↓ 原始深度/RGB与各自CameraInfo
本工程按图像时刻查TF → wheel C++/CUDA建图 → Mesh/3D ESDF/轨迹/统计
```

D435i的IMU不是本次位姿来源；不需要cuVSLAM。本工程不会自动启动机械臂驱动，也不会给实机发送运动轨迹。先静止单视角，再由操作员示教器手动改变观察姿态；不要运行tools/view_sweep.py、sim_test.py、scene.sh或旧自动RTDE扫描程序。

现场按机器人已有安全规程操作，确认相机/支架/线缆固定、负载/TCP设置正确、运动区域有净空并可随时在示教器停止。当前Mapper没有机器人自体过滤，也没有集成实机避障控制；ESDF显示正常不代表已具备自动运动安全条件。

## 3. 软件安装与编译

目标平台为Ubuntu22.04 amd64、ROS2 Humble、系统Python3.10、兼容驱动/Toolkit及本工程固定wheel。基础安装见[迁移说明](MIGRATION_DEPLOYMENT_20261109.md)与[系统依赖说明](../../docs/deployment/SYSTEM_SETUP_UBUNTU2204.md)。本机已装版本记录：UR driver2.14.0、ur_description2.8.0、RealSense wrapper4.58.4、ROS librealsense2.57.7；这不等于已在本轮验证它们与现场设备的组合。

仅当缺少时安装：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ur-robot-driver ros-humble-ur-calibration \
  ros-humble-ur-description ros-humble-controller-manager \
  ros-humble-realsense2-camera 'ros-humble-librealsense2*' \
  ros-humble-rmw-cyclonedds-cpp ros-humble-tf2-tools ros-humble-rqt-image-view
```

不要为迁移同时安装另一套/usr/local版RealSense SDK。驱动安装参考[官方RealSense说明](https://github.com/realsenseai/realsense-ros#installation-on-ubuntu)。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export UR7E_CUDA_ARCH=89
./build.sh
source ./env.sh
/usr/bin/python3 tools/check_wheel.py
/usr/bin/python3 tools/check_backend.py
```

最后一条为合成GPU契约检查，不连接硬件。其他GPU需改UR7E_CUDA_ARCH。

## 4. 所有实机终端统一环境

本次建议全部使用ROS_DOMAIN_ID=135，避免混入旧域0程序。若必须接入域0的既有驱动，应把下列135在**所有**终端统一改为0，不能只改Mapper。

每个新终端先执行：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
export UR7E_ROBOT_IP=192.168.56.100
export UR7E_PC_IP=192.168.56.38
export UR7E_REAL_DIR="$HOME/ur7e_real_test"
mkdir -p "$UR7E_REAL_DIR"
```

UR7E_PC_IP必须是当前PC可从机器人回连的有线IP。以下仅检查网络，不改变配置：

```bash
ip -br -4 addr
ip route get "$UR7E_ROBOT_IP"
ping -c 3 "$UR7E_ROBOT_IP"
```

若route显示src不是192.168.56.38，把UR7E_PC_IP改成正确地址，并同步更新示教器External Control主机IP；必要时按现场网络方案配置PC与控制柜同网段。ROS_LOCALHOST_ONLY限制ROS/DDS发现，不影响UR驱动通过TCP/RTDE连接控制柜。

所有进程先在同一PC部署，实机不用Gazebo，也不用/clock，use_sim_time必须false。旧Conda环境与旧overlay不要混入。若必须使用外部UR驱动overlay，在UR驱动终端source本工程env.sh后再加载它；不要让其覆盖Mapper的库环境。

## 5. 准备真实机器人校准与External Control

### 5.1 工厂校准不是手眼标定

历史工厂校准文件在旧用户/home/wqj下，当前机器未找到，不能直接照抄路径。若有同一台机器人的已校验原件，复制为 `$UR7E_REAL_DIR/ur7e_factory_calibration.yaml`；否则在机器人上电、保持静止时提取：

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:="$UR7E_ROBOT_IP" \
  target_filename:="$UR7E_REAL_DIR/ur7e_factory_calibration.yaml"
```

提取完成后退出该launch，检查文件存在。该步骤来源于[UR官方校准说明](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html)，参数也已按本机Humble launch核对。工厂校准用于base→tool运动学，手眼标定用于tool→camera，二者不能互相替代。

### 5.2 示教器

沿用现场已配置的External Control URCap，核对其PC IP与50002端口及机器人网络服务设置。需要新装时按[Humble官方机器人设置](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/doc/installation/robot_setup.rst)进行。若现场实际为PolyScope X，按该系统对应驱动流程重新确认，不套用历史PolyScope5步骤。

本次保持运动控制器inactive，不启动旧SAGE/RTDE自动控制程序。由操作员完成上电/使能等现场操作，本文不提供自动上电或解除保护停止命令。

## 6. 终端A：UR7e驱动与关节TF

先执行第4节环境块，再运行：

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:="$UR7E_ROBOT_IP" reverse_ip:="$UR7E_PC_IP" \
  kinematics_params_file:="$UR7E_REAL_DIR/ur7e_factory_calibration.yaml" \
  use_fake_hardware:=false headless_mode:=false \
  activate_joint_controller:=false launch_rviz:=false
```

操作员按既有流程在示教器启动External Control，等待真实关节反馈及校准检查正常。启动驱动不代表需要激活轨迹控制器；本文不发送轨迹。官方启动接口参考[Humble launch源码](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/launch/ur_control.launch.py)。

终端B（先执行第4节环境块）：

```bash
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states --wall-time
```

Ctrl+C后：

```bash
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_link tool0
```

Ctrl+C后继续后续操作。必须确认是真反馈、时间戳持续推进、模型姿态与实物一致；若已有同域同话题robot_state_publisher，不再启动第二个。UR官方模型不包含相机外观是正常现象，不能因此加载Gazebo模型再发一套重复TF。

## 7. 终端C：独立启动D435i，输入30Hz

先执行第4节环境块；确认Realsense Viewer和其他相机节点已退出，检查USB3连接。使用已安装wrapper的实际参数名：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera serial_no:=_349622072343 \
  enable_depth:=true enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 rgb_camera.color_format:=RGB8 \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  pointcloud.enable:=false align_depth.enable:=false \
  enable_sync:=true publish_tf:=true
```

序列号若更换了相机需改为实际设备。参数由[当前官方launch](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py)及本机4.58.4安装文件核对。开启同步不代表跨设备TF与曝光已自动对齐。

**当前工程内置launch_camera入口不作为本次推荐命令**：它使用旧 `depth_module.profile/rgb_camera.profile` 名称且publish_tf=false。独立启动后，Mapper设置launch_camera=false；避免重复打开相机。

终端B检查：

```bash
ros2 topic list -t
ros2 topic echo /camera/camera/depth/camera_info --once --qos-reliability best_effort
ros2 topic echo /camera/camera/color/camera_info --once --qos-reliability best_effort
ros2 topic hz /camera/camera/depth/image_rect_raw --wall-time
```

Ctrl+C后检查颜色：

```bash
ros2 topic hz /camera/camera/color/image_raw --wall-time
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

正常预期为640×480、深度16UC1毫米或32FC1米、RGB为rgb8/bgr8且各自CameraInfo frame匹配。点云输出由Mapper产生，无需打开相机SDK点云增加负载。30Hz掉帧先查USB/曝光/系统负载；必要时把两路profile都退为640x480x15诊断，并记录输入变化，不能当同条件30Hz性能结果。

## 8. 终端D：手眼TF，只发布一条机身安装边

历史真实标定为tool0→camera_color_optical_frame，相机驱动负责camera_link→各内部optical frame。推荐链为：

```text
base_link → tool0 → camera_link → camera_color_optical_frame
                             └→ camera_depth_optical_frame
```

**不要再发布tool0→camera_color_optical_frame与SDK内部父子链争用同一个child。** 如果URDF或现有静态节点已包含tool0→camera_link，跳过本节发布命令，只验证。

若当前仍为历史同一相机、同一固定挂载，且手眼标定和SDK内部外参经核对未变，可使用旧实测脚本中已组合的机身变换：

```bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id tool0 --child-frame-id camera_link \
  --x -0.0406379061113624 --y -0.05902311421350448 --z -0.012384254831755863 \
  --qx 0.0025872583947125077 --qy -0.002933202909281628 \
  --qz 0.9251005977793049 --qw -0.3797019705019513
```

这组数值来自[旧真实安装脚本](../../src/ur7e_real_reconstruction/hardware/start_camera_mount.sh)，不是仿真挂载参数。只凭文件存在不能证明今天仍然标定正确；支架重新安装、相机更换或标定更新后，应按下式重新组合：

```text
T_tool0_link = T_tool0_color × inverse(T_link_color)
T_base_depth(t) = T_base_tool0(t) × T_tool0_color × T_color_depth
```

不能把tool0→color的平移/四元数直接用于tool0→depth或tool0→link。也不要使用本工程 `ur7e_d435_sim_clearance.yaml` 或其他旧日期的仿真安装文件。

终端B验证以下两条，分别Ctrl+C结束：

```bash
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

tf2_echo主要核查链路，不能证明图像采集时刻TF也可用；这一点由后面Mapper的成功融合计数与TF时间检查共同确认。若树已连接但融合计数不增长，应继续检查时间/CameraInfo/队列。

## 9. 终端E：先做静止深度＋3D ESDF测试

先执行第4节环境块；保持机械臂静止、目标静止且处于约0.3～1m的清晰视野，先排除手腕或支架遮挡。本次输入模板在[real_ur7e_d435i.yaml](examples/real_ur7e_d435i.yaml)，明确关闭初始颜色融合，输入量程为0.2～3m。

```bash
./run.sh start_scene:=false launch_camera:=false use_sim_time:=false \
  run_rviz:=false \
  mapper_config:="$PWD/docs/examples/real_ur7e_d435i.yaml"
```

不要启动旧perception感知节点、nvblox_in_hand或workspaces mapper。无需MoveIt或SAGE即可做本次感知测试。

### 9.1 终端F：RViz使用真实话题与系统时间

现有RViz配置的RGB话题是仿真单层/camera命名空间，复制一份实机视图并只修改显示话题：

```bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import os,yaml
p=Path('/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/rviz/diagnostic.rviz')
d=yaml.safe_load(p.read_text())
for item in d['Visualization Manager']['Displays']:
    if item.get('Class')=='rviz_default_plugins/Image':
        item['Topic']['Value']='/camera/camera/color/image_raw'
out=Path(os.environ['UR7E_REAL_DIR'])/'real_d435i.rviz'
out.write_text(yaml.safe_dump(d,sort_keys=False))
print(out)
PY
rviz2 -d "$UR7E_REAL_DIR/real_d435i.rviz" --ros-args -p use_sim_time:=false
```

此终端也先执行第4节环境块。若使用脚本 `rviz.sh`，必须指定 `USE_SIM_TIME=false ./rviz.sh diagnostic`，因为脚本默认仿真时间。复制后的视图更适合实机RGB话题。

Fixed Frame设base_link；初始显示3D ESDF。Mesh默认可能未勾选，添加/启用MarkerArray的 `/ur7e_nvblox/mesh`；Depth-only阶段没有真实RGB着色属于预期。UR RobotModel从/robot_description读实际模型；D435模型外观未附加不影响内部TF与建图。

RViz里ESDF的彩虹是距离，不是物体材质颜色。observed点可在已观测自由空间与表面内侧，unknown不应被当作安全自由空间。Orbit视图用鼠标旋转/平移/缩放，不依靠机械臂运动来改变RViz观察视角。

### 9.2 终端B检查

```bash
ros2 param get /ur7e_nvblox use_sim_time
ros2 param get /ur7e_nvblox voxel_size
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

成功条件：error为空、depth_integrated持续增加、esdf_updated增加、native_esdf_points>0，且输出stamp随新深度前进。40Hz融合门限和33.4Hz更新/显示都是上限；实际输入是30Hz，不能把参数值当实测。

## 10. 再开启颜色融合

先核对彩色CameraInfo的distortion_model、D、K、P及图像编码。当前backend的Sensor使用K且不传入D，不会对原始RGB做畸变校正；历史2026-09-28报告记录当时彩色D全零，但不能据此假定今天也全零。

- 若当前RGB可按其K作为无畸变针孔图像使用，继续下面颜色测试。
- 若RGB有非零畸变，需先校正再输入。可用image_proc的rectify_node；同时必须保证Mapper使用的K等于校正图像有效内参（通常来自P左3×3）。若K/P不同，单改图像话题不够，需补齐正确CameraInfo适配后再作彩色质量验收。

可选校正工具入口（只有需要时安装/启动，不替代内参检查）：

```bash
sudo apt-get install -y ros-humble-image-proc
ros2 run image_proc rectify_node --ros-args \
  -r __ns:=/camera/camera/color \
  -r image:=image_raw -r camera_info:=camera_info -r image_rect:=image_rect_color
```

image_proc该节点只发布校正图像，不自动替本工程改写CameraInfo。参考[Humble校正节点源码](https://github.com/ros-perception/image_pipeline/blob/humble/image_proc/src/rectify.cpp)。需要使用该入口时，重启Mapper并把color_topic指向image_rect_color，匹配的color_info_topic也必须正确。

对当前可直接使用原始RGB的情况，保持静止，先清空单深度试验地图，再开启颜色；服务/参数命令在终端B执行：

```bash
ros2 param set /ur7e_nvblox enable_depth false
ros2 service call /ur7e_nvblox/clear_map std_srvs/srv/Trigger '{}'
ros2 param set /ur7e_nvblox enable_color true
ros2 param set /ur7e_nvblox enable_depth true
```

看到color_integrated递增并检查彩色Mesh；颜色层 `/ur7e_nvblox/color_layer`是原生颜色体素，不等于RGB图像。只开启颜色后再用该阶段的记录评估着色质量，不把前一阶段无色地图混为结果。

## 11. 操作员手动多视角重建

第一轮建议“停稳采集”，便于隔离时间同步问题：

1. 在当前姿态静止采集约5～10秒，检查表面位置与已知实物尺寸。
2. 暂停深度/颜色融合；通过现场示教器已有操作方式手动改变观察方向，避开支架线缆及近距离自体遮挡，不调用Gazebo或自动RTDE扫描脚本。
3. 到达新视角停稳，确认真实joint_states/TF恢复更新，再恢复融合并采集5～10秒。
4. 做3～5个小范围不同视角，先观察表面是否重复叠层、漂移、串色。通过后才考虑低速连续移动时的实时性/残影测试。

暂停：

```bash
ros2 param set /ur7e_nvblox enable_depth false
ros2 param set /ur7e_nvblox enable_color false
```

恢复（颜色已核对后）：

```bash
ros2 param set /ur7e_nvblox enable_depth true
ros2 param set /ur7e_nvblox enable_color true
```

暂停参数不是机械臂停止命令，也不是严格完成所有GPU任务的屏障；暂停后待融合计数稳定再移动。现场示教器模式切换若导致External Control停止，可能需重新启动程序以恢复驱动数据链；必须检查实际反馈，不假定相机pose仍在更新。

本工程camera_path最多保存3000个约5Hz轨迹pose，且对应成功融合的深度；暂停融合期间不会完整记录运动路径。完整原始运动记录需要bag里的joint_states与TF。

## 12. 如何记录实测结果

主要话题：

| 话题 | 内容 |
| --- | --- |
| /camera/camera/depth/image_rect_raw、color/image_raw | 输入图像 |
| /camera/camera/depth/camera_info、color/camera_info | 两路内参 |
| /joint_states、/tf、/tf_static | 真实关节与位姿链 |
| /ur7e_nvblox/esdf_points | 3D ESDF原生体素中心、有符号距离和标记 |
| /ur7e_nvblox/mesh | 标准MarkerArray彩色Mesh |
| /ur7e_nvblox/tsdf_layer、color_layer | 原生地图层 |
| /ur7e_nvblox/camera_pose、camera_path | 图像时刻机械臂相机位姿与路径 |
| /ur7e_nvblox/metrics | 融合/更新/发布计数及计时与库路径 |

分别做静止30秒、手动停稳多视角、可选连续运动三个阶段。频率比较时保持同一v=0.03、640×480、输入档位、范围、订阅者和GUI负载；先记录输入与成功融合，再记录ESDF更新和发布。输出年龄应由接收时刻减图像header时间，在同机系统时钟下计算；跨主机需先做时钟同步。

### 12.1 不新增监测节点的简单频率记录

终端B分别执行，每条采样约30秒再Ctrl+C：

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/camera/color/image_raw --wall-time
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
ros2 topic hz /ur7e_nvblox/mesh --wall-time
```

融合/ESDF内部调用以metrics的wall_hz和totals为准；submit_ms是CPU调用时间，不能当GPU完整内核耗时。ESDF发布率不代表每条都是新的输入信息，需在bag中对header去重并检查时间推进。完整延迟与P95需要后处理，不用RViz左下角FPS替代。

### 12.2 保存地图与统计

使用独立会话目录，避免覆盖旧文件：

```bash
export UR7E_RUN="$UR7E_REAL_DIR/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$UR7E_RUN"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_RUN/map.blox'}"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_RUN/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_rates ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_RUN/rates.json'}"
ros2 service call /ur7e_nvblox/save_timings ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_RUN/timings.json'}"
ros2 param dump /ur7e_nvblox > "$UR7E_RUN/mapper_parameters.yaml"
```

地图保存服务占用GPU所有者，放在性能采样结束后执行。clear_map不要在正式会话中误调用。

### 12.3 原始数据与结果分开录bag

另一个终端先执行第4节环境块，设同样的UR7E_RUN实际路径，再录制；优先从设备节点启动前开始录，或确认/TF静态数据已在bag中捕获：

```bash
ros2 bag record -o "$UR7E_RUN/raw_inputs" \
  /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw /camera/camera/color/camera_info \
  /joint_states /tf /tf_static /robot_description /ur7e_nvblox/metrics
```

Ctrl+C正常结束，执行 `ros2 bag info "$UR7E_RUN/raw_inputs"`核查。ROSbag可能需要QoS覆盖来捕获best_effort相机流及transient_local静态TF，按topic info的实际QoS确认。若录制/高频joint_states导致掉帧，记录负载变化；不要静默删掉时间戳或TF信息。

ESDF/Mesh结果可另录，但大消息会增加负载：

```bash
ros2 bag record -o "$UR7E_RUN/map_outputs" \
  /ur7e_nvblox/esdf_points /ur7e_nvblox/mesh \
  /ur7e_nvblox/camera_pose /ur7e_nvblox/camera_path /ur7e_nvblox/metrics
```

截图在当前RViz中保存或用桌面截图，注明时间与阶段。本工程自动截图工具按窗口标题匹配，当前自定义real_d435i.rviz标题可能不匹配，不能盲目调用仿真截图/自动测试脚本。

## 13. 停止顺序与排错

结束采样→保存→Ctrl+C结束bag→停止Mapper/RViz→由操作员按现场流程停止External Control→结束相机与UR驱动。停止建图节点不会让机器人自动停止；需要机器人停止时使用现场认可的停止方式。

| 现象 | 检查 |
| --- | --- |
| UR回连失败 | 当前PC有线IP、reverse_ip、URCap主机IP和端口；历史.38不一定仍有效 |
| 工厂校准文件缺失 | 复制正确原件或重新从该机器人提取，不用默认模型代替精度验收 |
| robot型号/校准错误 | 核实历史UR5/UR7e差异、控制器配置与铭牌，不关闭检查绕过 |
| CameraInfo有、融合0 | 图像frame/尺寸/编码、ROS域、图像时刻TF、真实系统时间 |
| RGB窗口No Image | RViz仍订阅/camera/color/image_raw；实机是/camera/camera/... |
| TF重复/跳变 | 两个robot_state_publisher、两个mount节点或手眼直连optical与SDK父链冲突 |
| 表面双层/运动残影 | 先停稳复测，再查时间戳/TF延迟、工厂校准、手眼和自体污染 |
| 颜色错位/串色 | 各自optical TF、内参/畸变、RGB-D曝光时差；不能直接把深度光学位姿当颜色位姿 |
| 有输入但颜色不融合 | 初始模板故意enable_color=false；通过光学检查后开启 |
| 只有灰Mesh | 先确认color_integrated、RGB格式、颜色层；ESDF彩虹不是RGB材质 |
| 新机频率不及仿真 | USB/曝光、CPU降频、GPU负载、joint_states/TF负载、录bag和可视化开销 |
| ESDF有缺口 | unknown遮挡/无效深度/门限，增加受控视角；不把未知强制填自由 |

## 14. 本轮交付与验收边界

已提供从历史实测参数出发的分终端流程和真实输入YAML，未更改Mapper、驱动或机械臂控制代码。真实UR7e/D435i端到端性能与重建质量尚需本次按文档测试；旧实机4Hz左右记录属于旧应用，之前新工程约26Hz ESDF输出属于仿真，二者都不能当成本轮实机结果。
