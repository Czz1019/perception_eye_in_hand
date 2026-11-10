# ur7e_nvblox_in_hand 实机颜色接入修正、重测及运行说明

日期：2026-11-09。使用现有 UR7e 反馈、D435i 与安装TF；没有发送运动指令，也没有启动硬件驱动。本轮重测是约30秒现有实机数据观察，不是手拖运动质量评测。

## 1. 修正范围与不能混淆的结论

之前“颜色融合累计只有1次”的实测来自 C++ `nvblox_in_hand` 的 `/nvblox_node/metrics`。本轮先测试 Python `ur7e_nvblox_in_hand` 基线，确认它在显式传入正确真实话题后，颜色融合会持续增加，约28Hz短时快照，并没有复现C++的停滞。因此没有修改Python融合公式、权重、TF时间查询或底层库，也没有声称修复了C++异常。

这次修正的是已经确认容易导致“无输入/无显示”的Python实机接入问题：默认话题仍为单层camera前缀、RViz相机话题不随输入参数改变、内置相机启动沿用旧参数名，以及诊断视图默认隐藏Mesh。

## 2. 具体文件与改动

| 文件（相对本工作区） | 修改 | 解决的问题 |
|---|---|---|
| src/ur7e_nvblox_in_hand/launch/bringup.launch.py | 增加 resolve_input_topics；use_sim_time=false且未显式指定时，四路输入自动使用/camera/camera/... | 避免实机误订阅仿真/旧命名空间 |
| 同上 | 显式topic参数优先；仿真继续沿用原话题 | 保留自定义相机与仿真兼容性 |
| 同上 | 内置RViz按最终解析的输入话题重映射四路camera输入，始终加载本包RViz配置 | 避免RGB显示与Mapper输入分离、手动加载错另一包配置 |
| 同上 | 内置相机节点namespace=camera、name=camera；使用depth_module.depth_profile/rgb_camera.color_profile，并开启相机内部TF | 与现有4.58驱动参数及/camera/camera命名一致 |
| src/ur7e_nvblox_in_hand/config/d435.yaml | 更新上述profile参数名，publish_tf=true | 相机自身发布camera_link到optical的内部外参 |
| src/ur7e_nvblox_in_hand/rviz/diagnostic.rviz | 默认显示本包Wheel RGB Mesh | 启动后无需先寻找隐藏的Mesh显示项 |
| tests/test_real_input_topics.py | 增加实机、仿真、自定义话题优先级3项测试 | 防止后续又出现话题选择回退 |

此次没有修改backend.py、node.py、手眼标定、安装TF数值、仿真配置、C++包或workspaces。相机内部TF不等于tool0到camera_link安装TF；安装发布节点仍需单独保持运行。

实机默认规则由use_sim_time决定：如使用外部相机的自定义命名空间，应继续显式提供四路topic；不要假定所有真实相机都使用camera/camera。

## 3. 输入诊断结果

8秒采样得到RGB 240帧、RGB CameraInfo 240帧，共240组精确相同时间戳，编码rgb8。由此排除了“当前RGB与CameraInfo整体无法匹配”的假设；它不证明C++的NITROS/回调层已经正确工作。

图像刚到达时，有部分精确时刻TF查询暂时超前于最新反馈约十几毫秒。Python队列原本会等待后续TF，在有限图像年龄内重试，本次没有改为最新TF，也没有修改时间戳。采样程序的瞬时查询失败率不等于Mapper最终融合失败率。

测试前已保存C++地图为 `real_test_results/color_fix_20261109/cpp_before.nvblx`，随后只停止C++ Mapper及其RViz，硬件继续运行。重测期间确认只有一个/ur7e_nvblox，RViz订阅本包输出。

## 4. 实机重测结果

条件：640×480、RGB目标30Hz、v=0.03m、范围3m、深度/颜色融合门限40Hz、ESDF更新/导出33.4Hz、Mesh5Hz、衰减/半径清理关闭。外部相机已运行，本次没有启动或修改其曝光/滤波参数。

测量程序预热8秒，随后约30秒采样。频率按接收墙钟时间及累计计数差计算；统计包含Mesh和ESDF订阅产生的导出开销。

| 项目 | 结果 | 定义 |
|---|---:|---|
| 深度输入接收 | 28.55Hz | metrics计数差/约29秒快照跨度 |
| 颜色输入接收 | 29.83Hz | 同上 |
| 成功深度融合调用 | 28.55Hz | depth_integrated计数差 |
| 成功颜色融合调用 | 29.79Hz | color_integrated计数差 |
| ESDF更新调用 | 28.55Hz | esdf_updated计数差，不是GPU独立计时 |
| ESDF点云发布 | 28.53Hz | 857条消息，相邻首尾到达时间计算 |
| Mesh发布 | 4.97Hz | 149条MarkerArray消息 |
| ESDF点数 | 3235～3240 | 当前视场中的原生observed ESDF体素 |
| ESDF数据年龄P50/P95 | 47.72 / 55.72ms | 监测器墙钟接收时间减点云所引用深度时间戳 |

数据年龄包含采集时间基准偏差、输入等待、计算、导出和传输等因素，不是GPU核耗时，也不是屏幕显示延迟。地图规模、视角、运动状态改变后结果会变化；不能用本轮频率宣称所有手拖运动均达到相同性能。

## 5. 导出结果与剩余问题

重测导出 `python_after_mesh.ply` 成功，609顶点、691三角面、150种RGB；344顶点为(127,127,127)，占56.49%。这证明网格包含实际颜色，不能以“color_integrated增加”独自证明颜色覆盖完整或投影准确。

此前C++导出为1736顶点、2014面，灰值127约87%。两轮地图规模与观测不相同，因此不能把87%到56.49%直接写成严格的算法改进幅度。自然灰色也可能等于127，灰值比例仅作排查指标，不是精确的未着色标签。

剩余灰色可能与颜色/深度视场差异、遮挡、颜色积分可见性/置信条件、历史几何、内外参及位姿对应有关。当前未量化分解原因。3cm网格对小结构的几何模糊也没有通过本轮接入修正得到解决。

## 6. 验证与文件位置

5项测试通过：3项话题选择测试、既有Mesh线格式测试、ESDF字段契约测试。本包colcon编译通过。

实际证据目录：
`/home/czz/perception_eye_in_hand/real_test_results/color_fix_20261109/`

- python_before.log / python_before.json：显式正确话题的Python基线。
- python_after.log：更新启动逻辑后，不传四路topic的实机运行日志。
- python_after_measurement.json：汇总频率、点数、数据年龄、最后一次完整metrics。
- python_after_events.json：采样原始统计与到达时间。
- python_after_mesh.ply / python_after_mesh_analysis.json：网格与颜色统计。
- python_after_map.blox：原生地图。
- cpp_before.nvblx：切换前保存的C++地图。

这些实机结果位于git忽略的real_test_results中；本文档与源码改动可单独纳入版本管理。

## 7. 编译与完整运行指令

### 7.1 编译本次修改的Python ROS包

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
colcon build --base-paths src --packages-select ur7e_nvblox_in_hand --executor sequential
source ./env.sh
```

只涉及Python/launch/config，不需要重新编译CUDA桥。全新机器尚未构建依赖时，仍需按部署文档完成依赖与完整build.sh流程。

### 7.2 公共硬件保持运行

已有机械臂、相机、安装TF不重复启动；所有终端同域135。相机如未运行，可另开终端：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera serial_no:=_349622072343 \
  enable_depth:=true enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 rgb_camera.color_format:=RGB8 \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  pointcloud.enable:=false align_depth.enable:=false enable_sync:=true publish_tf:=true
```

安装TF发布终端必须保持运行；手眼仍以原tool0→color标定为基础，不能直接把该数值改名给depth。

### 7.3 一条launch启动Mapper及正确RViz

先关闭旧Mapper及其RViz，勿关闭硬件。本轮测试结束时保留了已启动的Python Mapper/RViz供观察；若继续使用它们，不要重复执行启动命令。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=false \
  run_rviz:=true rviz_mode:=diagnostic global_frame:=base_link \
  voxel_size:=0.03 max_range_m:=3.0 enable_color:=true \
  depth_hz:=40.0 color_hz:=40.0 \
  esdf_hz:=33.4 esdf_view_hz:=33.4 mesh_hz:=5.0 layer_hz:=5.0 \
  decay_hz:=0.0 clearing_hz:=0.0
```

无需再手动重映射RViz，也无需加载C++工作区环境。RViz中Mesh和ESDF可交替显示，避免遮挡。本次诊断RViz默认Mesh开启的最后配置改动需要重开RViz才能生效；已开窗口可直接勾选Wheel RGB Mesh。

### 7.4 检查与手拖

另开终端加载同环境：

```bash
ros2 param get /ur7e_nvblox color_topic
ros2 topic echo /ur7e_nvblox/metrics --once --full-length
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

确认颜色融合计数持续增加后再缓慢手拖；仍需机械臂持续提供关节反馈。此Mapper不控制机械臂、不进入自由驱动模式、不提供实际避障保护。

### 7.5 保存最终数据

停稳3～5秒，保持Mapper运行，在已加载项目环境的终端执行：

```bash
export UR7E_COLOR_RESULT_DIR="$HOME/ur7e_real_test/color_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$UR7E_COLOR_RESULT_DIR"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_COLOR_RESULT_DIR/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$UR7E_COLOR_RESULT_DIR/map.blox'}"
```

需要保留整个ESDF过程时，在手拖前单独启动录包，结束后只对录包终端Ctrl+C，保存服务完成后再关闭Mapper：

```bash
ros2 bag record -o "$UR7E_COLOR_RESULT_DIR/bag" \
  /ur7e_nvblox/esdf_points /ur7e_nvblox/mesh /ur7e_nvblox/metrics \
  /ur7e_nvblox/camera_path /joint_states /tf /tf_static /robot_description
```

录包前先创建结果目录变量。录包增加系统负载，如比较性能应保持同样录制条件。
