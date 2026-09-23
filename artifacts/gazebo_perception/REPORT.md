# Gazebo 感知修改与实测报告

日期：2026-09-23。系统：Ubuntu 22.04 / ROS2 Humble / Gazebo Classic / Python 3.10；GPU RTX 4060 Laptop 8GB，驱动 580.178.04，PyTorch 2.9.1+cu128。

## 结果

真实 Gazebo 深度 → 曝光时刻 TF → 共享 CUDA NvbloxBackend → TSDF/ESDF → ROS2/RViz 已运行。墙/地面视角与球体视角的操作验收均通过。全局 Gazebo 时钟重置暴露了控制插件的独立限制，未把这项失败隐藏为成功，详见下文。

## 来源与修改

核对远端 main 和用户 ZIP，提交均为 `6bf63cc2f75d18751470231e0f912e6ff546da04`。同步落后的核心源码，未为 Gazebo复制后端。9 个核心/真机文件与上游逐字节一致，哈希见 `source_manifest.json`。本机没有 `src/ur7e_nvblox`，未删除或创建替代旧包。原包修改前备份位于 `/tmp/perception_nvblox_before_gazebo_20260923`。

| 文件或组 | 作用 |
|---|---|
| realtime_nvblox/gazebo_depth.py | 两种深度格式、大小端/步长、内参降采样、米制 Calibration、TF矩阵、时间窗口验证 |
| realtime_nvblox/gazebo_ros_node.py | 独立 ROS 前端；图像严格时刻 TF；输入/输出 latest-only；GPU/发布线程；epoch、保存/重置、统计 |
| config/gazebo_robot.yaml | 文档指定的仿真默认参数；不改真机外参 |
| launch/gazebo_mapping.launch.py | config/python_executable/run_rviz 参数，独立 gazebo_ros_mapper |
| rviz/gazebo_esdf.rviz | base_link、RobotModel、TF、observed 与可选 full grid |
| setup.py / package.xml | 保留真机入口；新增仿真入口与时钟/TF消息依赖 |
| 根目录 run_gazebo_mapping.sh | 清理混入的其他工作空间环境，启动本工程感知 |
| examples/verify_gazebo_mapping.py | 观察姿态、静止/运动计量、严格 TF 对比、区域稳定性、保存/重置；默认只读 |
| examples/verify_gazebo_rejection.py | 独立域注入不匹配 TF/非法时间戳，验证两次时钟回退后的 CUDA 地图恢复 |
| test/test_gazebo_depth.py | 格式/内参/时间/TF/epoch 与无硬件 SDK 导入回归 |
| GAZEBO_MAPPING.md | 三终端启动、操作验证、服务调用与限制说明 |
| nvblox_backend.py、runtime.py、ros_node.py、realsense_source.py 等 | 同步指定 main；Gazebo 不调用真机 runtime，所有共享核心仍匹配上游 |
| robot_pose.py、robot_eye_in_hand.yaml、配套配置/测试 | 同步上游真机姿态路线；未将仿真标定填入真机配置 |

完整逐文件变更列表见 `source_manifest.json`；本次未改变 UR7e 外参及模型安装位姿。

## 构建与运行

实际构建（成功）：

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
```

实际仿真与 RViz 运行：

```bash
ROS_DOMAIN_ID=78 GAZEBO_MASTER_URI=http://127.0.0.1:11478 ./run_ur7e_d435_scene.sh gazebo_gui:=false
ROS_DOMAIN_ID=78 ./run_gazebo_mapping.sh config:=/home/czz/perception_eye_in_hand/artifacts/gazebo_perception/test_config.yaml run_rviz:=true
```

实际验证：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ROS_DOMAIN_ID=78 python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --exercise-simulation
ROS_DOMAIN_ID=78 python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --exercise-simulation --view sphere --seconds 15 --output artifacts/gazebo_perception/sphere_verification.json
ROS_DOMAIN_ID=79 python3 src/realtime_nvblox/examples/verify_gazebo_rejection.py
```

单元测试32通过；独立启用真实 CUDA 的 GPU 测试1通过。测试命令使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，绕开本机无关的损坏 typeguard pytest 插件。未安装/升级全局依赖。GPU运行有现有 requests 版本警告，不影响本次验证。

## 实时性：墙钟计量，RViz 同时运行

| 窗口 | 深度输入 Hz | 融合 Hz | ESDF Hz | 接收到发布 P50 ms | P95 ms | 仿真/墙钟速率 |
|---|---:|---:|---:|---:|---:|---:|
| 墙/地面：静止 | 10.73 | 9.88 | 4.99 | 103.8 | 145.4 | 0.993 |
| 墙/地面：运动 | 12.61 | 10.83 | 5.64 | 105.4 | 145.6 | 1.001 |
| 球体：静止 | 11.53 | 9.93 | 4.53 | 107.4 | 145.4 | 0.993 |
| 球体：运动 | 9.71 | 8.47 | 4.35 | 107.4 | 143.9 | 1.000 |

延迟统计是最近最多512次采样的滚动分位数，不是硬实时上界；频率按窗口计数差除以墙钟时间。GPU融合/查询读回 P95 约5–6ms，接收到发布包含等待仿真时钟/TF、数据调度与发布。RViz之前的初测较快（约8Hz ESDF），但验收以上表为准。配置10Hz是上限，本机含RViz的实测没有达到10Hz，不宣称硬实时达标。

## 几何与操作结果

- 完整网格：52111 点，字段 x/y/z/rgb/distance；observed 非空（墙视角 2590 点，球视角 2665 点），保留完整网格 unknown。
- 严格时刻 TF：两次共 454 个位姿样本，最大平移差 2.78e-16 m，符号等价四元数差 <3e-16。
- 运动：相机范围约 7.30 cm，过程中融合与ESDF持续。
- 固定世界区域距离变化：墙视角全局共同表面 P95 4.87 mm；球视角球体区域 89 个共同网格单元，P95 7.46 mm。两面墙与地面均有共同观测。此指标是固定位置 ESDF 距离变化，不是独立标定精度证明；新出现/被遮挡的单元不纳入对比。
- 保存：实际 `base_link_map.nvblox`，最新球视角文件 1859584 字节。
- reset_map：两次运动验收后均自动恢复有观测地图，最新 epoch=4。
- 缺失TF测试：已有9秒TF、图像要求9.9秒，仍丢弃 22 帧，融合/发布均0；非法、过期和未来时间戳分别计数。
- 连续两次时钟回退：独立合成输入中，epoch 1、2均实际重建CUDA地图并恢复非空ESDF，见 `rejection.json`。

## 最后一次 stats

```json
{
  "depth_received": 2017,
  "depth_integrated": 1735,
  "depth_overwritten": 279,
  "tf_missing": 0,
  "stale_depth": 1,
  "esdf_updated": 897,
  "esdf_published": 897,
  "known_cells": 2666,
  "total_cells": 52111,
  "input_stamp_ns": 204422000000,
  "camera_position_base": [
    0.2110245571084245,
    0.40492416100185447,
    0.6816223313507426
  ],
  "receive_to_result_ms": 135.75598100032948,
  "map_epoch": 4
}
```

## 实际节点与话题

```text
/controller_manager
/forward_position_controller
/gazebo
/gazebo_mapping_verification
/gazebo_realsense_camera
/gazebo_ros2_control
/joint_state_broadcaster
/realtime_nvblox
/record_perception_graph
/robot_state_publisher
/rviz2
/transform_listener_impl_61c1c1c69810
```

图中 verification/record 节点属于临时验收工具，结束后退出。`gazebo_realsense_camera` 是仿真插件节点，不是 RealSense SDK驱动。核心话题：`/camera/depth/image_rect_raw`、`/camera/depth/camera_info`、`/joint_states`、`/clock`、`/tf`、`/tf_static`、`/realtime_nvblox/esdf_3d`、`/realtime_nvblox/esdf_observed`、`/realtime_nvblox/camera_pose`、`/realtime_nvblox/stats`。完整类型见 `ros_graph.json`。

## 尚存问题

- 默认机械臂初始姿态向外上方看，observed可能为空；运行观察姿态脚本，不要为了看到点云改标定。
- Gazebo全局reset实验失败记录保存在 `gazebo_global_reset_limitation.json`：/joint_states和动态TF停发；mapper正确清图并丢弃无TF帧。日常用reset_map；全局reset后需重启场景。独立输入的两次epoch恢复测试通过，不能据此声称Gazebo控制插件reset也通过。
- 10Hz配置上限未在RViz并行验收达到；没有做长时间满负荷、硬实时或闭环避障验收。
- 遮挡区域保持unknown；没有动态自体滤除。不同epoch若出现数值完全重叠的延迟Image时间戳，标准消息缺少epoch ID，无法无条件区分，详见运行说明。
- 真机代码保留并核对上游，未连接真实机械臂或相机进行硬件回归。
