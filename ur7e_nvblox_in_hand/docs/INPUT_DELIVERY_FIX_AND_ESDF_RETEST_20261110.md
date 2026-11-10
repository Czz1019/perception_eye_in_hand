# 图像缺帧修复与融合、3D ESDF 复测

日期：2026-11-10。修改范围仅为 perception_eye_in_hand/ur7e_nvblox_in_hand，未修改 workspaces 或另一个 C++ 功能包。测试为隔离 Gazebo 仿真，不涉及真实机械臂。

## 1. 修复了什么

此前仿真插件的 Image 与 CameraInfo 使用 Best Effort 发布、有限历史队列。二者虽由同一帧生成，但独立传输；录包存在大量仅 CameraInfo 到达的情况。本次为仿真相机加入可靠发布，并为 Python mapper 加入可选可靠订阅，修复已复现的图像输入缺失问题。

可靠性与历史深度同时调整，因此本轮证明的是这组传输配置的效果，没有把损失精确定位到内核、某个 DDS 队列或录包内部。可靠传输仍有有限队列与资源约束，不构成任何负载下绝不丢帧的承诺。

## 2. 修改文件及作用

| 文件 | 本次修改 |
|---|---|
| third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp | 图像与 CameraInfo 发布 QoS 可选 reliable/best_effort，默认 reliable，历史深度 10；输出所选策略日志。 |
| third_party/realsense_gazebo_plugin/src/RealSensePlugin.cpp | 允许解析新增 imageReliability 项，避免基类拒绝该参数。 |
| urdf/ur7e_d435.urdf.xacro | 新增 sim_camera_reliability 参数并传入插件。 |
| launch/scene.launch.py | 声明并转交仿真相机可靠性参数。 |
| ur7e_nvblox_in_hand/node.py | 新增 input_qos=sensor_data/reliable，对图像与 CameraInfo 使用同一选定 QoS；非法值在启动时拒绝。 |
| config/base.yaml | 声明 input_qos，通用默认 sensor_data，保持外部相机兼容。 |
| launch/bringup.launch.py | 配套仿真启动时自动匹配相机可靠性；用户明确传入 input_qos 时优先采用用户值。 |

上表路径分别相对于 src/ur7e_d435_gazebo 或 src/ur7e_nvblox_in_hand。未改变 CameraInfo 缓存策略、图像时间戳、手眼标定、相机内参、TF 查询方式或底层 TSDF/ESDF 算法。没有用修改时间戳或填充空图像来凑齐帧数。

## 3. 相同静止场景的输入对照

修复前后各录制约 30 秒，实验室场景、640×480、相机设定 30 Hz、GUI 关闭、未运行 mapper。频率按各话题首尾消息间墙钟跨度计算。

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| 深度图 Hz | 24.71 | 29.97 |
| 彩色图 Hz | 24.17 | 29.97 |
| 深度 CameraInfo Hz | 27.85 | 29.97 |
| 彩色 CameraInfo Hz | 28.55 | 29.97 |
| 有深度内参、缺同时间戳图像 | 93 | 0 |
| 有彩色内参、缺同时间戳图像 | 131 | 0 |

修复后深度图、深度 CameraInfo、彩色图、彩色 CameraInfo 各 888 条；两路在共同覆盖区间均无缺失配对。仿真时间/墙钟比例前后均约 0.999，不支持整体仿真降速是主要原因。

## 4. 运动采集与重建测试条件

使用此前检查过的实验室横向扫描轨迹，机械臂运动后返回初始视角。新输入深度图 2605 帧、彩色图 2605 帧，墙钟频率分别为 29.95、29.95 Hz；两路均无 CameraInfo 时间戳缺少对应图像。

重建测试使用同一份新录制输入分别按 1 倍速回放，stride=2、3 各一轮。体素 0.01 m、截断 4 体素即 0.04 m、RGB/Depth 640×480、深度/颜色门限 40 Hz、ESDF 更新与显示上限 33.4 Hz、Mesh 上限 5 Hz。GUI 关闭，保留实际 ESDF 与 Mesh 订阅。地图每轮重新建立。

stride 只改变 3D ESDF 导出显示采样间隔，完整地图体素仍为 1 cm。统计去除回放开始前 10 秒，截止回放完成；延迟为同时间戳深度 CameraInfo 接收至完整 ESDF 点云接收的代理值，非相机曝光至 RViz 显示延迟。

## 5. 本轮实际结果

| 指标 | stride=2 | stride=3 |
|---|---:|---:|
| 深度接收回调 Hz | 29.95 | 29.96 |
| 彩色接收回调 Hz | 29.95 | 29.96 |
| 深度融合调用 Hz | 28.71 | 29.15 |
| 彩色融合调用 Hz | 28.67 | 29.27 |
| 3D ESDF 更新调用 Hz | 23.31 | 24.15 |
| ESDF 发布调用 Hz | 23.30 | 24.13 |
| Mesh 发布调用 Hz | 4.82 | 4.94 |
| 完整非空 ESDF 接收 Hz | 22.95 | 20.25 |
| 实际接收载荷 MB/s | 246.41 | 63.35 |
| 最后一帧 ESDF 点数 | 542648 | 158011 |
| 输入内参至 ESDF 接收延迟 P50，ms | 93.37 | 62.56 |
| 上述延迟 P95，ms | 156.56 | 109.21 |

**融合与更新均为现有代码中的调用计数，不是 CUDA 完成率，也不是 RViz FPS。** Reliable 可以改善消息送达，但不能替融合线程完成计算。完整输入恢复后，内部限频、队列覆盖、过期丢弃、TF 等待、ESDF 导出和 Mesh 读回仍可能限制实际融合与更新。

### 5.1 为什么 ESDF 还没有达到 30 Hz

输入已恢复约 30 Hz，但工作线程仍串行执行深度、颜色、ESDF 更新、导出和 Mesh 读回。dirty_depth 表示地图有变化，多次深度融合可能归并为一次 ESDF 更新；due() 使用当前墙钟加周期设置下一次期限，不补做已经错过的更新。

| 最后最多 200 次调用的主机耗时均值 | stride=2 | stride=3 |
|---|---:|---:|
| ESDF 导出，含 GPU 等待，ms | 15.89 | 5.28 |
| 点云打包与发布，ms | 23.69 | 5.37 |
| Mesh 生成/读回，ms | 36.71 | 34.14 |
| Mesh 打包与发布，ms | 42.04 | 30.22 |

打包发布在独立线程中，不能把这些耗时简单相加视为端到端延迟。Mesh 单次读回约 34–37 ms，已超过 30 Hz 对应的 33.33 ms，仍会周期性阻塞工作线程。

最后诊断快照中，stride=2 累计 depth_overwritten=70、color_overwritten=71；stride=3 为 30、24。这是 Python 内部输入队列满后的覆盖，区别于修复前图像尚未到达接收回调的缺失。另外 depth_gate_skipped 为 40、44，说明源时间戳仍有少量短于 25 ms 的间隔，被既有 40 Hz 门限拒绝；它已不是此前 18 Hz 的主要原因。快照累计计数与稳定区间频率的统计窗口不同，不作逐项总量相减。

stride=3 导出更轻、延迟更低，但本轮完整点云接收频率仍低于 stride=2。输出最新值队列、DDS 和接收端调度可能影响送达；目前没有逐消息传输追踪，不能指定唯一根因，也不能把更少点数直接等同于更高显示频率。

本轮已经修复已复现的输入缺失，融合接近 30 Hz；尚未实现完整 3D ESDF 的稳定 30 Hz 更新与送达。后续应单独测量减少 Mesh 读回频率、按需 ESDF 导出、输入处理与显示解耦的效果。

## 6. 与此前测试的对比

此前输入文件平均只有深度 23.74 Hz、彩色 23.21 Hz，mapper 又存在进一步接收损失。旧表为两轮均值，新表为一轮；输入 bag 重新采集，轨迹与参数相同但时间过程不是同一份数据，因此不是严格的单变量处理性能基准。静止输入 A/B 试验及本轮配对计数单独验证了传输修复效果。

| 指标 | 原 stride=2 | 新 stride=2 | 原 stride=3 | 新 stride=3 |
|---|---:|---:|---:|---:|
| 深度接收 Hz | 17.87 | 29.95 | 18.62 | 29.96 |
| 深度融合调用 Hz | 17.51 | 28.71 | 18.36 | 29.15 |
| ESDF 更新调用 Hz | 16.37 | 23.31 | 16.97 | 24.15 |

![修复前后处理链路频率](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/pipeline_before_after.png)

## 7. 保存结果的实际数据渲染

![实际 ESDF 切片与保存 Mesh 顶点](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110/actual_esdf_and_mesh.png)

图片来自保存的 PointCloud2 和 PLY，非 RViz 截图。上排为 z=0.125 m 的 ESDF 切片，颜色表示到障碍物的带符号距离；它不是物体表面点云。下排为彩色 Mesh 顶点俯视投影，不是带光照的三角面渲染。未知区域不填充。没有在本轮重新做 Mesh/ESDF 几何精度对照，输入完整不等于已有几何误差全部解决。

## 8. 编译与运行

终端 1，编译：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
source ./env.sh
colcon build --base-paths src --packages-select ur7e_d435_gazebo ur7e_nvblox_in_hand --executor sequential
source ./env.sh
```

启动配套仿真与 RViz；该指令不会自动执行扫描运动：

```bash
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 \
  sim_camera_reliability:=reliable input_qos:=reliable \
  voxel_size:=0.01 truncation_vox:=4.0 esdf_mode:=3d esdf_view_stride:=2
```

stride=3 时先关闭前一组仿真，再仅将最后的 esdf_view_stride 改为 3。不要同端口启动两套。

终端 2，检查输入和输出：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 topic info /camera/depth/image_rect_raw --verbose
ros2 topic info /camera/color/image_raw --verbose
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

metrics 是 1 Hz 诊断消息，内部 totals 的时间差用于计算融合与更新频率；不要把 metrics 自身频率视为 ESDF 更新频率。topic hz 反映该订阅器实际接收，不等同于 GPU 计算或 RViz 渲染帧率。

分别保存地图与 Mesh：

```bash
mkdir -p /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual_reliable
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath '{file_path: "/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual_reliable/mesh.ply"}'
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath '{file_path: "/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/manual_reliable/map.blox"}'
```

外部相机/旧 bag 不保证提供 Reliable；此时先检查发布端 QoS。对 Best Effort 发布端使用 input_qos:=sensor_data，或在确认适用后调整发布/回放策略。只把订阅改成 Reliable 会造成不匹配，不能恢复旧 bag 已丢失的图像。本次未修改真实相机驱动或系统网络参数。

## 9. 验证与可追溯文件

编译通过；现有 stride、输入话题选择、消息构造相关 10 个测试通过。测试运行时禁用外部 pytest 插件自动加载，避免本机 typeguard 插件环境错误；未安装或升级依赖。初次加载新增插件参数时发现基类严格解析器拒绝新项，已补齐并重新编译；该失败尝试保留在 reliable/，不计入结果。

原始目录：/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/input_delivery_fix_20261110。静止前后输入位于 baseline/ 与 reliable_static/；运动输入位于 moving/input_bag/；两轮重建位于 stride2/、stride3/，含参数、日志、metrics_events.json、received_clouds.json、最后接收的 ESDF CDR、mesh.ply、map.blox。汇总为 results.json。

所有本轮启动的仿真与 mapper 在测试完成后关闭。
