# Gazebo UR7e 感知运行与验证

本入口使用 Gazebo 深度、CameraInfo 和图像曝光时刻的 TF，复用新版 `NvbloxBackend` 完成 TSDF/ESDF。不会启动 RealSense SDK、cuVSLAM 或真机 runtime；原 `ros_mapper` 保留。实测手眼外参不变。

## 构建

在项目目录执行（已实际构建成功，不必删除 build/install）：

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realtime_nvblox ur7e_d435_gazebo
source install/local_setup.bash
ros2 pkg executables realtime_nvblox
```

可执行列表包含 `gazebo_ros_mapper` 和 `ros_mapper`。

## 三个终端运行

三个终端使用相同 ROS_DOMAIN_ID。下面使用独立域 78 和端口 11478，避免混入已运行的其他场景或真机。若该端口已有场景，复用它或先在其启动终端 Ctrl+C，不要重复启动服务器。

终端一：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
export GAZEBO_MASTER_URI=http://127.0.0.1:11478
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

终端二：

```bash
cd ~/perception_eye_in_hand
export ROS_DOMAIN_ID=78
./run_gazebo_mapping.sh run_rviz:=true
```

终端三，设置观察姿态（只用于本工程的 Gazebo 场景）：

```bash
cd ~/perception_eye_in_hand
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=78
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --prepare-view
```

**观察姿态是必要步骤。** 原场景初始姿态的相机朝上、朝外，看不到默认 ESDF 网格内的近距离障碍物。脚本用 4 秒平滑移动腕部，使镜头朝向墙和地面，然后清图。只改变仿真关节，不移动相机模型、不改外参、不改变 TF 定义。相机位于约 `[0.455, 0.041, 0.682] m`，光轴朝下。

此时 RViz 的 Fixed Frame 为 `base_link`，默认显示 RobotModel、TF、ESDF observed。颜色表达距离，unknown 不显示在 observed 中。可勾选 Full ESDF Grid 查看全部 52111 个网格点（包括灰色 unknown）。背面和遮挡区域仍可能未知，不能作为已确认安全空间。

## 操作验证

只读检查 20 秒频率与严格 TF 对齐：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py
```

完整仿真操作检查：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py --exercise-simulation
```

它会设置观察姿态、测量静止窗口、让肩部绕初始角度做 ±0.08 rad 的缓慢运动、恢复观察姿态、检查静态表面距离变化、保存地图、调用 reset_map 并验证新地图。默认不会调用 Gazebo 全局 reset。结束后保留观察姿态。

球体被墙遮挡时，切换第二个观察姿态：

```bash
python3 src/realtime_nvblox/examples/verify_gazebo_mapping.py \
  --exercise-simulation --view sphere \
  --output artifacts/gazebo_perception/sphere_verification.json
```

只切换球体观察姿态可以使用 `--prepare-view --view sphere`。脚本默认报告输出至 `artifacts/gazebo_perception/verification.json`，`passed` 为检查结果。运动功能必须显式添加上述选项，脚本还会检查 Gazebo 服务是否存在。

手动查看输出、清图和保存：

```bash
ros2 topic echo /realtime_nvblox/stats --once
ros2 topic hz /realtime_nvblox/esdf_observed --qos-reliability best_effort
ros2 service call /realtime_nvblox/reset_map std_srvs/srv/Trigger '{}'
ros2 service call /realtime_nvblox/save_map std_srvs/srv/Trigger '{}'
```

默认保存至 `~/nvblox_output/realtime_nvblox_gazebo/base_link_map.nvblox`。验收运行通过自定义 YAML 将地图保存在项目 `artifacts/gazebo_perception/` 下。该文件是实际 nvblox 地图，不是占位文件。

另开**没有 Gazebo**的 ROS 域验证缺失 TF、非法/过期/未来图像，以及连续两次时钟回退：

```bash
ROS_DOMAIN_ID=79 python3 src/realtime_nvblox/examples/verify_gazebo_rejection.py
```

脚本自行启动测试 mapper，并在结束时停止该子进程。其余域的仿真不受影响。

## 时间、坐标与线程

- 唯一映射位姿：`base_link <- camera_depth_optical_frame @ Image.header.stamp`。没有 latest/nearest TF 回退，没有 FK 重算，没有二次手眼变换。
- 16UC1 按毫米转换，32FC1 按米读取，处理大小端、行填充和无效深度。P 内参优先，K 回退；stride 同时缩放尺寸和内参。
- ROS 回调只接收数据；工作线程执行 TF 查询、CUDA 融合和 ESDF 查询；独立线程发布 latest-only 结果。
- 深度输入和发布结果各保留最新一项。统计中的 overwritten 是主动丢弃旧输入，不是无限积压。
- 图像新鲜度用 `/clock`。时钟回退/清图后 epoch 增加，清除待处理输入/输出并重建 CUDA 地图；旧 epoch 的正在执行结果不能发布。静态 TF 缓存恢复，动态 TF 必须重新收到。
- `receive_to_result_ms` 从 mapper 收到图像到发布计算，包含 TF/时钟等待和 GPU 查询；不是从相机曝光开始的总延迟。分位数来自最近最多 512 次采样。`mapping_with_gpu_readback` 包含 GPU 结果读回；`integrate_submit` 不能单独当作 GPU 完成时间。
- `esdf_hz: 10` 是仿真时间更新上限，不保证任意渲染负载下墙钟 10 Hz。完整网格 RELIABLE 深度 1；observed BEST_EFFORT 深度 1。

## 已发现的边界

1. 本机 Gazebo Classic 全局 `/reset_simulation` 后，深度和 `/clock` 继续发布，但 `/joint_states` 和动态 `/tf` 停止。mapper 会清图并增加 `tf_missing`，不会拿旧 TF 融合。日常清图使用 `reset_map`；已经触发此问题时，停止并重启仿真。`--reset-simulation` 可复现该上游控制链限制，默认验收不执行它。独立输入测试已验证：新 epoch TF 正常供应时，mapper 两次回退后均能恢复 CUDA 建图。
2. 标准 Image 不携带仿真 epoch 标识；若旧 epoch DDS 图像的数值时间戳恰好落入新 epoch 的合法窗口，单凭 Image/Clock 无法彻底辨别。当前实现会淘汰已经入队/正在处理的旧 epoch 数据以及时间窗口外的数据。对跨重启严格隔离的采集任务应重启发布者/订阅者或使用独立 ROS 域。
3. 地图是静态场景的累积 TSDF/ESDF；当前工作未实现动态机器人自体过滤和运动物体清除，也未接入碰撞规划闭环。相机可见的机器人或支架可能被融合。
4. 真实硬件路线做了源码一致性和单元测试回归，没有连接真机进行新的采集验证。

实测数值、文件改动说明、节点与话题记录见项目 `artifacts/gazebo_perception/REPORT.md`。
