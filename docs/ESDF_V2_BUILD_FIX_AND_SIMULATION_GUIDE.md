# sparse-v2 构建错误修复与仿真运行操作说明

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

日期：2026-10-27。修改对象：`/home/czz/perception_eye_in_hand`。本次已验证生产目标和可选几何工具的**编译、链接、安装**，未执行几何测试，未启动 mapper、Gazebo 或 RViz，未进行新的性能测试。

## 1. 本次报错的因果关系

```text
native_geometry_test 链接失败
    → 整个原生包构建/安装没有完成
    → 启动检查拒绝旧安装程序
    → 感知应用未启动
    → /esdf_3d、/stats、/camera_path、/mesh 等话题均不存在
```

因此只有 `/parameter_events` 和 `/rosout` 时，首先要解决构建/启动，不能靠反复 source 或 topic echo 产生感知数据。source 仅准备环境，不启动节点。

`selected interface "lo" is not multicast-capable` 是本机 localhost 通信配置的提示；此次阻止启动的错误是链接失败。它不能解释为 ESDF 处理过慢或 voxel_size 错误。

终端出现单独 `>` 提示时，通常是 Bash 正在等待未闭合引号/反引号。用户日志中的 Markdown 三反引号被复制进了终端；先 Ctrl+C 返回正常提示符，之后只复制代码块内部命令，不复制围栏字符。

## 2. 链接错误的具体原因与修复

错误发生在新回归工具的普通 C++ 源码中直接调用 `EsdfLayer::allocateBlockAtIndexAsync()`。它实例化的 GPU 哈希插入方法包含 `thrust::pair` 参数。

实测读取已安装库的符号，导出的命名空间为：

```text
thrust::THRUST_200500_700_750_800_860_890_NS::pair
```

用户报错中的 C++ 对象文件则请求：

```text
thrust::THRUST_200500___CUDA_ARCH_LIST___NS::pair
```

二者是不同 C++ 符号，链接器不能互相替代。这是主机侧模板实例化暴露了 CUDA/Thrust 的架构相关 ABI，不是少 source 一个 ROS 包，也不是少链接一个普通系统库。仅把设备架构改成 89、调整 voxel_size 或重新安装 Python 包都不能修复这条不匹配的调用。

已修改 [native_geometry_test.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_geometry_test.cpp)：移除这条主机侧直接分配 GPU block 的调用；从官方 Mapper 已生成的真实分配块读取 unknown 标志，保留未知过滤、原生距离对照、空地图、空 ROI 和内部体素筛选的检查。没有伪造 Thrust 命名空间，也没有改动 workspaces 库。

同时修改：

| 文件 | 修复 |
| --- | --- |
| [CMakeLists.txt](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/CMakeLists.txt) | 生产 mapper 与可选几何工具分开；`UR7E_BUILD_NATIVE_GEOMETRY_TEST` 默认 OFF |
| [build_ur7e_native.sh](/home/czz/perception_eye_in_hand/build_ur7e_native.sh) | 默认构建生产目标；`ESDF_BUILD_GEOMETRY_TEST=ON` 时额外编译工具，但不执行它；成功构建安装后记录完成标记 |
| [run_ur7e_native.sh](/home/czz/perception_eye_in_hand/run_ur7e_native.sh) | 以成功构建标记和对应目标源文件检查过期状态，避免可执行文件无需重链接时一直误报 stale |

旧检查把所有源文件（包括测试源码和 CMake 文件）与 mapper 二进制 mtime 比较。即使成功构建了新的测试工具，mapper 本身未改动、不需重新链接，它的时间仍可能较旧。现在仅在整个构建/安装返回成功后更新时间标记；失败不会更新标记。生产检查也不再因为测试源码单独改变就阻止生产 mapper。

不要手动 touch 可执行文件或完成标记来绕过检查。重新构建是正常恢复路径。

## 3. 已验证的范围

- `ESDF_BUILD_GEOMETRY_TEST=ON ./build_ur7e_native.sh`：mapper 和修复后的几何工具均成功编译、链接、安装。
- `./build_ur7e_native.sh`：默认生产构建成功，几何工具选项 OFF。
- `./run_ur7e_native.sh --check-build`：生产构建检查通过；该参数只做文件检查，不启动 ROS 或 GPU 程序。
- ON 构建后可选工具的文件检查也通过；最后一次默认 OFF 构建会取消其完成标记，后续要用该工具须显式 ON 构建。
- 没有执行测试可执行文件，也没有运行新版感知或仿真。因此这里只证明构建修复，不宣称 sparse-v2 的功能/频率/延迟验收已通过。

证据：[默认构建日志](/home/czz/perception_eye_in_hand/artifacts/esdf_v2_build_fix_20261027/build_production.log)、[含几何工具构建日志](/home/czz/perception_eye_in_hand/artifacts/esdf_v2_build_fix_20261027/build_with_geometry.log)、[官方库对应符号](/home/czz/perception_eye_in_hand/artifacts/esdf_v2_build_fix_20261027/official_thrust_symbol.txt)。

## 4. 第一步：确认构建成功

在项目目录运行：

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_native.sh && ./build_ur7e_d435_scene.sh
```

两次均应显示 `Summary: 1 package finished`。**任意一次出现 Failed，先停止后续启动/话题查询**，保留该次完整错误日志排查；不要把后续的 Unknown topic 当成新的地图算法问题。

然后可单独检查：

```bash
./run_ur7e_native.sh --check-build
```

无报错且返回命令提示符即检查通过。该检查不会启动 mapper；完成它后看不到感知话题是正常的。

## 5. 第二步：终端 A 启动完整仿真感知应用

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
./run_ur7e_perception.sh gazebo_gui:=true
```

保持该终端运行。它统一启动：

1. 本项目 Gazebo 机器人、相机、关节控制器和 TF。
2. 公共 640×360 深度/内参输入（场景原始 1280×720、stride=2）。
3. 本项目原生 TSDF / 三维 ESDF，3 cm 体素，既定 ROI。
4. RViz，默认 occupied ESDF 和相机 Path。

**不要再另外启动 scene、common_depth 或 native mapper，也不要同时运行旧 Python mapper。** 等待相机、控制器和地图初始化；若终端 A 报错退出，先处理 A 的首个错误。

端口占用时可只读检查：

```bash
ss -ltnp 'sport = :11457'
```

已有同域场景且没有重复 mapper/公共输入时，改用 `./run_ur7e_perception.sh start_scene:=false`。该模式只启动输入/感知/RViz，不新建场景，也不会补开已有场景的 Gazebo GUI。或者先在原场景终端 Ctrl+C，等待端口释放再完整启动。不要使用全局 pkill。

## 6. 第三步：终端 B 确认节点与话题，再测频率

新终端准备与 A 相同的 ROS 域和 DDS：

```bash
cd /home/czz/perception_eye_in_hand
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
ros2 node list
ros2 topic list -t
```

应至少能找到 `/ur7e_nvblox`、`/ur7e_common_depth` 和机器人状态发布节点，以及下列核心话题。若仍只有 rosout/parameter_events，返回终端 A 检查是否真正启动、是否已退出，核对两终端 domain/RMW；先不要继续 echo。

| Topic | 内容 / 类型 |
| --- | --- |
| `/esdf_input/depth`、`/esdf_input/camera_info` | 公共 Image / CameraInfo |
| `/ur7e_nvblox/esdf_3d` | PointCloud2，已观测且 is_inside 的体素；不补未知点 |
| `/ur7e_nvblox/esdf_observed` | PointCloud2，已观测近表面距离带 |
| `/ur7e_nvblox/esdf_distance` | PointCloud2，所有已观测有效距离；包含自由空间 |
| `/ur7e_nvblox/mesh` | nvblox_msgs/Mesh，官方增量表面协议 |
| `/ur7e_nvblox/mesh_markers` | MarkerArray，标准 RViz 表面显示 |
| `/ur7e_nvblox/camera_path`、`/ur7e_nvblox/slam_path` | nav_msgs/Path，机械臂图像时刻 TF 轨迹和别名 |
| `/ur7e_nvblox/camera_pose` | PoseStamped，相机位姿 |
| `/ur7e_nvblox/stats`、`/ur7e_nvblox/map_epoch` | 状态/计数和地图代际 |

确认节点存在后执行：

```bash
ros2 topic hz /esdf_input/depth --wall-time
```

Ctrl+C 后分别运行以下命令，不要在一个正在阻塞的命令后粘贴其他操作：

```bash
ros2 topic hz /ur7e_nvblox/esdf_3d --wall-time
```

```bash
ros2 topic echo /ur7e_nvblox/stats std_msgs/msg/String --once
```

```bash
ros2 topic echo /ur7e_nvblox/camera_path nav_msgs/msg/Path --once --qos-durability transient_local --qos-reliability reliable
```

```bash
ros2 topic echo /ur7e_nvblox/slam_path nav_msgs/msg/Path --once --qos-durability transient_local --qos-reliability reliable
```

stats 应报告 `interface_revision: 2` 和 `output_mode: allocated_blocks_sparse_v2`，并包含实际 fusion/ESDF/export 计数、分配块和输出点数。配置目标 30 Hz 不等于实测 30 Hz；RViz FPS 也不是地图更新频率。

可用项目监控同时看各阶段：

```bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox:${PYTHONPATH:-}
/usr/bin/python3 -B -m ur7e_nvblox.monitor
```

监控本身是 observed 点云消费者，会触发对应按需导出。`queued` 是 native 处理侧首见队列项，不是原始 DDS 收包总数。

## 7. RViz 中如何查看 Mesh、ESDF 和路径

Fixed Frame 为 `base_link`。

- **ESDF Occupied**：`/ur7e_nvblox/esdf_3d`，Reliable；只看已观测内部体素。点数不应固定为整个空 ROI 的单元数。
- **Surface Mesh (enable to view)**：勾选后查看 `/ur7e_nvblox/mesh_markers`，类型 MarkerArray，Reliable/Volatile。默认关闭以避免额外 Mesh 负载。
- **Camera Path (robot TF)**：`/ur7e_nvblox/camera_path`，Reliable/TransientLocal；正常初次只有短路径，移动相机后才有轨迹。
- **ESDF Distance Band**：`/ur7e_nvblox/esdf_observed`，BestEffort；可能看到有厚度的距离带，这是有效自由/内部距离，不是表面 Mesh。
- **All Known ESDF Distances**：`/ur7e_nvblox/esdf_distance`，诊断用；已观测自由空间较多时点也多，但不应有未知占位。

不要把官方自定义 Mesh 插件当作本机默认选项；此前缺 GLSL150 资源，新默认提供标准 Marker 桥接。真正的视觉 SLAM 路径 `/visual_slam/tracking/slam_path` 不由本项目生成；这里的 slam_path 是 robot TF 路径别名，没有视觉定位或回环优化。

查看原生 Mesh 元信息时再加载对应自定义消息环境：

```bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ros2 topic info /ur7e_nvblox/mesh --verbose
```

Mesh 按订阅需求工作：topic 存在但无消费者/无新网格时不保证持续发送。启用标准 Mesh 后，可手动请求一次完整重同步：

```bash
ros2 service call /ur7e_nvblox/republish_mesh std_srvs/srv/Trigger '{}'
```

## 8. 更新正常但画面为空时

先确认公共深度、图像时刻 TF、融合计数都在更新，再看 ROI 内是否确实有被观测表面。原始初始姿态不保证相机朝向既定工作区，本项目也不会自动移动机械臂来制造画面。

可以手动控制仿真机械臂朝向目标区域；已使用过的观察关节角（rad）为 `[0, -1.57, 1.1, -0.90879335, -3.21580312, -0.69520875]`，顺序为 shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3。它是仿真参考姿态，不是实机运动指令。

需要可复现的短时功能检查时，可在 B 的环境中执行下面**可选步骤**。脚本会平滑移动仿真机械臂到观察姿态并小幅摆腕；只允许测试域 127，不要对物理机器人执行：

```bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox:/home/czz/perception_eye_in_hand/src/realtime_nvblox:${PYTHONPATH:-}
mkdir -p /home/czz/perception_eye_in_hand/artifacts/my_sparse_v2_check
/usr/bin/python3 -B /home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py \
  --external --common-input --mode native --voxel-size 0.03 \
  --seconds 5 --exercise-simulation \
  --output /home/czz/perception_eye_in_hand/artifacts/my_sparse_v2_check/result.json
```

这是用户后续可选择执行的检查，本轮没有执行。GUI 开启下的短时结果不是严格性能基线。当前 RViz/监控存在额外点云消费者，勿随意加 `--lifecycle`，否则“无消费者”断言会因为其他订阅者而失败。

## 9. 可选几何工具：先编译，再按需执行

日常仿真不需要该工具。需要它时：

```bash
cd /home/czz/perception_eye_in_hand
ESDF_BUILD_GEOMETRY_TEST=ON ./build_ur7e_native.sh
```

以上只编译/安装，不执行测试。用户之后明确需要验证 CUDA 几何时，才另行执行：

```bash
ESDF_NATIVE_EXECUTABLE=native_geometry_test ./run_ur7e_native.sh
```

再次使用默认 OFF 构建后，可选工具的成功构建标记会被取消，即使目录中留有旧工具文件也不会被当作刚构建的版本。不要用这个可选工具的环境变量启动统一感知应用。

## 10. 退出与排错顺序

先退出诊断/监控命令，再在终端 A Ctrl+C，等待 launch 子进程退出。关闭窗口不等于停止 gzserver。遇到异常按以下顺序定位：

| 现象 | 优先检查 |
| --- | --- |
| undefined reference / Failed package | 构建日志，尚未进入感知运行阶段 |
| stale / 无成功构建标记 | 使用根目录 build_ur7e_native.sh 完成构建；不要手工改文件时间 |
| 只看到 rosout/parameter_events | 应用未启动、已退出或终端的 domain/RMW 不同 |
| 有节点和话题，但没有输入 | 相机/公共转换、CameraInfo、图像 frame 和时间 |
| 更新计数增加但无 occupied 点 | 相机视向、ROI、观测表面；不能用未知占位点填画面 |
| Mesh 无消息 | 启用消费者，再检查实际 TSDF 表面与层发布；必要时 republish_mesh |
| Path 没有长线 | 是否有成功融合和相机运动；路径来源是机械臂 TF，不是自动 SLAM |
| 出现单独 > 提示符 | Bash 语法尚未闭合，Ctrl+C，重新只粘贴命令正文 |

感知结构与接口的详细说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧版截图和约 2.4 ms 的数据是历史结果，本次构建成功不能代替新版运行验收。
