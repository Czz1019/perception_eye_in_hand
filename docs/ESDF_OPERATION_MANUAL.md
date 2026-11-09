# ESDF 完整操作、复测与原生 ROS 基线手册

> 2026-10-27 实施更新：原生 C++ 模块已接入并完成同精度仿真对照。本文保留早期设计/操作历史；最新代码状态、实测结果和运行方式以 [接入报告](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md) 和 [完整操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md) 为准。

日期：2026-10-27。适用于 `/home/czz/perception_eye_in_hand` 和本机 `/home/czz/workspaces/isaac_ros-dev`。

本手册分为：A 当前已实现 Python 版的完整操作；B workspaces 官方 ROS C++ 的独立基线检查；C 后续接入验收。**B 的命令经源码核对但未在本轮实际启动验证；仓库内新的 C++ 适配后端尚不存在。** 本轮只新增说明文档，不执行下列安装或运动操作。

设计和状态依据：[实际应用实时性与原生模块设计](ESDF_NATIVE_REALTIME_DESIGN.md)。历史数据：[2026-10-26 实测报告](ESDF_ROS_ALIGNMENT_IMPLEMENTATION.md)。

## 0. 最短操作路线

1. 阅读第 1 节，确认 GPU、Python、ROS 和空间。
2. 第 2 节构建当前两个感知包；仿真包缺失时单独构建。
3. 第 3 节创建统一终端环境，启动场景、mapper/RViz 和监测器。
4. 第 4–6 节检查图像、位姿、距离场和真实更新频率。
5. 第 7 节运行隔离 Gazebo/CUDA 验证，保存自己的新结果目录。
6. 第 8 节按实际精度运行同条件矩阵，不能把调粗体素当作实现优化。
7. 第 9 节先独立建立官方 ROS 基线；它与当前 mapper 顺序运行，不能同时混图。
8. 完成后按第 11 节停止自己启动的进程。

## 1. 环境与依赖检查

### 1.1 空间、版本和驱动

```bash
cd /home/czz/perception_eye_in_hand
df -h / /home/czz /tmp
df -i / /home/czz /tmp
nvidia-smi
/usr/bin/python3 --version
source /opt/ros/humble/setup.bash
ros2 --help
command -v colcon
command -v nvcc
```

2026-10-27 本轮只读检查时，根分区约 90 GB、home 约 61 GB 可用，inode 也充足；这是当时的空间状态，不保证后续构建/GPU运行成功。此前“磁盘满”的工具初始化错误当前未复现。

`nvidia-smi` 失败先检查驱动/设备访问；不要把它误判为缺少 Python 包。`nvcc` 主要用于源码构建；现有 wheel 跑 CUDA 与能编译 CUDA 扩展是两件事。

### 1.2 Python 路径检查

从没有混入其他 conda/ROS overlay 的新终端开始：

```bash
cd /home/czz/perception_eye_in_hand
source /opt/ros/humble/setup.bash
unset PYTHONNOUSERSITE
export PYTHONPATH="$PWD/src/ur7e_nvblox:$PWD/src/realtime_nvblox:${PYTHONPATH:-}"
/usr/bin/python3 -B - <<'PY'
import sys
import importlib.util
import torch
import rclpy
from realtime_nvblox.nvblox_backend import preload_nvblox_runtime
print('python:', sys.executable)
print('torch:', torch.__version__, 'CUDA:', torch.cuda.is_available())
print('nvblox package:', importlib.util.find_spec('nvblox_torch').origin)
print('native library:', preload_nvblox_runtime())
from nvblox_torch.mapper import Mapper
print('Mapper import: OK')
PY
```

应看到明确的解释器路径、CUDA=True、当前 nvblox_torch 位置及可导入 Mapper。这里只检查加载，不等于完成融合测试。

本机历史实测使用系统 Python 3.10、PyTorch 2.9.1+cu128、用户目录的 nvBlox wheel。启动脚本选择顺序为 `ESDF_PYTHON` → 存在的 whole_control → `/usr/bin/python3`。需要其他环境时，先用对应解释器完成上面的检查，再运行：

```bash
ESDF_PYTHON=/absolute/path/to/python ./run_ur7e_esdf.sh run_rviz:=false
```

这是解释器路径模板，不要把 `/absolute/path/to/python` 原样执行。脚本不负责安装 Torch/CUDA。

### 1.3 依赖缺失时如何处理

先查缺失项，不做整体升级：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix rclpy
ros2 pkg prefix gazebo_ros
ros2 pkg prefix tf2_ros
ros2 pkg prefix rviz2
ros2 pkg prefix rmw_cyclonedds_cpp
rosdep check --from-paths src/realtime_nvblox src/ur7e_nvblox src/ur7e_d435_gazebo --ignore-src --rosdistro humble
```

若本机已配置 ROS apt 源，仅缺 Cyclone DDS，可选择补装：

```bash
sudo apt-get update
sudo apt-get install ros-humble-rmw-cyclonedds-cpp
```

若 rosdep 已初始化，且其检查显示缺少其他系统依赖，可根据实际输出安装：

```bash
rosdep install --from-paths src/realtime_nvblox src/ur7e_nvblox src/ur7e_d435_gazebo \
  --ignore-src --rosdistro humble -r -y
```

这些命令会安装系统包，本文没有执行。不要为“提速”盲目升级驱动、Torch、CUDA 或整套 ROS。nvblox_torch 必须选择与解释器、Torch ABI、CUDA 和系统匹配的构建；手册不提供未经匹配验证的通用 pip 升级命令。官方 ROS 的 NITROS/GXF 依赖也不能由这个 wheel 替代。

## 2. 构建当前仓库可运行版本

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_esdf.sh
```

成功标志是 `realtime_nvblox` 与 `ur7e_nvblox` 均 `Finished`，不是仅看到启动日志。该脚本限定两个源码路径，避免 `realtime_nvblox_2` 声明同名包的问题；它仅处理待重建两包的失效安装链接和非法旧索引。

若仿真包尚未安装：

```bash
./build_ur7e_d435_scene.sh
```

该场景构建脚本未限定 base-paths；如果 colcon 报同名包发现冲突，可在干净终端中使用明确路径：

```bash
cd /home/czz/perception_eye_in_hand
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --base-paths src/ur7e_d435_gazebo \
  --packages-select ur7e_d435_gazebo --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

普通安装和 symlink 安装切换、旧用户绝对链接可能造成安装错误。不要直接删除整个 install/build；先定位具体包和错误路径，必要时备份后只重建相关包。不要修改手眼文件来绕过挂载校验。

## 3. 三终端启动仿真、实时画面与监测

### 3.1 创建一份统一环境文件

构建完成后执行一次：

```bash
mkdir -p /tmp/ur7e_esdf_run
cat > /tmp/ur7e_esdf_run/env.sh <<'SH'
export ESDF_REPO=/home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
export PYTHONDONTWRITEBYTECODE=1
cd "$ESDF_REPO"
source /opt/ros/humble/setup.bash
unset COLCON_CURRENT_PREFIX
source "$ESDF_REPO/install/realtime_nvblox/share/realtime_nvblox/package.bash"
unset COLCON_CURRENT_PREFIX
source "$ESDF_REPO/install/ur7e_nvblox/share/ur7e_nvblox/package.bash"
SH
```

三个终端都 `source` 这份文件。127 与 11457 是历史测试使用的隔离域/端口，**不代表现在仍有场景运行**。如果端口已用，先确定进程归属；也可选择其他空闲端口，但所有仿真终端必须一致。自动运动验收脚本限制 ROS 域 127。

Cyclone 是本机之前得到较好图像接收效果的对照环境，不是所有机器的必需选择。换 RMW 必须重启整组相关进程并保持一致，不能只改监测终端。

### 3.2 终端 1：场景

```bash
source /tmp/ur7e_esdf_run/env.sh
./run_ur7e_d435_scene.sh gazebo_gui:=true
```

只要传感器和 robot_state_publisher 运行，GUI 可改为 false；但 headless 不表示相机渲染成本为零。

正常应看到控制器激活、相机插件加载、障碍物生成。若有 master 已占用提示，不要继续启动第二套同端口服务器。已有场景满足输入和标定要求时跳过本步骤。

### 3.3 终端 2：感知和 RViz

```bash
source /tmp/ur7e_esdf_run/env.sh
./run_ur7e_esdf.sh run_rviz:=true
```

应出现 mapper 的 `Mapper ready` 日志及 RViz。首次 CUDA 初始化可能明显慢于稳态；不要把冷启动时间计入每帧稳态延迟。

RViz 设置：

| 项目 | 值 |
| --- | --- |
| Fixed Frame | `base_link` |
| PointCloud2 Topic | `/ur7e_nvblox/esdf_observed` |
| Reliability | Best Effort |
| Position Transformer | XYZ |
| Color Transformer | RGB8 |
| Decay Time | 0 |
| Pose Topic | `/ur7e_nvblox/camera_pose` |

默认是 ESDF 距离带点云，不是彩色重建 Mesh。相机没看到的区域不应被误当成“空且安全”。相机朝向与 ROI 不重叠时可以没有有效点，不要马上提高发布频率或放宽 TF 校验。

### 3.4 终端 3：真实频率

```bash
source /tmp/ur7e_esdf_run/env.sh
ros2 run ur7e_nvblox esdf_rates
```

示意输出（不是预期固定数值）：

```text
depth=... fusion=... ESDF=... export=... received=... unique_image_stamps=... Hz points=... map_version=... error=None
```

解释：`depth` 是输入，`fusion` 是积分，`ESDF` 是更新调用，`export` 是查询读回，`received` 是监测器收到的 observed 消息。`unique_image_stamps` 是当前统计窗口不同源图像的数量，不保证几何发生了变化；`map_version` 应持续前进。

监测器本身订阅 observed，因此会触发按需导出。要测试无消费者，必须同时关闭 RViz、监测器、topic hz、记录器和其他点云订阅者。

## 4. 输入和位姿排查命令

在已 source 环境的额外终端执行，长时间监测用 Ctrl+C 结束：

```bash
ros2 node list
ros2 topic info /camera/depth/image_rect_raw --verbose
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic echo /camera/depth/camera_info --once --field header
ros2 topic hz /joint_states
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

`tf2_echo` 只能快速查看链是否存在；它不能替代 mapper 按每张图像时刻做变换的验证。检查 CameraInfo 的尺寸和 frame 是否与实际深度一致。16UC1 通常是毫米，32FC1 通常是米；当前解码器按类型转换，不应再手动叠乘一次比例。

读取实际生效参数，而不是只看源码 YAML：

```bash
ros2 param get /ur7e_nvblox use_sim_time
ros2 param get /ur7e_nvblox voxel_size_m
ros2 param get /ur7e_nvblox esdf_output_mode
ros2 param get /ur7e_nvblox depth_stride
ros2 param get /ur7e_nvblox esdf_hz
ros2 param get /ur7e_nvblox publish_hz
ros2 param dump /ur7e_nvblox > /tmp/ur7e_esdf_run/effective_parameters.yaml
ros2 topic echo /ur7e_nvblox/stats --once
```

参数运行时修改目前没有实现可靠的动态重配置流程。要改变体素或采样模式，修改独立配置后重启 mapper；不要假定 `ros2 param set` 成功返回就已经重建 GPU 地图。

## 5. 固定实际精度运行与兼容接口

### 5.1 创建自己的实验配置

```bash
cp src/ur7e_nvblox/config/gazebo.yaml /tmp/ur7e_esdf_run/application.yaml
```

用编辑器设置业务实际需要的 `voxel_size_m`，例如 0.03；此值只是示例，不是新的性能捷径。保持 ROI、输入、频率及距离带明确。然后启动：

```bash
./run_ur7e_esdf.sh config:=/tmp/ur7e_esdf_run/application.yaml run_rviz:=true
```

native 下 ROI 的候选规模近似为 `体积 / voxel_size³`。相同 2×2×1.5 m 范围，3 cm 的候选约二十多万个，比 5 cm 明显增加。应在这个精度下验收，不能换成更粗尺寸后宣称满足原要求。

### 5.2 保留旧固定网格消费者

在独立 YAML 中设置：

```yaml
esdf_output_mode: dense
grid_resolution_m: 0.05
```

这两项必须放在已有 `/**: -> ros__parameters:` 层内，不是完整独立 YAML。

此时旧默认 ROI 为 `41×41×31=52,111` 点。完整话题 `/ur7e_nvblox/esdf_3d` 包含 unknown sentinel，必须在下游判断 unknown。native 模式的坐标、总数、顺序与它不同；依赖旧顺序的算法应显式使用 dense。

完整话题默认 Reliable，observed 默认 Best Effort。查看完整话题会主动引入完整读回/传输，不应用它的负载去代表仅 RViz observed 的负载。

## 6. 采集、保存、重置与记录

手动采集时使用现有的仿真关节控制端移动机械臂；本次没有新增可拖动机械臂的操作界面。RViz 的 Pose 显示不是机械臂控制器。用于复测的平滑运动注入见第 7 节，仅限隔离 Gazebo。

保存和重置：

```bash
ros2 service call /ur7e_nvblox/save_map std_srvs/srv/Trigger '{}'
ros2 service call /ur7e_nvblox/reset_map std_srvs/srv/Trigger '{}'
```

save_map 返回实际路径，默认由 `output_directory` 控制。reset_map 在下一个有效输入上重建，随后 `map_epoch` 应变化；不要用 `/reset_simulation` 代替，已有 Gazebo 全局重置后控制插件不再更新关节 TF 的历史限制。

建议每次创建新目录保存参数、版本和日志：

```bash
mkdir -p /tmp/ur7e_esdf_run/measurement
ros2 param dump /ur7e_nvblox > /tmp/ur7e_esdf_run/measurement/parameters.yaml
git rev-parse HEAD > /tmp/ur7e_esdf_run/measurement/git_head.txt
git diff -- src/ur7e_nvblox > /tmp/ur7e_esdf_run/measurement/source_changes.patch
ros2 run ur7e_nvblox esdf_rates --seconds 60 \
  | tee /tmp/ur7e_esdf_run/measurement/live_rates.txt
```

需要后续同输入回放时记录原始数据：

```bash
ros2 bag record -o /tmp/ur7e_esdf_run/input_bag \
  /camera/depth/image_rect_raw /camera/depth/camera_info \
  /tf /tf_static /clock /joint_states
```

录包会增加负载，应记录是否启用；静态 TF 的录制/回放 QoS 和起始历史必须检查。回放时不要同时保留原场景继续发布同名图像和时钟。带 `/clock` 的 bag 是否另加 `ros2 bag play --clock`，要先检查 bag 的话题，避免两个时钟源；先核对 `ros2 bag play --help`，再确定回放策略。

## 7. 自动验收：真实 CUDA 与隔离 Gazebo

### 7.1 CUDA/消息回归

```bash
source /tmp/ur7e_esdf_run/env.sh
unset PYTHONNOUSERSITE
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD/src/ur7e_nvblox:$PWD/src/realtime_nvblox:${PYTHONPATH:-}"
RUN_GPU_TESTS=1 /usr/bin/python3 -B -m pytest src/ur7e_nvblox/test -q
```

历史最终结果为 13 passed；本轮没有重跑。测试覆盖平面距离/变换、unknown、数组所有权、GPU 过滤、采样、限频抖动、时间回退与版本去重。若改用其他解释器，替换上面的 `/usr/bin/python3`，不要只改环境名。

### 7.2 Gazebo 运动与生命周期验证

保留终端 1 的场景，先关闭终端 2 的 mapper/RViz、终端 3 及其他点云消费者。脚本会自行创建一个 mapper，不能与已有 mapper 并行。

```bash
source /tmp/ur7e_esdf_run/env.sh
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/src/ur7e_nvblox:$PWD/src/realtime_nvblox:${PYTHONPATH:-}"
/usr/bin/python3 -B src/ur7e_nvblox/test/run_gazebo_validation.py \
  --mode native --voxel-size 0.03 --esdf-hz 30 --publish-hz 30 \
  --seconds 60 --exercise-simulation --lifecycle \
  --output /tmp/ur7e_esdf_run/measurement/gazebo_native_003.json
```

0.03 是示例业务精度，替换为你的实际要求。历史归档用 0.05/0.10 和约 12 秒窗口，不是已经完成上述 60 秒、3 cm 测试。

`--exercise-simulation` 会在域 127 中平滑移动到观察姿态，并小幅改变 wrist_1；只允许连接该隔离 Gazebo，禁止将该命令用于实机。`--lifecycle` 还会销毁脚本自身点云订阅，验证地图继续更新时查询为零，并重置再订阅；外部 RViz/记录器仍存在会使该检查失败。

成功输出 JSON 的 `result: PASS`、两阶段数据、TF 最大误差、无消费者查询数与 reset epoch，并保存点云 NPZ。失败也保存 JSON，应阅读 error 和 last_stats，不能仅看末尾几行频率。

### 7.3 稳定合成输入（不能替代真实场景）

```bash
source /tmp/ur7e_esdf_run/env.sh
export ROS_DOMAIN_ID=128
export PYTHONPATH="$PWD/src/ur7e_nvblox:$PWD/src/realtime_nvblox:${PYTHONPATH:-}"
/usr/bin/python3 -B src/ur7e_nvblox/test/run_stream_benchmark.py \
  --mode native --voxel-size .03 --esdf-hz 30 --observed --check-stop \
  --seconds 30 --output /tmp/ur7e_esdf_run/measurement/synthetic_native_003.json
```

只在本终端切到域 128；不要以为它仍连接域 127 的场景。测试使用 160×90 合成深度，历史 5 cm 配置约 30 Hz，不能代替 1280×720 相机输入下的现场表现。

## 8. 同条件对照的完整记录表

每个后端至少记录：

| 项目 | 必填内容 |
| --- | --- |
| 输入 | 数据/bag 标识、分辨率、深度格式、输入 Hz、stride 和内参 |
| 位姿 | base/末端/光学 frame、标定版本、图像时刻 TF 成功率 |
| 地图 | voxel_size、积分距离、截断/权重、ESDF 最大距离、ROI |
| 输出 | Mesh/切片/距离带/完整场、距离带、订阅者/记录器数量 |
| 软件 | 当前实际执行文件、源码哈希、GPU/驱动/Torch/CUDA、RMW |
| 性能 | 接收/融合/更新/导出/消息/新版本 Hz、P50/P95/P99 延迟 |
| 正确性 | 距离、unknown、pose、重置、停帧、动态残影和自身污染 |

当前 stats 提供 mean/P50/P95/max，没有原始延迟事件序列或 P99。要补齐正式验收的 P99，需要另行记录原始事件耗时；不能对多次 P95 再求分位数来冒充 P99。它是后续测量工作，不是本手册已经完成的新计时实现。

当前 Python 的 `depth_stride=2` 将 1280×720 降到 640×360。官方 ROS 基线若直接吃 1280×720，就不能称为相同融合输入。可先让当前 Python 版也设 stride=1，与官方原始输入对比；或为双方提供同一份预处理后的深度及同步缩放的 CameraInfo。不能只给一方降采样。

复用同样的体素与范围后，先比较“只更新地图”，再逐项打开相同显示/算法消费者。减少 unknown 显示或缩小距离带应单列为输出负载变化，不当作核心 ESDF 计算提速。

## 9. workspaces 官方 ROS C++ 独立基线操作

### 9.1 状态与依赖检查

本节使用现有参考工作区，不是仓库里已完成的新后端。新开终端，避免 source 当前仓库的 Python 环境或用户 CUDA wheel 动态库目录：

```bash
source /opt/ros/humble/setup.bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
ros2 pkg prefix nvblox_ros
ros2 pkg prefix nvblox_msgs
ros2 pkg prefix nvblox_rviz_plugin
ros2 pkg prefix isaac_ros_managed_nitros
ros2 pkg prefix isaac_ros_gxf
ros2 pkg executables nvblox_ros
ldd /home/czz/workspaces/isaac_ros-dev/install/nvblox_ros/lib/nvblox_ros/nvblox_node
```

任何 `not found`、失效软链接或 NITROS/GXF 包缺失，都要先处理，不能认定“文件存在就可运行”。优先复用该工作区匹配的依赖。必要时先检查源码构建依赖：

```bash
cd /home/czz/workspaces/isaac_ros-dev
rosdep check --from-paths src/isaac_ros_nvblox --ignore-src --rosdistro humble
```

只有依赖齐备且需要重建时，才使用下面的原生架构构建模板；本轮未执行，也不保证当前缺失的包能靠此命令自动补全：

```bash
colcon build --packages-up-to nvblox_ros nvblox_rviz_plugin \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_NATIVE_CUDA_ARCHITECTURE=1
```

NITROS/GXF 需要与这份 Isaac ROS 版本匹配。不要用新主分支依赖混编旧二进制。若最终决定使用小型 C++ 核心适配而非完整 ROS 节点，应另建受控安装前缀；当前尚无该适配包的可执行命令。

### 9.2 创建独立配置，保留原参考文件

先关闭当前 Python mapper；场景可继续运行。将官方配置复制到本次运行目录，并覆盖必要值：

```bash
mkdir -p /tmp/ur7e_esdf_run
/usr/bin/python3 - <<'PY'
from pathlib import Path
import yaml
src=Path('/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml')
doc=yaml.safe_load(src.read_text())
p=doc['/**']['ros__parameters']
p.update(use_sim_time=True, global_frame='base_link', use_tf_transforms=True,
         use_topic_transforms=False, num_cameras=1, esdf_mode='3d', use_depth=True,
         use_color=False, use_lidar=False, voxel_size=0.03, input_qos='SENSOR_DATA',
         integrate_depth_rate_hz=30.0, update_esdf_rate_hz=30.0,
         publish_layer_rate_hz=15.0, map_clearing_frame_id='base_link',
         map_clearing_radius_m=-1.0, decay_tsdf_rate_hz=0.0,
         publish_esdf_distance_slice=False, print_rates_to_console=True,
         print_timings_to_console=True, print_statistics_on_console_period_ms=5000)
p['static_mapper']['projective_integrator_max_integration_distance_m']=4.0
Path('/tmp/ur7e_esdf_run/official_baseline.yaml').write_text(yaml.safe_dump(doc,sort_keys=False))
PY
```

这里 3 cm 是对照示例，应与当前 Python 实验精度一致；原生参数名是 `voxel_size`，不是 Python 的 `voxel_size_m`。ESDF 30、层发布 15 分别是更新与显示目标，不能合并解释。关闭 decay/clearing 是为了固定地图比较；动态应用不能不加评估就沿用永久静态积累。

这份配置仍继承了其他官方参数，例如 ESDF 最大距离、权重、截断和层显示裁剪；必须把它们与 Python 实际后端参数核对后，才能宣称严格同条件。只覆盖上面几个值不足以完成精确算法对照。

### 9.3 连接本仓库相机与机械臂 TF

```bash
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /tmp/ur7e_esdf_run/official_baseline.yaml \
  -r camera_0/depth/image:=/camera/depth/image_rect_raw \
  -r camera_0/depth/camera_info:=/camera/depth/camera_info
```

话题原名 `camera_0/depth/image` 来自本地 `nvblox_node.hpp`；图像格式转换还需要实际 NITROS 运行验证。本节未执行，不宣称这些命令已在本机成功运行。

官方 Transformer 使用图像 frame 和时刻查 TF，所以不要额外发布静态 `odom -> camera_link` 来“补齐”眼在手链。检查：

```bash
ros2 node info /nvblox_node
ros2 param get /nvblox_node global_frame
ros2 param get /nvblox_node voxel_size
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 param dump /nvblox_node > /tmp/ur7e_esdf_run/measurement/official_effective_parameters.yaml
ros2 service list -t
```

### 9.4 原生显示与三维距离查询分别验证

官方 RViz 插件选择 `/nvblox_node/mesh` 的 NvbloxMesh 显示，并将 Fixed Frame 改为 base_link；也可单独选择 TSDF layer。必须先 source 官方工作区，插件才可被 RViz 发现。3D 配置下不能期待二维 `static_esdf_pointcloud` 自动输出。

先做一次有界距离查询，避免一开始就连续导出全场：

```bash
ros2 service call /nvblox_node/get_esdf_and_gradient nvblox_msgs/srv/EsdfAndGradients \
  "{update_esdf: false, visualize_esdf: false, use_aabb: true, frame_id: base_link, aabb_min_m: {x: -1.0, y: -1.0, z: -0.25}, aabb_size_m: {x: 2.0, y: 2.0, z: 1.5}}"
```

响应可能很长；它应有 success、voxel_size_m、origin_m 和数组维度。返回数组也包含 unknown，且会分配完整 AABB 数据。当前本地转换代码不应仅凭服务名就认为一定有梯度通道。

配套 `live_esdf_query` 已安装且可执行时，可再启动有界显示客户端：

```bash
ros2 pkg executables live_esdf_query
ros2 run live_esdf_query global_esdf_query_node --ros-args \
  -p use_sim_time:=true -p query_frame:=base_link \
  -p aabb_min_x:=-1.0 -p aabb_min_y:=-1.0 -p aabb_min_z:=-0.25 \
  -p aabb_size_x:=2.0 -p aabb_size_y:=2.0 -p aabb_size_z:=1.5 \
  -p rate_hz:=5.0 -p stride:=1 -p max_distance:=0.15 \
  -p update_esdf:=false -p visualize_esdf:=false
```

先 5 Hz 只用于检查功能，不是最终实时验收。之后按业务需求与对照方案逐档提高查询频率并记录开销。RViz PointCloud2 选择 `/global_esdf_cloud`，Reliable；Mesh 和距离点云是两个输出，应分别记录是否开启。

不要直接照抄旧 launch 中 6×6×2.5 m、30 Hz 的全域查询来代表机械臂局部查询。客户端 stride 只减少显示遍历/点数，不减少服务端 dense 读回与响应大小。

## 10. 实机接入准备（尚未实机验收）

本节只配置感知，不提供或发送机器人运动命令。机器人驱动、相机驱动和真实手眼标定应先按现有已验证的系统启动。

1. 新终端使用实际机器人系统的 ROS_DOMAIN_ID；跨机器通信时不要沿用 `ROS_LOCALHOST_ONLY=1`。
2. 检查真实 `/joint_states`、TF、深度和 CameraInfo。根据实际话题/frame 建配置，不假定它们与 Gazebo 同名。
3. 确认真实手眼外参已进入 TF 树；不要把仿真的安装参数直接替换到实机。
4. 时钟统一；通常 `use_sim_time=false`，相机曝光时间和机器人 TF 必须可对应。
5. 深度未对齐 RGB 时用深度内参/深度 optical frame；对齐到 RGB 后需要对应 RGB optical frame 和内参。
6. 先静止采集，检查平面距离和 TF，再由操作人员手动慢速移动；记录图像时刻失败率、地图年龄、自身污染与拖影。

当前 mapping.launch.py 给内置 RViz 固定传入 use_sim_time=true；实机运行时应先 `run_rviz:=false`，再独立启动 RViz：

```bash
rviz2 -d /home/czz/perception_eye_in_hand/src/ur7e_nvblox/rviz/esdf.rviz \
  --ros-args -p use_sim_time:=false
```

实机配置应另建 YAML，设置真实话题和 `use_sim_time: false`，通过 config 参数传入。仅把仿真 YAML 的时钟改为 false 不足以完成实机适配；必须同时核实 frame、标定、单位和内参。当前仍缺自身过滤、动态清除等能力，不能把本轮仿真通过等同于实机避障可用。

## 11. 停止与故障排查

停止顺序：记录器/验证脚本 → 监测器与 RViz → mapper → 自己启动的场景。优先在相应终端 Ctrl+C。若 Gazebo 只关了 GUI，gzserver 可能继续运行：

```bash
ss -ltnp 'sport = :11457'
ps -eo pid,comm | rg 'gzserver|gzclient|rviz|robot_state'
```

确认 PID 归属后仅停止自己这组进程；不要执行 `pkill gzserver` 一类会停止其他工作区的全局命令。

| 现象 | 优先检查 | 处理方向 |
| --- | --- | --- |
| 等待相机/TF，无 Mapper ready | topic、CameraInfo、/clock、frame 和时间 | 先恢复输入，别提高 ESDF 目标 |
| depth 高，fusion 低 | tf_missing/stale/overwritten、CPU解码、内参不匹配 | 逐项定位等待/丢帧，禁止用 latest TF 掩盖问题 |
| fusion 高，ESDF 低 | 更新目标、GPU负载、频率预算和地图规模 | 同精度剖析，不靠改粗 voxel 验收 |
| ESDF 高，export=0 | 是否无点云消费者 | 当前设计行为；监测器会触发 observed 导出 |
| export 高，received 低 | QoS、DDS、消息大小、RViz和消费者负载 | 同一 RMW 对照，区分发布调用与实际接收 |
| 灰色大网格 | 是否订阅完整 esdf_3d | 默认显示 observed；完整 unknown 不等于地图实际分配体素 |
| 画面点少或空 | 相机视野、ROI、距离带、有效深度、unknown | 不要把未观察到的空间当自由 |
| 点云拖影/重影 | 旧数据、Decay Time、时间戳 TF、真实标定 | Decay=0，检查位姿/同步，不能只改颜色 |
| 包不存在/旧路径 | source 的 overlay、旧用户链接和实际包前缀 | 按限定脚本重建，核对 ros2 pkg prefix |
| NITROS/GXF/lib not found | 官方依赖/LD_LIBRARY_PATH/ABI | 先恢复匹配原生依赖，不用 pip wheel 冒充 ROS 依赖 |
| 磁盘满/工具无法初始化 | 空间与 inode、活动日志/录包 | 先停止自己持续写入的任务，再清理可确认的生成物；保留源码和证据 |

## 12. 最终交付与判断标准

保留有效配置、源码与依赖版本、原始统计 JSON、运行日志、是否开启 RViz/录包、静止与运动窗口及失败记录。以相同业务精度、范围和消费者对照 workspaces 原生路径，满足地图新鲜度与正确性要求后，才认为原生接入达到实际目标。

当前可运行路径的操作流程已经给出；仓库内原生 C++ 模块接入的最终启动命令、完整接口与性能成绩，必须等实际实现并验收后再补充，不能从本手册的独立官方基线命令推断它已经完成。
