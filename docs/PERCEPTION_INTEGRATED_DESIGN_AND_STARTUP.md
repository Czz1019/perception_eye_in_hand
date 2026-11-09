# perception 项目内感知实现：目标重述、结构与统一操作说明

> **sparse-v2 当前状态：** 已修复测试工具的 Thrust 链接错误及构建过期检查，生产目标和可选工具均已编译、链接、安装成功；**未执行工具测试，未启动新版感知或仿真**。下一步按 [构建修复与仿真操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 启动应用，再查看 Topic。算法/接口说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧性能和截图不能作为 v2 验收结果。

日期：2026-10-27。本说明针对“不是改 workspaces，而是在 perception_eye_in_hand 中实现相应感知效果”的要求，以及本次场景启动报错。此前将“接入一个 C++ mapper”作为主要操作说明，未充分呈现项目的完整感知应用入口；本次已补上统一入口和启动验证。

## 1. 目标重新明确

交付对象是 `/home/czz/perception_eye_in_hand`。目标是在项目已有机械臂、相机、手眼配置和实际工作范围下，完成图像时刻位姿驱动的实时 TSDF/三维 ESDF，并把观测、未知、导出和显示分开，降低实际处理延迟。参考 workspaces 官方 ROS 实现的代码逻辑，而不是要求用户切换到 workspaces 中运行另一个应用。

“原生 C++ mapper”指**本项目内部的感知建图节点**，不是独立的用户任务或 workspaces 程序。mapper 是建图组件名称，C++ 是实现方式。图像和 TF 必须进入某个建图节点才能产生 ESDF；原先的 Python tf_mapper 也是同类组件。

因此，合理的使用方式是启动“perception 感知应用”，由它组织场景、输入、建图和显示，而不是要求用户理解并手动拼装多个底层模块。

## 2. 本次项目结构和统一入口

新增入口：[run_ur7e_perception.sh](/home/czz/perception_eye_in_hand/run_ur7e_perception.sh)。统一 launch 位于本仓库：[perception.launch.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/launch/perception.launch.py)。

```text
perception_eye_in_hand/run_ur7e_perception.sh
    └── 本项目 perception.launch.py
        ├── 本项目 ur7e_d435_gazebo：场景、机器人、相机、关节状态与 TF
        ├── 本项目 depth_adapter：统一深度图、内参、单位与时间戳
        ├── 本项目 NativeMapper：融合、三维 ESDF、原生导出、版本与统计
        └── 本项目 RViz 配置：已观测距离体素、相机 pose
```

统一入口默认启动全部组件，Ctrl+C 由 ROS launch 负责退出该次启动的组件。公共输入或 mapper 异常退出时，统一应用会关闭，避免留下只有部分组件的假运行状态。启动场景之前检查 Gazebo master 端口；占用时提前失败，不启动第二个输入/mapper。它不自动杀死已有场景。

旧的分终端脚本保留用于调试。`run_ur7e_esdf.sh` 仍是 Python 兼容/对照入口，未自动切到 C++。已有规划、auto 和真机应用未全部迁移；统一入口也不自动执行机械臂扫描。

## 3. 参考官方逻辑后，哪些行为已经在 perception 中实现

| 应用行为 | 本项目实现 | 意义 |
| --- | --- | --- |
| 深度与机械臂位姿同步 | 公共输入保留图像 stamp，官方模块按图像时刻查询 TF，应用校验 optical frame 和帧龄 | 不使用“最新 TF”替代曝光时刻位姿 |
| TSDF → 三维 ESDF | 本仓库 NativeMapper 调用官方 C++ 模块 | 使用官方距离传播语义，没有重新用占用点拟合一个假 ESDF |
| 原生体素中心与精度关联 | 本仓库 block_export 调用原生层到网格模板 | 不由独立固定查询间距决定所有点数 |
| 按需导出 | 检查订阅、map_version、源图时间 | 无消费者不导出，不重复旧地图冒充高 Hz |
| 未知/自由/障碍区分 | 默认显示 observed 距离带，完整网格保留未知 | 不把大量未知点都画成灰色；仍保留建图需要的自由空间 |
| 实时调度 | steady clock 调度，保留 ROS 图像与 TF 时间，短输入队列 | 避免约 10 Hz 仿真 clock 的周期门槛限制 |
| 真实指标 | 实际融合、ESDF、导出计数及图像 stamp/版本，完成时延和观察器配对 | 区分地图更新 Hz、消息 Hz 与 RViz FPS |

底层第三方依赖仍来自 `/home/czz/workspaces/isaac_ros-dev/install`。本项目节点、配置、导出、启动和统计均在 perception 中；运行统一入口不需要用户切换到 workspaces 启动应用。它仍不等于“已把 NVIDIA 全部源码及依赖内置到 perception”，换机器需要匹配依赖。若要求彻底脱离该目录部署，还需要单独整理第三方安装前缀/打包，本轮未将其冒充完成。

## 4. 为什么本次命令报错，以及已修复什么

### 4.1 多条 local_setup.bash not found

原场景脚本 source 了整个 `/home/czz/perception_eye_in_hand/install/local_setup.bash`。安装索引还包含历史规划和真机包，但这些包的部分安装文件缺失，所以出现无关包提示。此提示不意味着 ESDF 算法失败。

已修改 [run_ur7e_d435_scene.sh](/home/czz/perception_eye_in_hand/run_ur7e_d435_scene.sh)，只 source ROS 和当前场景包的 `package.bash`；统一入口也只加载场景包和本项目原生包，不加载整个旧 install 索引。已实际执行两个入口的参数检查，不再出现上述无关缺失提示。没有删除用户源码或强制重建全部规划包。

### 4.2 Gazebo master 11457 已占用

用户报错时，端口由上一轮保留的测试场景占用，实际 gzserver PID 为 28497。此前只说明“不重复启动”而保留后台实例，导致按文档重新启动时冲突。该旧测试实例已按域/命令/端口归属清理，没有全局 pkill。

若已有场景，应该复用它，或者退出其 launch 后重新启动；关闭 Gazebo 窗口不保证 gzserver 已退出。端口占用时的拒绝逻辑保留，避免两个场景同时争用 master。端口检查本身不是性能问题，也不是 C++ mapper 导致的。

### 4.3 统一启动遇到 DDS participant 索引不足

本次完整应用实测发现，安装版本 Cyclone DDS 的自动索引上限默认为 9，即 10 个候选索引。Gazebo 临时生成节点、控制器 spawner、RViz、mapper 和诊断工具同时启动，可能报 `Failed to find a free participant index`。

新增项目内 [cyclonedds_local.xml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml)，将自动索引上限设为 120。统一脚本仅在使用 Cyclone 且用户没有设置 CYCLONEDDS_URI 时采用它；不覆盖用户已有 DDS 配置。这是通信启动配置，不改变地图体素、输入分辨率、融合算法或导出范围。

另外修复公共深度组件收到 shutdown 后再次 shutdown 的退出异常。退出实测还发现 ros2 run 包装层可能留下 mapper 子进程，因此本项目脚本现直接 exec 本仓库安装的可执行文件，让 launch 信号直接到达 mapper。Gazebo 在一次 Ctrl+C 退出时出现 -11；这是仍需记录的仿真退出异常，不能宣称所有组件均已无异常退出。

## 5. 推荐操作：一条命令启动项目感知

### 5.1 构建

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./build_ur7e_native.sh
```

公共深度脚本直接读取本仓库 Python 源码，不要求为了此次统一入口再构建全部 Python 包。只有旧 Python 后端对照需要此前的 `build_ur7e_esdf.sh`。

### 5.2 启动

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_perception.sh gazebo_gui:=true
```

这一个入口会启动场景、公共深度、本项目 ESDF 和 RViz；**不要再另外运行 run_ur7e_d435_scene.sh、run_ur7e_common_depth.sh 或 run_ur7e_native.sh**。不要同时启动旧 Python mapper。

不需要 Gazebo GUI 时：`./run_ur7e_perception.sh gazebo_gui:=false`。不需要 RViz 时可加 `run_rviz:=false`。启动后保持终端运行，退出用 Ctrl+C，并等待子进程结束。

### 5.3 只复用已经存在的场景

先确认同域场景和 TF 正常，而且没有另一个 `/ur7e_nvblox` mapper 或公共输入组件重复运行，再用：

```bash
./run_ur7e_perception.sh start_scene:=false
```

它只启动输入、感知和 RViz，不会创建/关闭原先场景，也不会补开原场景的 Gazebo GUI。需要显示现有 Gazebo 时，可在同一 master 环境的另一个终端运行 `gzclient`。不要仅为避开端口报错随意更换 master：那会创建另一套场景，而不是复用已有场景。

### 5.4 查看真实频率

另开终端：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox:${PYTHONPATH:-}
/usr/bin/python3 -B -m ur7e_nvblox.monitor
```

检查 fusion、ESDF、export、received 和 unique_image_stamps，而不只看 RViz 右下角 FPS。

## 6. 启动后 points=0 不一定是程序失败

本次从原始初始姿态重新启动时，融合、ESDF 和版本都在持续增加，但观测距离带内 points=0。初始相机视线与指定导出 ROI 不一定有有效的表面交集；此前截图/主对照使用的是测试观察姿态。**不能把测试姿态的效果图当成任意初始姿态都会自动看到相同地图。**

保留原有初始关节配置、手眼外参、3 cm 精度和 ROI；不为制造画面擅自放大范围或修改相机外参。手动移动仿真机械臂，使相机朝向目标工作区后再观察。已验证的测试观察关节角为：

| 关节（控制器顺序） | 角度 rad |
| --- | --- |
| shoulder_pan_joint | 0.0 |
| shoulder_lift_joint | -1.57 |
| elbow_joint | 1.1 |
| wrist_1_joint | -0.90879335 |
| wrist_2_joint | -3.21580312 |
| wrist_3_joint | -0.69520875 |

正常统一启动不自动移动机械臂。需要复现实验时，在上述监控终端环境中另外加入实时库源码路径并运行既有仿真验收脚本；脚本会平滑移动到观察姿态并小幅摆动，只允许测试域 127：

```bash
export PYTHONPATH=/home/czz/perception_eye_in_hand/src/realtime_nvblox:${PYTHONPATH:-}
mkdir -p /home/czz/perception_eye_in_hand/artifacts/my_perception_smoke
/usr/bin/python3 -B /home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py \
  --external --common-input --mode native --voxel-size 0.03 \
  --seconds 5 --exercise-simulation \
  --output /home/czz/perception_eye_in_hand/artifacts/my_perception_smoke/result.json
```

不要对物理机器人执行此仿真运动测试。GUI 可以保持，用于功能检查；GUI 开启时的短测试不是正式性能基线。若测试 `--lifecycle` 的无消费者条件，必须先关闭其他点云消费者，包括 RViz 和监控。

## 7. “官方效果”与当前完成程度

官方常展示表面 Mesh、距离切片或地图层；当前默认展示已观测三维 ESDF 距离带。两种视觉对象不能仅凭截图点数认定算法不一致。自由空间里的有效距离体素并不都是浪费；未知点的大量显示才应从默认可视化中去掉。

本机官方 Mesh 插件缺少 GLSL150 资源，默认不加载；这一部分尚未完成与官方 Mesh 画面一致的显示验收。也没有执行无修改官方 standalone 节点的同帧回放对照，因此不能宣称所有官方效果已经逐项完全复现。

已经完成的是：本仓库内感知逻辑改造、3 cm 同输入/同范围的延迟对照、实际几何验证，以及本次增加的统一应用入口。早期正式对照的中位延迟从 Python 约 13.55/15.61 ms 降到 C++ 约 2.43/2.71 ms，输出约 18 Hz，输入受限，未证明稳定 30 Hz。完整数据和限制见 [接入报告](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md)。

本轮统一入口另做短时 GUI 功能验证，`PASS`：静止/运动阶段各约 5 秒，实际接收约 17.14/18.37 Hz，输入到点云 P50 约 2.36/3.05 ms；静止 9,130 点，运动 9,130～10,655 点，346 次 TF 检查最大矩阵误差约 1.58×10⁻⁷。这是短时功能结果，不替代正式基线，也不能用于证明长期稳定性。

结果位于 `/home/czz/perception_eye_in_hand/artifacts/perception_bringup_20261027`；不替换或混入早期关闭 GUI 的正式对照。

下一阶段若要求覆盖完整实际应用，应分别验收：真实相机/TF 延迟与自体污染、动态残影、规划消费者的网格/未知值契约、长期地图增长，以及官方可视化依赖。它们不应被“已接入 C++”一句话掩盖。

本次统一应用实际画面：[RViz 截图](/home/czz/perception_eye_in_hand/artifacts/perception_bringup_20261027/rviz_unified.png)。

本次测试结束后已停止测试应用，并检查 11457 端口释放、mapper 无残留。退出修复后的 mapper/公共输入单独联合启动与 Ctrl+C 退出检查均正常；完整 GUI 场景曾出现的 Gazebo 退出异常保留在证据日志中，不隐藏为“全部无异常”。
