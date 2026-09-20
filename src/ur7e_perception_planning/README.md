# UR7e 在线感知规划

复用 ur7e_nvblox 的时间戳 TF/深度工具和 realtime_nvblox 的原生 GPU 后端，
通过新的原子 PlanningMap 消息将 ESDF 接入 storm_kit SAGE。原包代码不变。

```bash
./build_ur7e_perception_planning.sh
./run_ur7e_perception_planning.sh
# 已有且初始姿态相同的场景：start_scene:=false
# 按需开启 Gazebo 图形窗口：gazebo_gui:=true
```

默认只运行 Gazebo 服务端（headless）和 RViz，不启动 `gzclient`。
这会避开 Gazebo 图形界面加载在线模型列表时出现的 `libcurl (35)/(56)`、
`SSL routines::unexpected eof while reading` 网络错误，同时减少图形渲染负载。
相机仍由 Gazebo 服务端渲染，深度、TF、ESDF 和控制器继续工作。
显式开启 `gazebo_gui:=true` 后仍可能遇到外部模型服务器的网络错误；
本包没有关闭 TLS 证书校验，也没有修改系统网络配置。

whole_control 同一进程运行感知与规划：地图在进程内直接交接，GPU 操作串行，
ROS 接口同时发布无损压缩的完整地图用于观测/记录。系统 ROS 运行 Gazebo/RViz。仅连接 Gazebo 的
`/forward_position_controller/commands`，不要同时运行其他规划器或扫描控制器。

默认采用已验证的高初始观察姿态，不运行 reconstruction 扫描节点。
新包的场景启动器将深度配置为 320×180 / 10 Hz、RGB 为 320×180 / 5 Hz，
关闭未使用的插件点云输出，以减少渲染负载；原场景包文件不变。
启动后等待地图，向初始末端偏移 [-0.02,-0.03,0] m 的目标低速移动。
可发送 base_link 下的小范围返回目标：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link}, pose: {position: {x: 0.22, y: 0.15, z: 0.86}, orientation: {w: 1.0}}}'
ros2 topic echo /perception_planning/status
ros2 service call /perception_planning/enable std_srvs/srv/SetBool '{data: false}'
```

当前 `/target_pose` 使用位置目标；消息中的姿态不会作为到达要求。

## 范围与地图语义

- 仅静态环境、在线深度融合、低速滚动规划；不承诺动态障碍物避障。
- 地图固定 base_link；PlanningMap 的数组按 [z,y,x] 排列，x 最快。`grid.map_arrays()`
  解码消息中的无损 zlib 数据，恢复原生距离、规划距离和自由空间来源三组数组。
- 同一消息内包含原生 nvBlox 距离、保守规划距离、自由空间来源、采集时间和地图版本。
- 未知空间和越界均阻止运动；已观测的深度射线前方作为自由空间证据。
- 相机近场与机器人背面不可见，初始机器人各碰撞球外 **0.12 m** 使用显式初始自由空间先验。
  该先验仅用于随包启动的已检查静态仿真姿态，不能直接用于未知真实环境。
  更换场景/初始姿态须重新确认净空，或将 initial_clearance_m 设为 0（可能无法起步）。
  新观测到的障碍始终覆盖先验。先验不会随机器人运动向未知区域扩展。
- 地面 z=0 为固定支撑平面；墙和球的位置不输入规划器。
- 深度去除机器人/相机/支架返回；无效像素不清除其后的空间。
- 相机/支架加入工具坐标系下的保守碰撞球；非邻接连杆使用球间自碰撞检查。
- SAGE 每轮冻结一份 GPU 地图；独立 CPU 执行器逐段检查最新地图，限制仿真时间下的关节速度。
- 原有不检查碰撞的局部修正关闭。新包在距目标 10 cm 内优先使用阻尼雅可比修正，
  必须通过相同的整臂 ESDF、连续段采样、自碰撞和关节限位检查才可执行。
  修正可行时省去本轮 GPU 优化，给感知留出资源；修正受阻或目标更远时调用 SAGE。
  状态中的 `command_source` 和 `sage_optimizations` 可区分两条实际执行路径。
- 执行检查最小规划净空 1.5 cm，新目标额外保留 1 cm 跟踪余量；这两者之外，
  距离场本身已扣除体素和插值误差。状态/地图/规划过期时冻结最后一条已检查指令。

默认最大关节速度 0.08 rad/s、规划目标周期 0.1 s、执行目标 50 Hz，均不代表实测频率。
静态场景允许图像/地图最多滞后 1.5 s 仿真时间，同时使用 3 s 墙钟地图看门狗；
运动过程中按采集时刻查询 TF。这一时间预算不适用于移动障碍物或高速运动。
规划距离另外扣除了网格插值误差余量；未知边界附近可能停止，需要更多观测。

日志位于 `~/nvblox_output/perception_planning/<时间>/`，含运行解析配置、状态、q_next、
q_cmd、q_actual、指令来源和碰撞检查结果。RViz 显示 ESDF、机械臂/负载碰撞球、目标和 SAGE 候选轨迹。
`/ur7e_nvblox/stats` 记录深度接收、融合与地图发布计数，以及排队、TF、自过滤和融合耗时。

## 验证与故障注入

源 ROS 和工作区后，在 whole_control 中执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -B -m pytest -q src/ur7e_perception_planning/test
# 下条命令会向仿真机械臂发送测试目标，并暂停/恢复感知和重置地图：
python -B src/ur7e_perception_planning/test/validate_sim.py --exercise --faults \
  --output /tmp/perception_planning_validation.json
```

`/perception_planning/mapping_enabled`（SetBool）仅暂停/恢复地图更新，规划端的地图超时停动仍生效。
`/ur7e_nvblox/reset_map`（Trigger）重置地图；初始自由空间先验始终固定在已验证的初始姿态，
不会随当前机械臂姿态扩张。`/perception_planning/enable`（SetBool）控制规划执行。

当前范围是**已观测空间内的小范围低速闭环**。墙内目标会受阻，视野外/未知区域也会受阻；
没有主动探索扫描、完整全局路径搜索、动态障碍消退或任意远目标到达保证。
运行验证结果见 [validation/RESULTS.md](validation/RESULTS.md)。碰撞球近似和离散地图不构成硬件安全保证。
默认 headless 启动的修复及两遍复测见 [validation/HEADLESS_FIX.md](validation/HEADLESS_FIX.md)。
