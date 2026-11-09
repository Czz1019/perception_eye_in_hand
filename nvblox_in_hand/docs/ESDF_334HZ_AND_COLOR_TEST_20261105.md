# ESDF 33.4 Hz 门限、二维同源配色与三维仿真测试

测试日期：2026-11-05。修改范围仅为 perception_eye_in_hand/nvblox_in_hand；workspaces 官方实现作为只读参考，父工程其他包未在本轮修改。

## 1. 本次结论

将 realtime 档的 update_esdf_rate_hz 从30改为33.4后，官方三维 ESDF **更新入口调用**从24.62提高到32.78 Hz。开启项目诊断导出后，更新调用32.59 Hz、三维点云发布29.44 Hz，深度/颜色实际融合仍约29.74/29.87 Hz。输入640×480、体素0.03m、范围3m保持相同，Mesh/地图层仍5Hz目标。

不过，这次不是“每秒30张全新地图”的证明：诊断点云关联深度时间戳推进只有约19.75 Hz，约32.9%的相邻点云关联相同深度时间戳。调高显示门限至40可让消息达到32.24 Hz，关联时间戳推进仍约19.82 Hz。因此保留33.4作为当前诊断显示默认，不将40作为默认；继续盲目提高显示频率收益有限。

配色与 workspaces 官方二维 ESDF 配置对齐，采用 intensity 距离、固定0–1m、rainbow且不反转；保留3像素点显示。配色不更改 ESDF 值、不去掉内部点，也不补齐 unknown。当前遗留几何质量和盲区问题没有因此解决。

## 2. 33.4 Hz 为什么有效、又有什么局限

只读参考源码：

- /home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp，shouldProcess（约564–572行）、tick中ESDF调度（约625–631行）、processEsdf（约768行）。
- 该分支采用 elapsed > 1 / desired_frequency_hz，执行后将上次时间重置为本次now；主tick约10ms。它是最小间隔门限，不是独立精确30Hz定时器，也没有累积相位补偿。

30Hz要求间隔严格大于33.333ms，30ms的tick不通过，通常到40ms才执行，所以约25Hz。33.4Hz对应29.940ms，名义30ms的tick可通过，因而约33Hz。本轮实测含调度抖动约32.5–32.8Hz。

这里改的是 **ESDF update门限33.4Hz**。深度/颜色融合门限保持之前的40Hz，对30Hz图像时间戳留有余量；并未把相机改为33.4或40Hz。传感器仍约30Hz，两路融合没有因此进一步明显提高。

原生tick内周期ESDF更新与图像到达/融合没有按每帧一一绑定；相机时间戳、ROS仿真时钟、tick和诊断导出的wall-clock门限也并非同一个相位。即使两个平均频率都高，某次调用仍可能发生在没有新深度融合之后；另一次又可能合并多帧深度。诊断导出目前以ESDF调用计数判定版本，而非以新深度时间戳判定版本，因此允许重复关联时间戳的点云。本次没有改动这些C++逻辑。

33.4是当前10ms调度条件下有效的参数尝试，不是任意机器和高负载下的稳定30Hz保证；也不能由调用次数直接得出GPU完成率或地图信息刷新率。

## 3. 修改清单

| 文件（均位于本工程src/nvblox_in_hand） | 修改及目的 |
| --- | --- |
| config/profiles/realtime.yaml | update_esdf_rate_hz 30→33.4，越过名义30ms调度门限；融合40、Mesh/层5不变 |
| config/nvblox/view.yaml | 诊断导出默认5→33.4Hz，让三维显示可跟随核心更新；仍是独立门限 |
| launch/perception/nvblox.launch.py | esdf_view_hz启动默认同步为33.4；官方后端不加载诊断导出参数 |
| rviz/diagnostic.rviz、official_2d.rviz、mesh.rviz、in_hand.rviz | ESDF点云统一距离配色，禁止每帧自动缩放颜色范围 |
| tests/test_launch_contract.py（位于工程根tests） | 原有参数契约断言同步新默认值 |

未改变官方核心、CUDA导出器、采样规则、点云字段、融合分辨率和机器人TF。构建安装内容已同步。

## 4. 配色来源与含义

只读参考：

/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/visualization/realsense_example.rviz，Esdf Slice显示项。

下列7项与参考完全一致，核对证据见palette_reference.json：

```yaml
Color Transformer: Intensity
Channel Name: intensity
Autocompute Intensity Bounds: false
Min Intensity: 0.0
Max Intensity: 1.0
Use rainbow: true
Invert Rainbow: false
```

0附近为暖色，距离增加沿彩虹色变化。负值和大于1m的值只在显示颜色上夹到两端，数据仍保留真实有符号距离（本轮点云最小-2m、最大2m）。Intensity着色时不使用点云rgb字段。内部点、表面site和靠近地面点都可能呈现相近暖色，不能用颜色单独判断inside或unknown；应结合is_inside/observed字段。

本次没有复制官方二维Flat Squares方块样式，三维保持Style=Points、Size(Pixels)=3、Decay Time=0。空间体素精度仍为3cm，屏幕点大小不是精度。三维全体observed体素包含地面附近和内部点，颜色分布不会与一张二维切片完全相同；若想看精确物体表面，应看Mesh。

## 5. 测试方法与条件

独立ROS_DOMAIN_ID=129、Gazebo端口11459；真实Gazebo/RViz/CUDA执行，四轮依次重新创建mapper和地图。保留UR7e+D435场景，机器人先进入同一观察姿态，然后每轮静止约10秒、末端关节小幅摆动并返回约22秒、停止约8秒；没有环绕扫描。截图只捕获本轮RViz窗口。

条件：单相机，depth16UC1、RGB rgb8均640×480，目标30Hz；两路CameraInfo尺寸/K/P匹配；base_link坐标、图像时间机械臂TF；voxel_size=0.03m，最大融合距离3m，static_tsdf，原生3D ESDF；融合门限40Hz，Mesh/地图层5Hz；衰减和半径清图关闭。实时因子约0.999。

核心频率来自现有官方save_timings的ros/depth/integrate、ros/color/integrate、ros/esdf计数差/墙钟区间，不用配置数字替代实测，不新增生产统计字段。点云和Mesh频率来自接收时间。计数快照与阶段端点有少量时间差，不能将临界一两帧差异当成严格丢帧率。2D本轮未重新运行，配色做来源配置核对，运行验证均为3D。

构建完成后离线20项Python检查及既有C++显示策略测试通过，再启动正式采集；CUDA导出器本轮未单独重跑GPU单测，但仿真导出、原生体素中心/有符号距离和24字节点格式核验通过。构建结束前误启的一次预采集已停止，保存在attempt0_before_build并明确排除，未混入下表。

## 6. 实测频率：末端运动阶段

| 轮次 | ESDF/显示门限Hz | 深度融合Hz | 颜色融合Hz | ESDF调用Hz | 3D点云发布Hz | Mesh Hz |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 30Hz对照，诊断后端 | 30 / 30 | 29.62 | 29.57 | 24.62 | 24.58 | 4.75 |
| 官方组件，无项目三维导出 | 33.4 / 不适用 | 29.78 | 29.87 | 32.78 | 不发布 | 4.74 |
| 当前默认诊断显示 | 33.4 / 33.4 | 29.74 | 29.87 | 32.59 | 29.44 | 4.74 |
| 额外显示门限对照 | 33.4 / 40 | 29.76 | 29.67 | 32.53 | 32.24 | 4.74 |

30Hz对照也显式设显示30Hz，避免把原默认5→33.4的显示提速误算成核心算法提升。官方组件轮没有项目esdf_points，不可填0或用Mesh Hz代替三维ESDF更新。

| 诊断轮次 | 点云接收间隔中位数/P95 | 关联深度时间戳推进Hz | 相邻关联时间戳重复比例 |
| --- | --- | ---: | ---: |
| 30/30 | 40.12 / 48.66ms | 19.91 | 19.0% |
| 33.4/33.4 | 31.93 / 40.01ms | 19.75 | 32.9% |
| 33.4/40 | 30.30 / 37.54ms | 19.82 | 38.5% |

关联时间戳由采样时记录的最近/clock减点云age重建，取微秒整数后比较；这衡量点云关联的输入时间是否推进，不是几何内容差异率，也不是对全部融合帧参与ESDF的逐帧追踪。可能在两个导出间合并多次深度融合。因此不能据此断言只有20Hz图像参与融合；官方融合计数显示约30Hz。要确认每帧从融合到GPU ESDF完成需后续逐帧追踪，本轮没有实现。

### 延迟及质量

原有官方ESDF输入年龄统计是滚动均值，运动阶段：对照31–37ms（窗口均值中位数34ms），官方33.4轮33–39ms（34ms），诊断33.4轮33–37ms（35ms）。没有显示出延迟降低。

点云接收时最近仿真时钟减关联输入时间戳：对照P50/P95为33/62ms，诊断33.4为45/73ms，显示40为43/71.7ms。这是近似输入年龄，受/clock接收和回调排队影响，不是屏幕端到端延迟；也不是CUDA事件测得的GPU完成耗时。提高调用次数本轮没有改善这些年龄指标。

诊断33.4轮从静止14747点增加到运动停止18165点，其中inside7272、outside10893、site3321（site与inside/outside不是互斥三分类）。这些是原生已观测体素中心，不是按频率新增点，不是等量自由空间点。观察方向增加后点数才增长。

四轮Mesh已知墙体附近顶点的绝对距离中位数约2.20–2.21mm、P95均37.60mm，未见几何精度改善。此指标仅为墙体邻域、按顶点加权，并非全场景均匀表面积误差；也不能说明覆盖完整。诊断33.4轮有界原生查询221952格，known17357、unknown204595，证明盲区仍大量存在；AABB结果包含大范围未观测空间，不能直接当作障碍物表面重建率。查询/地图保存均安排在频率测量之外。

## 7. 实际效果图

下图为33.4/33.4轮停止后的真实RViz窗口；点来自三维observed体素，颜色按距离映射。视角可用于查看空间关系，不能凭截图认定重建精度或用右下角RViz FPS代表ESDF Hz。

![33.4Hz三维ESDF实际窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf334_color_test_20261105/diagnostic3d_stopped.png)

![相机实际RGB输入](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf334_color_test_20261105/diagnostic3d_stopped_rgb.png)

![频率与体素统计](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf334_color_test_20261105/test_summary.png)

截图角度与界面交互可能不同，不做逐像素质量比较。

## 8. 完整运行和查看命令

### 构建

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./test.sh
```

### 仿真及三维ESDF显示（终端A）

先在旧启动终端Ctrl+C停止同域同端口场景和mapper。不要同时启动多条mapper指令，也不要仅关闭Gazebo窗口。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run.sh gazebo_gui:=true mapper_backend:=diagnostic esdf_mode:=3d   esdf_hz:=33.4 esdf_view_hz:=33.4 esdf_view_mode:=observed
```

RViz自动使用新的距离配色；Move Camera+Orbit可鼠标转动观察。本命令不自动驱动机械臂环绕。已有同环境本项目场景时加start_scene:=false；这不会修改已运行相机的分辨率。若只需官方感知与Mesh，用./run.sh gazebo_gui:=true，ESDF门限同为33.4，但默认官方组件不发布项目三维点云。

### 输入、发布频率及原生统计（终端B）

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
./check_sim.sh
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 param get /nvblox_node esdf_view_hz
ros2 topic list -t
# 分别运行，采样后Ctrl+C
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/color/image_raw --wall-time
ros2 topic hz /nvblox_node/esdf_points --wall-time
ros2 topic hz /nvblox_node/mesh --wall-time
ros2 topic echo /nvblox_node/metrics --once
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath "{file_path: '/tmp/in_hand_rates.txt'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath "{file_path: '/tmp/in_hand_timings.txt'}"
```

核心更新看控制台ros/update_esdf及官方计数，不以topic hz或RViz FPS替代；metrics仅诊断后端存在。本地官方3D不输出static_esdf_pointcloud，需要2D则停止mapper后改esdf_mode:=2d。camera_path/pose话题名称以topic list及[参数话题表](PARAMETERS_AND_TOPICS.md)为准，路径来自机械臂TF，不是额外SLAM。

### 单独比较门限

停掉上一轮mapper再启动，每次只选择一条：

```bash
# 同分辨率/精度的30Hz门限对照
./run.sh mapper_backend:=diagnostic esdf_hz:=30 esdf_view_hz:=30
# 可选：核心33.4，显示40；本轮未见新输入信息率提升
./run.sh mapper_backend:=diagnostic esdf_hz:=33.4 esdf_view_hz:=40
```

复现本次自动采集（会启动自己的场景、RViz和小幅关节运动，只用于空载仿真；须先停止占用11459的旧测试）：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/esdf334_color_test_20261105/run_measurement.py
```

脚本固定写入本次artifact目录，重新执行前先备份该目录以免覆盖原始证据；DISPLAY应匹配当前桌面。本次自动测试已结束，自己启动的Gazebo、mapper、RViz均已退出，11459端口空闲。

## 9. 后续优化方向

1. 以新融合深度/脏块推进为依据调度ESDF，并明确“有新观测的更新”与空调用；考虑按目标时间累积deadline，避免每次now重置导致量化降频。需要先逐帧关联验证，不能仅改计数。
2. 诊断导出按新的输入版本或地图版本发布，减少相同关联时间戳重复；将可视化耗时与原生更新分开测量，按需降频显示，避免全块导出占用计算。
3. 用CUDA事件、输入时间戳和真正渲染时刻分析端到端延迟，定位排队、TF等待、更新及导出各段；本轮现有统计不足以判定GPU瓶颈。
4. 盲区采用多视角采集和显式unknown规划约束；残影先验证图像时刻TF、RGB-D外参/同步、自体遮罩，再按场景决定清图或衰减。提高ESDF门限、换配色都不能修复这些问题。

上述为后续方向，本轮仅完成门限参数和RViz配色修改。

## 10. 原始证据索引

全部位于 /home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf334_color_test_20261105：

- results.json、samples.json：完整阶段、输入、命令与接收样本。
- analysis.json：原有官方计数差、实际频率与滚动年龄统计。
- display_freshness.json、analyze_freshness.py：关联输入时间戳推进与重复率。
- quality_metrics.json、analyze_quality.py：墙体局部几何度量。
- palette_reference.json：workspaces二维配色逐项对照。
- *_rates.txt、*_timings.txt、*_parameters.yaml、mapping_*.log：运行时参数和原生统计；具体文件名以目录为准。
- *_esdf.npy、*_native_esdf.npy、mesh_*.ply及各轮截图：原生点云/网格与真实窗口证据。
- build.log、offline_tests.log、source_manifest.json：构建检查和源码快照哈希。
