# 参考 workspaces 与官方文档的 ESDF 点云优化

后续版本提示：本文记录本次示例主体重构之前的状态，当前默认已改为官方组件，诊断点云改为24字节并默认observed。新版代码、参数和验证边界见 [workspaces示例重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md)。

后续语义纠正：本机workspaces的EsdfVoxelFilter只有定义，未找到调用点，不能据此推断官方完整距离场输出应只保留inside。当前默认surface隐藏outside自由空间，属于不满足完整距离场需求的适配选择；其性能改善也不代表地图质量改善。详见 [内部点与简化范围审计](ESDF_SEMANTICS_AND_SIMPLIFICATION_AUDIT_20261104.md)。以下保留此前修改及实测过程记录。

日期：2026-11-04。已完成代码修改、构建、GPU正确性检查、仿真实测与真实RViz窗口截图。本轮保持0.03 m体素、原始1280×720输入、3 m融合范围、颜色/ESDF/显示30 Hz目标、Mesh/Layer 5 Hz目标。

## 1. 为什么之前是方块，是否真的用了官方代码

此前三维显示是项目适配节点自行发布的 `visualization_msgs/Marker::CUBE_LIST`，方块边长为0.95×voxel_size。0.03 m体素会画成2.85 cm方块；再把表面两侧12 cm距离带一起画出，就形成截图中的厚方块壳。

这不是workspaces默认的PointCloud2切片显示。此前若把这条项目附加显示路径笼统称为“官方显示效果”，是不准确的。底层复用官方算法与上层显示适配必须分开说明。

实际来源核对：

| 部分 | 实际实现/来源 |
| --- | --- |
| 深度TSDF、颜色融合、三维ESDF、Mesh、地图服务 | workspaces安装的 libnvblox_ros_lib.so 和 libnvblox_lib.so |
| 图像时间TF与官方调度 | 官方 NvbloxNode / Transformer |
| 此前的三维CUBE_LIST距离带 | 项目 mapper_node.cpp 附加实现，已移除 |
| 本次三维PointCloud2与GPU紧凑导出 | 项目适配实现，筛选参考workspaces原生ESDF状态，不冒充官方原封不动的三维点云节点 |
| 官方二维ESDF切片 | 原有 static_esdf_pointcloud，保留，不作为新三维话题的别名 |

[实际链接记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/runtime_libraries.txt) 确认两个核心库来自 `/home/czz/workspaces/isaac_ros-dev/install/nvblox_ros/lib/`。没有改动workspaces或父工程。

## 2. 重新参考的具体内容

本机 [layer_publishing.cpp](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:186) 的 EsdfVoxelFilter 条件为 `voxel.is_inside && voxel.observed`。本次默认surface模式采用这一条件，另加有限距离检查。这里的surface名称沿用其表面显示意图，实际是已观测inside体素样本，不是Mesh零等值面的精确顶点。

本机 [realsense_example.rviz](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/visualization/realsense_example.rviz:187) 用PointCloud2和intensity显示ESDF切片。其切片本身是二维；原生体素插件另有方块材质和fallback点材质，所以“官方永远只显示点”也不准确，显示类型取决于所选图层。

官方接口文档明确：`static_esdf_pointcloud` 是二维静态ESDF点云，intensity为到最近障碍的米制距离；颜色输入用于Mesh着色。[官方接口说明](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)

官方参数文档将体素尺寸、ESDF更新频率和地图层发布频率分别定义。点大小、体素精度和更新频率不能混用。[官方参数说明](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)

网页版本与本地ROS迁移版可能不同，实际接口以本地代码为准。[参考文件哈希与来源](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/references.json) 已保存。

## 3. 已完成的修改

### 3.1 用点云替代CUBE_LIST

新话题为 `/nvblox_node/esdf_points`，类型 sensor_msgs/PointCloud2。每点20字节，字段为x/y/z、distance、intensity及rgb；distance与intensity共享同一米制浮点值。

RViz默认Style=Points、Size(Pixels)=3，以intensity着色，Decay Time=0，每帧替换显示，不累计旧消息。显示大小不再等于3 cm实体方块，但点的位置和数量仍受原生体素精度约束，没有插值伪造更多观测。

旧 `/esdf_native_band` Marker 已不再发布。历史artifacts里的方块RViz配置保留为历史记录，不会自动变成新显示；当前入口使用新配置。

### 3.2 先在GPU筛选，再读回结果

此前：读取全部分配ESDF块并同步 → CPU遍历所有体素 → 筛选 → 构造Point和Color对象数组 → CUBE_LIST。

现在：枚举实际分配块 → GPU检查observed/inside/距离并紧凑输出 → 只读回入选点 → 连续二进制PointCloud2。

每批最多512块，复用GPU缓冲。不生成固定ROI查询点阵，不把unknown填成自由，不重新实现TSDF或ESDF数学算法。GPU仍会访问全部分配块，而且有计数/数据同步；这不是已经实现了变化块增量或完全异步的导出。

### 3.3 修正显示节流

ESDF更新仍由官方processEsdf处理，现有esdf_updated统计保留。显示导出移到官方tick完成之后，按开始到开始计时，并且同一ESDF更新版本不重复导出。

此前last_view在发布结束后更新，下一次还要额外等1/30秒，处理耗时被叠加到周期上。现在从开始时计时，避免该额外等待。没有新增独立并发线程读取正在变化的地图，仍保持同一核心处理链路与CUDA流次序。

点云header使用对应ESDF更新时最新融合深度的时间戳，而不是用发布now掩盖数据年龄；这不代表已经测得完整屏幕延迟。

### 3.4 三种选择模式及独立显示配置

| 参数/入口 | 用途 |
| --- | --- |
| esdf_view_mode:=surface，默认 | observed && inside，参考workspaces原生ESDF筛选 |
| esdf_view_mode:=band | observed且合法距离，绝对值≤esdf_view_band_m，默认12 cm |
| esdf_view_mode:=observed | 所有已观测有限距离，适合检查自由空间；不包含unknown |
| ./rviz.sh 或 ./rviz.sh esdf | 新三维点云，默认打开 |
| ./rviz.sh mesh | 原生RGB Mesh，避免表面遮挡内部点影响判断 |
| ./rviz.sh 2d | 配合esdf_mode:=2d查看官方二维切片 |

默认点云着色是距离伪彩色，不是物体RGB。物体外观应看Mesh或原始RGB图像。三维模式和官方二维切片仍是不同功能。

## 4. 代码导航

- [esdf_pointcloud_exporter.hpp](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/include/nvblox_in_hand/esdf_pointcloud_exporter.hpp)：点布局、选择模式、复用缓冲。
- [esdf_pointcloud_exporter.cu](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu)：GPU筛选、原生中心坐标、紧凑读回。
- [mapper_node.cpp](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp)：调用官方更新，PointCloud2打包与节流，保留已有统计字段。
- [mapping.launch.py](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/mapping.launch.py)：模式参数与验证，official后端仍直接运行官方节点。
- [in_hand.rviz](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/rviz/in_hand.rviz)：三维像素点显示；mesh.rviz及official_2d.rviz为独立查看配置。

## 5. 实际测试条件

使用独立ROS域129和Gazebo端口11459，避免与其他运行串话。先让机械臂到之前的观察姿态，再启动mapper；每种模式从新地图开始，先静态、再wrist_1 ±0.06 rad往复20秒、最后停稳。没有环绕扫描。

所有几何参数与上一轮3 cm测试一致。band模式用于检查相同12 cm筛选的性能；surface模式用于检查默认显示，不能用更少的surface点数冒充相同band工作量的加速。

测量节点仍订阅Mesh，保证5 Hz的Mesh需求；真实RViz用于查看点云，桌面交互中部分截图也打开Mesh或改变视角。没有统一录制输入、随机化顺序或独占桌面资源，因此是实际短时验证，不是严格因果基准。

## 6. 实测结果

### 6.1 腕部运动阶段

| 指标 | 旧CUBE_LIST，12 cm带 | 新PointCloud2，12 cm带 | 新PointCloud2，surface |
| --- | ---: | ---: | ---: |
| 深度融合 | 18.30 Hz | 17.98 Hz | 18.15 Hz |
| 颜色融合 | 18.49 Hz | 18.29 Hz | 18.42 Hz |
| ESDF更新（现有esdf_updated） | 24.44 Hz | 24.47 Hz | 24.50 Hz |
| 显示消息 | 12.93 Hz | 24.40 Hz | 24.43 Hz |
| Mesh消息 | 4.64 Hz | 4.75 Hz | 4.74 Hz |
| 附加显示耗时样本均值 | 11.15 ms | 0.57 ms | 0.34 ms |
| 附加显示耗时样本P95 | 13.85 ms | 1.46 ms | 0.50 ms |

相同band选择下，附加显示样本均值降低约94.9%，消息频率接近ESDF更新频率。不是靠缩小体素或把band改成surface才得到这一提升。但两轮非同一录制输入，百分比只是本次对照结果，不保证所有场景都有相同比例。

![实际更新与显示开销对比](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/optimization_comparison.png)

### 6.2 点数

| 模式 | 运动时每帧点数 | 停稳后每帧点数 |
| --- | ---: | ---: |
| band | 9,567–11,434 | 11,376 |
| surface | 4,869–5,914 | 5,904 |

上一轮band停稳为11,392点，本轮11,376点，差异约0.14%，支持“相同距离带导出没有靠大量删点获得更快显示”。运动初期本轮地图还在扩展，因此运动平均点数与上一轮并不完全一致。

surface最终对应14,796个observed体素中5,904个inside标志体素，并非把地图改小；只是默认只显示这一集合，其他有效自由体素仍在地图中。需要自由空间距离带时使用band或observed。

PointCloud2每点20字节，band停稳点数据约227,520字节；此前CUBE_LIST同量级点的Point+Color数组每点至少40字节，尚未计协议开销。CPU消息构造和传输负载也因此降低。

### 6.3 更新与延迟边界

官方日志仍叫ros/update_esdf，band和surface滚动均值约23.8–25.3 Hz。正式统计仍来自已有esdf_updated和官方日志，没有新造一个“ESDF周期”指标。

本次显示目标仍为30 Hz，实际约24–25 Hz；核心10 ms tick、限频边界和输入融合节奏尚未修改，所以不宣称已经稳定达到30 Hz。显示调用同步耗时也不是单独的GPU算法完成时间。

按接收时最近一条/clock与点云header估算，运动阶段消息数据年龄中位约48–49 ms、P95约86–90 ms。这是异步时钟采样下的诊断估计，不包含RViz绘制与屏幕扫描，不能作为严格端到端延迟承诺。

## 7. 真实窗口截图

### 相同12 cm距离带，现在以点显示

![实际三维距离带点云窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/band_wrist_motion.png)

### 默认surface点模式

![实际surface点模式窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/surface_stopped.png)

截图直接读取本任务RViz窗口的后台像素，没有把其他前台应用截图当作结果，也没有用AI或绘图伪造重建。截图视角和窗口大小有桌面交互变化，不能逐像素对比推断精度。窗口约31 fps只是界面绘制频率，地图显示频率以上表消息计数为准。

## 8. 哪些问题仍然没有解决

本轮改善了点云展示、筛选语义、导出负载及显示节流，没有修改TSDF/ESDF几何算法、机械臂手眼、相机采样时间或机器人自过滤。因此：

- 3 cm体素的几何分辨率仍是3 cm，像素点更细不等于地图精度提高。
- 原始RGB仍有机器人入图、目标视角不理想的问题，死角unknown不会因点化而补齐。
- 当前静态TSDF的历史表面保留策略未改变；此前运动残影不能宣称已修复。
- GPU导出仍遍历全部分配块，大地图下应进一步做变化块/有界快照，而不是无限提高显示频率。

后续质量优化应先验证图像采样时间与机械臂TF、接入按图像时刻的机器人自过滤、改善目标视角；需要更细几何时再评估2 cm或1 cm，不能只调整RViz点尺寸。当前版本这些精度档的完整性能还需单独验收。

## 9. 新版本运行指令

停止旧入口后，在常规域127启动：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./build.sh
./run.sh gazebo_gui:=true mapper_backend:=observed_view \
  voxel_size:=0.03 color_hz:=30 esdf_hz:=30 esdf_view_hz:=30 \
  mesh_hz:=5 layer_hz:=5 esdf_view_mode:=surface
```

当前默认RViz已打开新点云。若场景已经运行，另开mapper需start_scene:=false，避免重复启动。只打开显示可用 `./rviz.sh`；看RGB Mesh用 `./rviz.sh mesh`。不要继续加载历史artifacts目录中的CUBE_LIST配置。

查看同样12 cm距离带，重启mapper时将模式换为band：

```bash
./run.sh esdf_view_mode:=band esdf_view_band_m:=0.12
```

核对新话题：

```bash
source ./env.sh
ros2 topic info /nvblox_node/esdf_points --verbose
ros2 topic hz /nvblox_node/esdf_points
ros2 topic echo /nvblox_node/metrics --once
```

需要官方二维切片时单独运行：

```bash
./run.sh mapper_backend:=official esdf_mode:=2d run_rviz:=false
# 另一个同环境终端：./rviz.sh 2d
```

二维入口是保留功能及新增查看配置，本轮性能实测对象是三维band/surface，不把二维未测结果混入表格。

## 10. 验证和证据

- 默认构建和最终安装两包成功，14项Python离线检查通过，既有C++显示策略检查通过。
- 新GPU测试核对surface/band/observed、unknown/非法值剔除、负块中心、距离符号、空地图与非法参数；已执行通过。
- 运行点云检查通过：point_step=20、数组长度一致、distance/intensity字段一致、原生体素中心、距离有限和模式选择正确。
- GPU测试最初直接调用底层模板分配块遭遇本机CUDA模板ABI链接差异；改为通过已安装官方Mapper创建测试块后通过，没有修改官方库。
- 首轮截图后端不兼容的尝试保存在attempt1_capture_backend_failure，未混入完整测试数据；修正后的窗口后台截图已核验。

原始资料：

- [完整结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/results.json)、[原始采样](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/samples.json)
- [band官方日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/mapping_band.log)、[surface官方日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/mapping_surface.log)
- [band运行参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/runtime_parameters_band.yaml)、[surface运行参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/runtime_parameters_surface.yaml)
- [GPU测试](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/gpu_export_test.log)、[最终离线检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/final_offline_tests.log)
- [测试脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_points_optimization_20261104/run_measurement.py)

本轮独立域测试启动的进程已停止，11459端口释放；非本轮的其他进程未操作。结果不代表实机、大地图、完整覆盖或严格屏幕端到端延迟已验收。
