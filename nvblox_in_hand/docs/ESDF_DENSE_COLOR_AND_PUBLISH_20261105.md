# 三维ESDF点云发布、密集距离色带与仿真验证

日期：2026-11-05。修改范围为 perception_eye_in_hand/nvblox_in_hand。本次关注发布入口和距离配色；不改变官方融合/ESDF算法、机械臂TF、体素精度和点云采样。

## 1. 为什么会看到“三维点云不发布”

此前报告表格中“官方组件：不发布”指的是项目附加话题 `/nvblox_node/esdf_points`，不是说官方三维ESDF没有计算。两种入口行为如下：

| 入口 | 感知主体 | 三维ESDF保存位置 | 项目三维点云 |
| --- | --- | --- | --- |
| `./run.sh` | 官方 `nvblox::NvbloxNode` 组件 | 官方Mapper的ESDF层 | 不创建该发布器 |
| `./run_esdf.sh`（本次新增） | 项目诊断派生节点，调用官方融合/ESDF主体 | 同类官方Mapper的ESDF层 | 原生体素GPU导出到 `/nvblox_node/esdf_points` |

代码依据：`src/nvblox_in_hand/launch/perception/nvblox.launch.py` 默认 mapper_backend=official，只在diagnostic分支创建mapper_node；`src/nvblox_in_hand/src/mapper_node.cpp` 的构造函数创建 `~/esdf_points` 发布器。

`publishEsdfPoints()` 还有这些必要条件：enable_esdf_view=true、esdf_view_hz>0、有订阅者、esdf_mode=3d、已融合有效深度且调用过ESDF更新、未导出过该调用版本、显示间隔达到要求。因此：

- 不存在发布器：先核对后端与话题名，旧 `/ur7e_nvblox/esdf_3d` 不适用于本工程新入口。
- 存在发布器但暂时无消息：检查深度/CameraInfo、图像时刻TF、/clock、ESDF模式和开关；topic hz会创建订阅，但应等待启动完成。
- 点云有消息但RViz空白：检查Fixed Frame=base_link、Topic=/nvblox_node/esdf_points、显示Enabled以及视角。不要把2D专用static_esdf_pointcloud用于3D模式。

新增run_esdf.sh显式传入diagnostic、3d、enable_esdf_view=true、rviz_mode=diagnostic，避免查看三维点云时误用默认Mesh入口。它复用run.sh，保留其环境加载和所有功能开关，不额外运行第二个Mapper，也不更改默认官方性能基线。附加CLI参数仍可覆盖这些选择，通常无需覆盖。

## 2. 为什么之前颜色单调

上一版直接使用workspaces官方二维配置的固定0～1m色带；它适用于二维切片，但这里显示的是三维全部observed体素，包括内部侧和地面附近体素。

上一轮18165点样本的距离分位数为：P5=-0.06m，P25=0m，P50=0.0424m，P75=0.12m，P90=0.18m，P95=0.21m。因此，大部分距离只用到了原来色带前面很小一段，负值还全部被夹到0端，造成大面积暖色。该问题属于距离值分布与显示范围不匹配，不能靠提高更新频率解决。

## 3. 本次具体修改

- 新增工程根 `run_esdf.sh`：三维点云专用入口，显式打开所需导出与RViz配置。
- 修改 `src/nvblox_in_hand/rviz/{diagnostic,official_2d,mesh,in_hand}.rviz` 中 `/nvblox_node/esdf_points` 显示项：Min Intensity=-0.09m，Max Intensity=0.24m；仍使用Intensity/intensity、固定范围、rainbow、不反转。
- 二维 `/nvblox_node/static_esdf_pointcloud` 色带仍保留0～1m；本次是借鉴距离着色方法，针对三维实测分布收紧范围，不再宣称三维上下界与官方二维完全相同。
- 保留Points、3像素、Decay Time=0；没有增大点数、空间重采样或修改体素大小。这里只让单位距离跨度内的颜色变化更快。

归一化显示关系为 `u=clamp((distance+0.09)/0.33,0,1)`，负距离可占据独立的一段颜色，0对应约27.3%的色带位置，0.03m对应36.4%，0.12m对应63.6%，0.24m对应100%。同样3cm变化，原色带只跨3%，新色带约跨9.1%，提高约3倍。

超出范围只夹显示颜色，点云distance字段不改变；本次仍可返回-2到2m。颜色不是inside或unknown的可靠替代：near-zero/site和内部都需结合is_inside/is_site字段；unknown不会因为换色变成可用距离。规划应读取有效性与原生距离，不能读取截图推断安全性。

## 4. 测试方法

独立domain129、Gazebo端口11459；真实CUDA Mapper和RViz，640×480 RGB-D目标30Hz、voxel_size=0.03m、融合范围3m、融合门限40Hz、ESDF/显示门限33.4Hz、Mesh/层5Hz。两轮均诊断后端，每轮重新建图，采用同一观察姿态、末端小幅摆动并返回，没有环绕。两份RViz配置的Orbit视角一致，区别为三维色带。

wide轮保留0～1m，dense轮采用-0.09～0.24m。采样阶段包括静止约10秒、运动及返回约22秒、停止约8秒；保存真实RViz窗口与RGB截图。运行时断言每轮只有一个三维点云发布器，并核验输入尺寸/内参、原生体素中心、observed/inside标志及24字节点格式。

构建成功后才开始测试。20项既有Python检查及C++显示策略检查通过；本轮没有修改CUDA/C++感知实现，没有单独重跑GPU导出单测。核心更新率按原有官方计数差计算，点云/Mesh按接收时间计算，不使用RViz FPS冒充ESDF频率。

## 5. 实测结果与效果图

两轮均完成，运行时均确认一个esdf_points发布器；没有复现诊断模式缺失发布器的问题。

| 配色 | 深度融合Hz | 颜色融合Hz | ESDF更新调用Hz | 三维点云发布Hz | Mesh Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| 原0～1m色带 | 29.84 | 29.84 | 32.70 | 29.55 | 4.74 |
| 新−0.09～0.24m色带 | 29.75 | 29.94 | 32.76 | 29.44 | 4.74 |

新配色轮停止后18184个原生observed点，inside7289、outside10895、site3323；site与inside/outside不是互斥分类。运动阶段点数从14747增长到18184。原配色轮停止后18178点，两轮小幅差别来自独立仿真采样/建图时序，不能算作换色增加的点。

新配色轮点云关联输入年龄P50约55ms、P95约83ms；原配色约54/83ms。这是接收时最近仿真时钟减点云关联深度时间戳，受/clock与回调排队影响，不是屏幕端到端延迟或GPU完成延迟。没有证据说明换色改善了实时性；频率差异很小。

### 颜色区分的定量核对

在同一份dense轮18184点距离快照上分别应用两种颜色范围，将归一化色带划分12个等宽区间：

| 指标 | 0～1m | −0.09～0.24m |
| --- | ---: | ---: |
| 有点落入的色带区间 | 5/12 | 12/12 |
| 最拥挤区间占全部点的比例 | 60.62% | 18.27% |
| 低于下界的点数 | 3966 | 35 |
| 高于上界的点数 | 27 | 307 |

这是距离映射后的区间占比，不是屏幕可见像素颜色统计；遮挡和相同位置投影仍影响实际观感。新范围使近距离点分散到更多色带区间，同时约1.9%的点落在范围外并夹到端点颜色。负距离不再全部挤到一个端点，但较远距离的细微变化会被压掉，因此不能把这一范围用于完整远场距离比较。

### 实际RViz截图

新配色，三维ESDF全部observed体素：

![新配色三维ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_dense_color_test_20261105/dense_stopped.png)

原配色：

![原配色三维ESDF](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_dense_color_test_20261105/wide_stopped.png)

实际窗口中的颜色层次明显增加。两份初始配置视角和显示开关一致，但采集到的真实窗口中相机视角及Mesh显示开关发生了交互变化，故截图只用于观察各自显示效果，不能逐像素比较，也不据此断言性能差异由配色造成。上面的颜色分布比较在同一份点云上计算，避免这项影响。

构建两包成功，20项Python和既有C++策略检查通过；新增run_esdf.sh经bash语法和ROS launch参数解析验证。仿真使用其等价的显式diagnostic/3d配置，证明发布与导出路径正常。测试结束后本轮Gazebo/Mapper/RViz均已退出，11459端口空闲。


## 6. 操作命令

先在旧运行终端Ctrl+C，停止同域同端口旧场景和mapper；不要同时执行run.sh与run_esdf.sh。

终端A，构建并启动三维ESDF：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_esdf.sh gazebo_gui:=true
```

等价感知配置为 `./run.sh mapper_backend:=diagnostic esdf_mode:=3d enable_esdf_view:=true rviz_mode:=diagnostic`。如果本项目场景已经在同环境运行，追加start_scene:=false，避免重复启动Gazebo。

终端B，加载同一环境并检查：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
./check_sim.sh
ros2 node info /nvblox_node
ros2 topic info /nvblox_node/esdf_points --verbose
ros2 param get /nvblox_node esdf_mode
ros2 param get /nvblox_node enable_esdf_view
ros2 param get /nvblox_node esdf_view_hz
# 分别运行，每项采样后Ctrl+C
ros2 topic hz /camera/depth/image_rect_raw --wall-time
ros2 topic hz /nvblox_node/esdf_points --wall-time
ros2 topic hz /nvblox_node/mesh --wall-time
ros2 topic echo /nvblox_node/metrics --once
```

正确时应有一个esdf_points发布器，类型sensor_msgs/msg/PointCloud2。默认官方组件没有metrics和esdf_points，不应在另一终端再启动第二个同名节点“补齐”话题。

已启动诊断Mapper但关闭了RViz时，可单独运行：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
./rviz.sh diagnostic
```

在RViz中使用Move Camera/Orbit拖动查看；Diagnostic ESDF显示项可继续调整Min/Max Intensity以适合其他场景。变更后的范围不是通用最优值：如果要比较较远自由空间距离，应增大Max；若仅看障碍物真实表面，应切到Mesh，不要把全部ESDF体素当作表面点云。

## 7. 结果的边界和后续方向

颜色更丰富只说明同一距离范围的显示分辨率提高，不代表建图精度、覆盖或安全性改善。相机盲区仍是unknown，机械臂自身进入视场的残留、TF时间误差或历史融合残影也不会因换色消失。

33.4Hz解决一部分tick量化限频，仍不能把更新调用次数当作新观测信息率；上次已发现点云关联输入时间戳会重复。本次没有修改版本判定或调度逻辑，因此不能声称已修复该问题。

后续需按新融合输入/脏块跟踪实际ESDF更新，区分空调用、合并多帧和重复导出；用逐帧关联及GPU完成测量定位延迟，再考虑对图像时刻TF、机械臂自体遮罩和历史清理进行优化。

## 8. 原始证据

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_dense_color_test_20261105`。

results.json/samples.json保存阶段与原始接收记录；analysis.json为实际频率与距离/色带分布；wide/dense_runtime_parameters.yaml为运行时参数；mapping_*.log及*_timings.txt、*_rates.txt为原有官方日志统计；*_esdf.npy为原生体素快照；各阶段png为实际窗口与RGB输入；build.log/offline_tests.log记录检查；source_manifest.json记录修改和测试源码哈希。

重新运行该目录run_measurement.py会覆盖同名证据，应先备份。脚本只允许domain129，启动自己的Gazebo、Mapper、RViz及仿真末端小幅运动；不要用于实机。
