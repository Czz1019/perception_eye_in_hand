# 0.03 m 体素、Mesh 5 Hz、颜色与 ESDF 30 Hz 复测

日期：2026-11-04。已修改配置、重新安装，实际运行 Gazebo、RViz 与之前的小幅腕部转动，保存原始数据和可核实的窗口截图。

## 1. 本次修改范围

| 配置 | 当前值 |
| --- | ---: |
| voxel_size | 0.03 m |
| 深度融合目标 depth_hz | 30 Hz |
| 颜色融合目标 color_hz | 30 Hz |
| ESDF 更新目标 esdf_hz | 30 Hz |
| ESDF 显示目标 esdf_view_hz | 30 Hz |
| Mesh 目标 mesh_hz | 5 Hz |
| 地图层公共发布目标 layer_hz | 5 Hz |
| 原始输入 | 1280×720 |
| 最大融合距离 | 3 m |
| ESDF 显示距离带 | 0.12 m |

仅修改 [base.yaml](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/nvblox/base.yaml) 的体素、Mesh 与公共发布参数，以及 [mapping.launch.py](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/mapping.launch.py) 的 Mesh/Layer 默认值。颜色和 ESDF 已有30 Hz设置保持。

Mesh 和地图层共享发布调度，所以不能仅降低 mesh_hz 而让 layer_hz 继续30，否则仍可能由高频公共发布触发 Mesh 工作。本次两者一起降低到5 Hz。

本次没有修改 mapper_node.cpp，没有添加统计字段、话题、ESDF算法或新的计数事件。C++源码与安装二进制哈希均与上一轮相同。测试脚本只读取已有信息并计算窗口统计。

## 2. 为什么之前写“ESDF周期”，本次用什么统计

官方源码 [NvbloxNode::processEsdf](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream/nvblox_ros/src/lib/nvblox_node.cpp:767) 原本就记录：

```cpp
timing::Rates::tick("ros/update_esdf");
multi_mapper_->updateEsdf();
```

因此按代码原名应称 **ESDF更新频率**。此前“周期”的用语是为强调它统计进入更新函数的节奏，并非另一个算法事件；不应把它作为新功能名称。本次报告使用官方 `ros/update_esdf` 及适配节点已有 `totals.esdf_updated`。

阶段频率计算为：`(窗口结束esdf_updated - 窗口开始esdf_updated) / 墙钟时间`。没有在运行节点新增频率字段。原始 metrics 里的兼容键仍保持原样，本次没有为了措辞去修改程序接口。

统计边界仍需保留：一次 update 的变化块数量可以不同，甚至没有新增几何；这些计数没有测量 GPU 完成或最终屏幕显示。应称更新频率，但不能据此宣称每秒产生同样数量的全新完整地图。

## 3. 实际测试流程

先启动场景，机械臂到达之前的观察姿态后才启动 mapper。依次记录：Mesh静态、腕部往复20秒、停稳；再开启三维ESDF显示并重复。运动幅度为 wrist_1 ±0.06 rad，没有环绕。

静态窗口约21秒，运动窗口约27秒（包括脚本平滑接近/回位），停稳窗口约13秒。运行真实RViz，RGB、机器人和Mesh均开启，后半段再显示ESDF。频率探针沿用已核验的原始CDR方式，不反序列化数万个Python点对象。

每组仅一次，体素与Mesh频率同时改变，不能将变化单独归因于其中一个参数。测试时存在桌面交互，截图观察角度也有变化；不以逐像素画面对比推断地图精度。

## 4. 更新、显示及融合频率

| 阶段 | 深度融合 Hz | 颜色融合 Hz | ESDF更新 Hz（esdf_updated） | Mesh接收 Hz | ESDF显示接收 Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| Mesh 静态 | 20.01 | 20.01 | 26.33 | 4.71 | 未订阅 |
| Mesh 腕部运动 | 18.24 | 18.32 | 24.63 | 4.68 | 未订阅 |
| Mesh 停稳 | 18.34 | 18.59 | 24.79 | 4.67 | 未订阅 |
| Mesh+ESDF 静态 | 18.29 | 18.19 | 24.43 | 4.59 | 12.84 |
| Mesh+ESDF 腕部运动 | 18.30 | 18.49 | 24.44 | 4.64 | 12.93 |
| Mesh+ESDF 停稳 | 17.78 | 18.41 | 24.64 | 4.63 | 12.14 |

RGB接收约28.3–29.0 Hz，Gazebo实时率约0.999–1.001。ESDF的30 Hz设置是目标，当前实测仍约24–26 Hz；三维显示约12–13 Hz。Mesh降频实际生效，接收频率接近5 Hz。

官方日志现有输出示例，未改字段名称：

```text
NVBlox Rates (in Hz)
namespace/tag - NumSamples (Window Length) - Mean
ros/update_esdf          100      25.1
```

剔除启动时只有一个样本的零值后，官方 `ros/update_esdf` 滚动均值范围为23.6–25.3 Hz。该统计用官方ROS时间与滚动窗口；上表是阶段墙钟窗口的计数差分，窗口不同所以数值不完全相同。

![两轮腕部运动测试频率比较](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/frequency_comparison.png)

### 原有耗时与延迟统计

| 开启ESDF显示阶段 | esdf_process_host_ms 样本均值 | esdf_view_host_ms 样本均值 | 显示样本P95 |
| --- | ---: | ---: | ---: |
| 静态 | 0.48 ms | 10.90 ms | 13.14 ms |
| 腕部运动 | 1.40 ms | 11.15 ms | 13.85 ms |
| 停稳 | 1.25 ms | 11.37 ms | 15.32 ms |

这些是现有metrics中最近一次耗时值的样本统计，不是全部调用逐次分布，也不是GPU完整耗时。官方控制台delay的滚动均值范围：ESDF 41–51 ms，深度18–27 ms，颜色26–31 ms；不是逐帧最大/最小值，也不包含完整屏幕链路。

## 5. ESDF点数与显示外观

| 阶段 | 每条距离带消息点数范围 | 平均点数 |
| --- | ---: | ---: |
| 静态 | 11,377 | 11,377 |
| 腕部运动 | 11,349–11,445 | 11,391.37 |
| 停稳 | 11,392 | 11,392 |

最终快照已有字段：98个分配块、50,176个块内体素槽位、14,808个observed体素、5,913个observed且inside标志体素，最终显示11,392点。槽位包含未知，显示点也不是所有已观测自由空间。

点仍由同一段代码得到：原生ESDF块 → observed且距离合法 → 换算米制有符号距离 → 保留绝对距离不超过12 cm → 原生体素中心输出Marker。没有新增固定空间查询点阵。

3 cm体素下，显示立方体边长为0.95×0.03=2.85 cm，距表面12 cm的带宽约对应4个体素。与原来2 cm相比，点更少、单个方块更大，不能把点数减少当作覆盖更完整。

### 核验通过的真实窗口截图

![本次0.03m体素静态Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/mesh_static.png)

![本次0.03m体素ESDF静态窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/mesh_esdf_static.png)

这两张均已核实为实际RViz窗口；ESDF截图视角经过桌面交互发生旋转，不能与旧报告视角直接逐像素比较。被前台窗口遮挡及未核验的其他窗口截图未保留为证据，原始RGB帧与统计结果不受这一截图问题影响。

界面显示约31 fps仍只是渲染帧率；ESDF数据实际显示约12–13 Hz。

## 6. 与上一轮2 cm、Mesh30 Hz的比较

取两轮均开启Mesh+ESDF的腕部运动阶段：

| 指标 | 上一轮2 cm/Mesh30目标 | 本轮3 cm/Mesh5目标 |
| --- | ---: | ---: |
| 颜色融合 | 17.54 Hz | 18.49 Hz |
| ESDF更新 | 24.38 Hz | 24.44 Hz |
| Mesh接收 | 14.41 Hz | 4.64 Hz |
| ESDF显示接收 | 11.75 Hz | 12.93 Hz |
| 显示导出样本均值 | 26.54 ms | 11.15 ms |
| 显示平均点数 | 31,607.67 | 11,391.37 |

显示点数减少约64.0%，附加导出耗时减少约58.0%，但ESDF更新频率基本未变，显示频率只小幅提高。说明当前瓶颈还受调度和显示节流方式影响，而不仅是数据量。

现有显示代码在发布结束后记录last_view，再等待至少1/30秒。约11 ms处理加33.3 ms等待已经超过33.3 ms的30 Hz周期；与约24 Hz更新时刻对齐后，容易隔次才发布。主tick10 ms、严格限频边界、共享处理链路也仍存在。本次只调参数，没有修改这些代码，因此不能承诺达到30 Hz。

## 7. 重建质量的代价

采用与上一轮相同的局部墙面诊断：在墙体外扩8 cm且z=0.02–0.58 m的邻域内，计算导出Mesh顶点到参考墙面的距离。此方法只评价选中邻域，受顶点分布和邻近非目标几何影响，不是全场景测量精度。

| 指标（最终网格） | 上一轮2 cm | 本轮3 cm |
| --- | ---: | ---: |
| Mesh顶点数 | 4,808 | 2,062 |
| 三角形数 | 7,094 | 2,933 |
| 墙体邻域表面距离P95 | 约9.37 mm | 约35.29 mm |
| 邻域内距参考墙面≤2 cm顶点比例 | 约98% | 84.59% |

本轮静态P95约35.00 mm，运动后约35.29 mm：局部粗化/离群问题在运动前就存在，不能单独归因于运动长尾。减少体素数量使处理更轻，但边缘细节与局部几何质量有明显取舍。

还需注意参数单位：TSDF截断仍为4体素，所以物理范围从8 cm增至12 cm；ESDF site判定距离仍为2体素，从4 cm增至6 cm。它们不是物理障碍膨胀半径，但会影响融合和种子判定。因此这轮改变的不只是显示方块大小。

本次没有单独隔离Mesh降频和体素变化，没有独立验证成像时间真值，也未充分复现此前明显长尾。不能据此认定unknown或残影已解决。

## 8. 运行指令

先停止旧场景与mapper，使用本轮显式配置：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./run.sh gazebo_gui:=true run_rviz:=false \
  mapper_backend:=observed_view esdf_mode:=3d \
  voxel_size:=0.03 max_range_m:=3.0 \
  depth_hz:=30 color_hz:=30 esdf_hz:=30 esdf_view_hz:=30 \
  enable_esdf_view:=true mesh_hz:=5 layer_hz:=5
```

另一个终端直接打开本轮ESDF配置：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
source ./env.sh
rviz2 -d artifacts/voxel003_mesh5_test_20261104/esdf_view.rviz --ros-args -p use_sim_time:=true
```

同环境终端查看现有统计与参数：

```bash
ros2 param get /nvblox_node voxel_size
ros2 param get /nvblox_node integrate_color_rate_hz
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 param get /nvblox_node esdf_view_hz
ros2 param get /nvblox_node publish_layer_rate_hz
ros2 topic echo /nvblox_node/metrics --once
```

小幅视角运动沿用 `../run_ur7e_view_sweep.sh --check-only`，通过后 `../run_ur7e_view_sweep.sh --seconds 20 --amplitude 0.06`，不要同时运行其他关节命令程序。严格复现本报告需先启动场景、完成观察姿态接近，再start_scene:=false启动mapper，参见[上一报告的流程](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/QUALITY_REALTIME_TEST_20261104.md)。

## 9. 验证与原始记录

构建两包成功，现有13项Python检查及C++显示策略检查通过。本轮没有改动或重命名现有ESDF统计接口；现有计数和官方日志都保留。

- [窗口统计与参数命令](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/results.json)
- [原始metrics、时钟和消息记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/samples.json)
- [官方ros/update_esdf与delay日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/mapping.log)
- [运行时参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/runtime_parameters.yaml)
- [局部质量数据](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/quality_metrics.json)、[最终网格](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/mesh_final.ply)
- [测量脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/voxel003_mesh5_test_20261104/run_measurement.py)

自行启动的mapper、RViz和本轮Gazebo进程已停止，11457端口已释放。结束检查还发现一个非本轮PID的gzserver，未擅自停止；因此不宣称桌面环境完全没有其他负载。结果是当前环境下的短时实测，不是独占机器的长期基准。
